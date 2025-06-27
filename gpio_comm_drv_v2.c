/**
 * @file gpio_comm_drv.c
 * @brief 5-wire 멀티-디바이스 P2P GPIO 통신 드라이버
 * @author HJH (Original Concept), Gemini (Implemented)
 * @version 2.1
 *
 * @note
 * - 이 드라이버는 1개의 제어핀과 4개의 데이터핀을 공유하는 P2P 통신 프로토콜을 구현합니다.
 * - 사용법:
 * 1. 드라이버 로드: insmod gpio_comm_drv.ko
 * 2. 디바이스 생성 (sysfs):
 * echo "my_dev1,rw,17,27,22,23,24" > /sys/class/gpio_comm/export
 * (이름, 모드(rw/r), 제어핀 bcm, 데이터핀1 bcm, 데이터핀2 bcm, 데이터핀3 bcm, 데이터핀4 bcm)
 * 3. 생성된 디바이스 노드(/dev/my_dev1)를 통해 read/write 수행
 */

 #include <linux/module.h>
 #include <linux/init.h>
 #include <linux/fs.h>
 #include <linux/cdev.h>
 #include <linux/device.h>
 #include <linux/gpio/consumer.h>
 #include <linux/interrupt.h>
 #include <linux/slab.h>
 #include <linux/uaccess.h>
 #include <linux/delay.h>
 #include <linux/ktime.h>
 #include <linux/wait.h>
 #include <linux/spinlock.h>
 #include <linux/crc16.h>
 

 
 // =================================================================
 // 1. 상수 및 매크로 정의
 // =================================================================
 
 #define CLASS_NAME "gpio_comm"
 #define DRIVER_NAME "gpio_comm_drv"

 #define MAX_GPIO_BCM 30

 #define MAX_DEVICES 10
 #define MAX_NAME_LEN 20
 #define NUM_DATA_PINS 4
 
 #define RX_BUFFER_SIZE 256
 #define CLOCK_DELAY_US 10000 // 기본 클럭 간격 (us)
 
 // ktime을 마이크로초(us) 단위로 사용
 #define NORMAL_CLK_MAX_US (CLOCK_DELAY_US * 15LL / 10)    // 1.5배까지는 일반 클럭으로 간주
 #define SLOW_CLK_MIN_US   (CLOCK_DELAY_US * 2LL)          // 2.0배부터는 느린 클럭(EOT)
 #define TIMEOUT_MIN_US    (CLOCK_DELAY_US * 5LL)          // 5.0배 이상은 타임아웃
 
// 라즈베리 파이의 GPIO 컨트롤러(BCM2835)의 기본 번호. 커널 내부에서 GPIO를 식별할 때 사용.
// `gpioinfo` 명령어로 실제 base를 확인해야 할 수 있음.
#define GPIOCHIP_BASE 512

 
 // =================================================================
 // 2. Enum 및 구조체 정의
 // =================================================================

 // 디바이스 연결 모드
 enum connect_mode {
    MODE_READ_ONLY,
    MODE_READ_WRITE,
};

// 디바이스 통신 상태
enum comm_state {
    COMM_STATE_UNINITIALIZED, // 초기화 이전
    COMM_STATE_IDLE, // 유휴 상태 (데이터 교환 가능)
    COMM_STATE_RECEIVING, // 데이터 수신 중
    COMM_STATE_SENDING, // 데이터 송신 중
    COMM_STATE_DONE, // 작업 완료 (read/write 대기 해제용)
    COMM_STATE_ERROR // 오류 발생
};

 struct gpio_comm_dev {
     char name[MAX_NAME_LEN];
     struct cdev cdev;
     struct device *device;

     enum connect_mode mode;

     // GPIO 핀 디스크립터
     struct gpio_desc *ctrl_pin;
     struct gpio_desc *data_pins[NUM_DATA_PINS];
     int my_pin_idx; // READ_WRITE 모드일 때 할당된 데이터 핀 인덱스 (0-3), -1은 R/O
 
     // IRQ 번호
     int irqs[NUM_DATA_PINS+1];  // 0번은 ctrl핀 irq
 
     // 상태 및 동기화
     enum comm_state state;
     spinlock_t lock;
     wait_queue_head_t wq;
 
     // 수신 관련
     u8 rx_buffer[RX_BUFFER_SIZE];
     atomic_t rx_len;        // 수신해야 할 총 길이
     atomic_t rx_bytes_done; // 수신 완료된 바이트 수
     atomic_t rx_bits_done;  // 현재 바이트 내 수신된 비트 수
     atomic_t data_ready;    // 수신 완료 플래그
     ktime_t last_irq_time;  // 마지막 IRQ 시간 기록 (us 단위)

    struct timer_list timeout_timer;
 };
 

 
 // =================================================================
 // 3. 전역 변수
 // =================================================================

 static struct gpio_comm_dev *g_dev_table[MAX_DEVICES];

 static int g_major_num;
 static struct class *g_dev_class;

 static int used_gpio_bcm[MAX_GPIO_BCM] = {0};


 
 // =================================================================
 // 4. 프로토타입 선언
 // =================================================================

 static irqreturn_t ctrl_pin_irq_handler(int irq, void *dev_id);
 static irqreturn_t data_pin_irq_handler(int irq, void *dev_id);
 static void set_all_pins_direction(struct gpio_comm_dev *dev, enum gpiod_flags dir);
 static void release_all_resources(struct gpio_comm_dev *dev);
 

 
 // =================================================================
 // 5. 하위 레벨 통신 함수
 // =================================================================

 static void write_4bits(u8 data, struct gpio_comm_dev *dev) {
     gpiod_set_value(dev->data_pins[0], data & 0x01);
     gpiod_set_value(dev->data_pins[1], (data >> 1) & 0x01);
     gpiod_set_value(dev->data_pins[2], (data >> 2) & 0x01);
     gpiod_set_value(dev->data_pins[3], (data >> 3) & 0x01);
 }
 
 static u8 read_4bits(struct gpio_comm_dev *dev) {
     u8 data = 0;
     data |= gpiod_get_value(dev->data_pins[0]) & 0x01;
     data |= (gpiod_get_value(dev->data_pins[1]) & 0x01) << 1;
     data |= (gpiod_get_value(dev->data_pins[2]) & 0x01) << 2;
     data |= (gpiod_get_value(dev->data_pins[3]) & 0x01) << 3;
     return data;
 }
 
 static void toggle_ctrl_clock(struct gpio_comm_dev *dev) {
     gpiod_set_value(dev->ctrl_pin, 0);
     udelay(CLOCK_DELAY_US);
     gpiod_set_value(dev->ctrl_pin, 1);
     udelay(CLOCK_DELAY_US);
 }


 
 // =================================================================
 // 6. Interrupt 및 타이머 핸들러
 // =================================================================
  
 /**
  * @brief 타이머 만료 시 호출 (수신 타임아웃 처리)
  */
 static void comm_timeout_callback(struct timer_list *t) {
     struct gpio_comm_dev *dev = from_timer(dev, t, timeout_timer);

     pr_warn("[%s] %s: Read timeout.\n", DRIVER_NAME, dev->name);
     
     atomic_set(&dev->data_ready, -ETIMEDOUT);
     wake_up_interruptible(&dev->wq);
 }
 
 /**
  * @brief 제어 핀 IRQ (데이터 수신 및 EOT 감지)
  */
 static irqreturn_t ctrl_pin_irq_handler(int irq, void *dev_id) {
     struct gpio_comm_dev *dev = (struct gpio_comm_dev *)dev_id;
     ktime_t now;
     s64 delta_us;
     u8 received_bits;
 
     now = ktime_get();
     delta_us = ktime_us_delta(now, dev->last_irq_time);
     dev->last_irq_time = now;
 
     // 너무 빨리 들어온 IRQ는 노이즈로 간주하고 무시
     if (delta_us < (CLOCK_DELAY_US / 2)) return IRQ_HANDLED;
 
     // 타임아웃 감지
     if (delta_us > TIMEOUT_MIN_US) {
         pr_warn("[%s] RX Timeout detected.\n", DRIVER_NAME);
         atomic_set(&dev->data_ready, -ETIMEDOUT);
         wake_up_interruptible(&dev->wq);
         return IRQ_HANDLED;
     }
     
     received_bits = read_4bits(dev);
 
     // EOT (느린 클럭 + 모든 데이터 핀 High) 감지
     if (delta_us > SLOW_CLK_MIN_US) {
         if (received_bits == 0x0F) {
             pr_info("[%s] EOT detected.\n", DRIVER_NAME);
             atomic_set(&dev->data_ready, 1); // 성공적으로 수신 완료

             wake_up_interruptible(&dev->wq);
         } else {
             pr_warn("[%s] Invalid EOT signal.\n", DRIVER_NAME);
             atomic_set(&dev->data_ready, -EIO);
             wake_up_interruptible(&dev->wq);
         }
         return IRQ_HANDLED;
     }
 
     // 일반 데이터 수신
     if (atomic_read(&dev->rx_bits_done) == 0) { // 상위 4비트
         dev->rx_buffer[atomic_read(&dev->rx_bytes_done)] = received_bits << 4;
         atomic_inc(&dev->rx_bits_done);
     } else { // 하위 4비트 및 바이트 완성
         dev->rx_buffer[atomic_read(&dev->rx_bytes_done)] |= received_bits;
         atomic_set(&dev->rx_bits_done, 0);
         atomic_inc(&dev->rx_bytes_done);
     }
 
     return IRQ_HANDLED;
 }
 
 /**
  * @brief 데이터 핀 IRQ (다른 노드의 전송 요청 감지)
  */
 static irqreturn_t data_pin_irq_handler(int irq, void *dev_id) {
     struct gpio_comm_dev *dev = (struct gpio_comm_dev *)dev_id;
 
     // 유휴 상태가 아니거나, 내 핀에서 발생한 IRQ는 무시
     if (dev->state != COMM_STATE_IDLE || irq == dev->irqs[dev->my_pin_idx + 1]) {
         return IRQ_NONE;
     }
 
     pr_info("[%s] Bus request from another node detected. Switching to RX mode.\n", DRIVER_NAME);
 
     // 1. 버스를 사용 중(BUSY)으로 마킹
     dev->state = COMM_STATE_RECEIVING;
     
     // 2. 내 데이터 핀을 입력으로 전환하여 버스 양보
     gpiod_direction_input(dev->data_pins[dev->my_pin_idx]);
     
     // 3. 제어 핀 IRQ를 활성화하여 데이터 수신 준비
     dev->last_irq_time = ktime_get();
     atomic_set(&dev->rx_bytes_done, 0);
     atomic_set(&dev->rx_bits_done, 0);
     enable_irq(dev->irqs[0]);

     // disable data_pin_irqs
     for (int i = 0; i < NUM_DATA_PINS; i++) disable_irq(dev->irqs[i + 1]);
 
     return IRQ_HANDLED;
 }
 


 // =================================================================
 // 7. file_operations 구현
 // =================================================================
 
 static int gpio_comm_open(struct inode *inode, struct file *filp) {
    struct gpio_comm_dev *dev = container_of(inode->i_cdev, struct gpio_comm_dev, cdev);
    filp->private_data = dev;
    pr_info("[%s] Device '%s' opened.\n", DRIVER_NAME, dev->name);
    return 0;
}

static int gpio_comm_release(struct inode *inode, struct file *filp) {
    pr_info("[%s] Device released.\n", DRIVER_NAME);
    return 0;
}
 
 static ssize_t gpio_comm_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos) {
     struct gpio_comm_dev *dev = filp->private_data;
     u8 *tx_buf;
     int i;
     u16 crc;
     size_t total_len = count + 4; // 2(길이 헤더) + count(데이터) + 2(CRC) 바이트
 
     // 1. 버스 상태 확인
     if (dev->state != COMM_STATE_IDLE) return -EBUSY;
 
     // 2. 전송 버퍼 할당 및 데이터 준비
     tx_buf = kmalloc(total_len, GFP_KERNEL);
     if (!tx_buf) { 
        dev->state = COMM_STATE_IDLE; 
        return -ENOMEM; 
    }
     if (copy_from_user(tx_buf + 2, buf, count)) { 
        kfree(tx_buf); 
        dev->state = COMM_STATE_IDLE;
         return -EFAULT; 
        }
 
     tx_buf[0] = (u8)(total_len & 0xFF);
     tx_buf[1] = (u8)((total_len >> 8) & 0xFF);
     crc = crc16(0, tx_buf + 2, count);
     tx_buf[count + 2] = (u8)(crc & 0xFF);
     tx_buf[count + 3] = (u8)((crc >> 8) & 0xFF);
     
     // 3. 데이터 핀 IRQ 비활성화 (내가 쓸거니까)
     for (i = 0; i < NUM_DATA_PINS; i++) disable_irq(dev->irqs[i + 1]);
 
     // 4. 버스 사용 요청: 내 데이터 핀 클럭킹
     gpiod_set_value(dev->data_pins[dev->my_pin_idx], 0);
     udelay(50);
     gpiod_set_value(dev->data_pins[dev->my_pin_idx], 1);
     msleep(10); // 다른 노드들이 반응할 시간
 
     // 5. 제어핀 예비 클럭킹 및 충돌 확인
     set_all_pins_direction(dev, GPIOD_OUT_LOW);
     for (i = 0; i < 3; i++) toggle_ctrl_clock(dev);
     if (read_4bits(dev) != 0) {
         pr_err("[%s] Bus collision detected! Aborting TX.\n", DRIVER_NAME);
         goto tx_abort;
     }
 
     // 6. 실제 데이터 전송
     for (i = 0; i < total_len; i++) {
         write_4bits(tx_buf[i] >> 4, dev);
         toggle_ctrl_clock(dev);
         write_4bits(tx_buf[i] & 0x0F, dev);
         toggle_ctrl_clock(dev);
     }
     
     // 7. EOT 신호 전송 (느린 클럭 + 데이터핀 모두 High)
     write_4bits(0x0F, dev);
     gpiod_set_value(dev->ctrl_pin, 0);
     udelay(CLOCK_DELAY_US * 3);
     gpiod_set_value(dev->ctrl_pin, 1);
 
 tx_abort:
     kfree(tx_buf);
     // 8. 버스 해제 및 상태 복원
     set_all_pins_direction(dev, GPIOD_IN);
     gpiod_direction_output(dev->data_pins[dev->my_pin_idx], 1);
     for (i = 0; i < NUM_DATA_PINS; i++) enable_irq(dev->irqs[i + 1]);
     dev->state = COMM_STATE_IDLE;
 
     return count;
 }
 
 static ssize_t gpio_comm_read(struct file *filp, char __user *buf, size_t len, loff_t *off) {
     struct gpio_comm_dev *dev = filp->private_data;
     int ret, bytes_to_copy;
     u16 rx_len, calc_crc, rx_crc;
 
     // 1. 데이터가 준비될 때까지 대기
     ret = wait_event_interruptible(dev->wq, atomic_read(&dev->data_ready) != 0);
     if (ret) return -ERESTARTSYS;
     
     ret = atomic_read(&dev->data_ready);
     atomic_set(&dev->data_ready, 0);
     if (ret < 0) return ret;
 
     // 2. 수신된 데이터 검증
     rx_len = (dev->rx_buffer[1] << 8) | dev->rx_buffer[0];
     if (rx_len != atomic_read(&dev->rx_bytes_done)) return -EIO; // 길이 불일치
     
     bytes_to_copy = rx_len - 4; // 2(길이) + 2(CRC) 제외
     calc_crc = crc16(0, dev->rx_buffer + 2, bytes_to_copy);
     rx_crc = (dev->rx_buffer[rx_len - 1] << 8) | dev->rx_buffer[rx_len - 2];
     
     if(calc_crc != rx_crc) { pr_err("[%s] CRC mismatch!\n", DRIVER_NAME); return -EBADMSG; }
 
     // 3. 유저 공간으로 복사
     if (len < bytes_to_copy) bytes_to_copy = len;
     if (copy_to_user(buf, dev->rx_buffer + 2, bytes_to_copy)) return -EFAULT;
     
     // 4. 버스 상태 복원 (수신 후)
     gpiod_direction_output(dev->data_pins[dev->my_pin_idx], 1);
     disable_irq(dev->irqs[0]);
     dev->state = COMM_STATE_IDLE;
 
     return bytes_to_copy;
 }
 
 static const struct file_operations gpio_comm_fops = {
     .owner = THIS_MODULE,
     .open = gpio_comm_open,
     .release = gpio_comm_release,
     .read = gpio_comm_read,
     .write = gpio_comm_write,
 };
 


 // =================================================================
 // 8. Sysfs
 // =================================================================
 
 static void set_all_pins_direction(struct gpio_comm_dev *dev, enum gpiod_flags dir) {
     int i;
     for (i = 0; i < NUM_DATA_PINS; i++) {
         gpiod_direction_output(dev->data_pins[i], dir);
     }
 }
 
 static void release_all_resources(struct gpio_comm_dev *dev) {
     int i;
     if (!dev) return;
     for (i = 0; i <= NUM_DATA_PINS; i++) {
         if (dev->irqs[i] > 0) free_irq(dev->irqs[i], dev);
     }
     if(dev->ctrl_pin) gpiod_put(dev->ctrl_pin);
     for (i = 0; i < NUM_DATA_PINS; i++) {
         if(dev->data_pins[i]) {
            int bcm = desc_to_gpio(dev->data_pins[i]);
            gpiod_put(dev->data_pins[i]);
            used_gpio_bcm[bcm] = 0;
         }
     }

     del_timer_sync(&dev->timeout_timer);
     device_destroy(g_dev_class, dev->device->devt);
     cdev_del(&dev->cdev);

     kfree(dev);
 }
 
 static ssize_t export_store(const struct class *class, const struct class_attribute *attr, const char *buf, size_t count) {
    char name[MAX_NAME_LEN], mode_str[4];
    int pins[NUM_DATA_PINS + 1];
    struct gpio_comm_dev *new_dev;
    int i, dev_idx = -1, pin_idx = -1, ret;

    // 1. 입력 파싱: "name,mode,ctrl,d0,d1,d2,d3"
    // 입력 예시: echo "my_dev1,rw,17,27,22,23,24" > /sys/class/gpio_comm/export
    ret = sscanf(buf, "%19[^,],%3[^,],%d,%d,%d,%d,%d", 
                 name, mode_str, &pins[0], &pins[1], &pins[2], &pins[3], &pins[4]);
    if (ret != 7) return -EINVAL;

    // 2. 유효성 검사 및 리소스 할당
    for (i = 0; i < MAX_DEVICES; i++) {
        if (!g_dev_table[i]) {
            dev_idx = i;
            break;
        }
    }
    if (dev_idx == -1) { return -ENOMEM; }

   // 이미 사용 중인 핀인지 확인
   for (int i = 0; i <= NUM_DATA_PINS; i++) {
       if (used_gpio_bcm[pins[i]]) {
           pr_err("[%s] GPIO pin %d already in use.\n", DRIVER_NAME, pins[i]);

           return -EEXIST;
       }
   }

   // 이미 초기화된 디바이스인지 확인
   for (int i = 0; i < MAX_DEVICES; i++) {
       if (g_dev_table[i] && strcmp(g_dev_table[i]->name, name) == 0) {
       pr_warn("[%s] Device %s already initialized.\n", DRIVER_NAME, name);

       return -EEXIST;
       }
   }

    // 3. 새 디바이스 구조체 할당 및 초기화
    new_dev = kzalloc(sizeof(struct gpio_comm_dev), GFP_KERNEL);
    if (!new_dev) { return -ENOMEM; }
    strcpy(new_dev->name, name);
    
    // 모드 설정
    if (strcmp(mode_str, "rw") == 0) new_dev->mode = MODE_READ_WRITE;
    else if (strcmp(mode_str, "r") == 0) new_dev->mode = MODE_READ_ONLY;
    else { ret = -EINVAL; goto err_free_dev; }

    // gpio 할당
    new_dev->ctrl_pin = gpio_to_desc(GPIOCHIP_BASE + pins[0]);
    for(i=0; i < NUM_DATA_PINS; i++) new_dev->data_pins[i] = gpio_to_desc(GPIOCHIP_BASE + pins[i+1]);
    set_all_pins_direction(new_dev, GPIOD_IN);

    // RW 모드인 경우, 비어있는 데이터 핀 할당
    if (new_dev->mode == MODE_READ_WRITE) {
        for(i = 0; i < NUM_DATA_PINS; i++) {
            if (gpiod_get_value(new_dev->data_pins[i]) == 1) continue;
            
            pr_info("[%s] %s: Using data pin %d for transmission.\n", DRIVER_NAME, name, i);
            new_dev->my_pin_idx = i;

            // TODO: 조금더 안전하게 일정기간 관찰하기?
            gpiod_direction_output(new_dev->data_pins[i], 1);
            break;
        }
        if (i == NUM_DATA_PINS) { ret = -EBUSY; goto err_free_dev; }

    } else {
        new_dev->my_pin_idx = -1;
    }

    // 4. character device 및 device 파일 생성
    cdev_init(&new_dev->cdev, &gpio_comm_fops);

    new_dev->cdev.owner = THIS_MODULE;
    new_dev->device = device_create(g_dev_class, NULL, MKDEV(g_major_num, dev_idx), NULL, new_dev->name);
    if (IS_ERR(new_dev->device)) {
        ret = PTR_ERR(new_dev->device); 
        goto err_free_dev; 
    }
    if (cdev_add(&new_dev->cdev, new_dev->device->devt, 1)) {
        goto err_destroy_device;
    }

    // 5. 타이머 및 대기 큐 초기화
    timer_setup(&new_dev->timeout_timer, comm_timeout_callback, 0);
    init_waitqueue_head(&new_dev->wq);

    // 6. IRQ 설정 (RW 모드만 데이터 핀 IRQ 필요)
    new_dev->irqs[0] = gpiod_to_irq(new_dev->ctrl_pin);
    ret = request_irq(new_dev->irqs[0], ctrl_pin_irq_handler, IRQF_TRIGGER_RISING, "ctrl_pin", new_dev);
    disable_irq(new_dev->irqs[0]);

    spin_lock_init(&new_dev->lock);
    init_waitqueue_head(&new_dev->wq);
    new_dev->state = COMM_STATE_IDLE;
    atomic_set(&new_dev->data_ready, 0);

    g_dev_table[dev_idx] = new_dev;
    pr_info("[%s] Device '%s' created (mode: %s, pin_idx: %d).\n", DRIVER_NAME, name, mode_str, pin_idx);
    return count;

err_destroy_device:
    device_destroy(g_dev_class, new_dev->device->devt);
err_free_dev:
    kfree(new_dev);
    return ret;
}
 

 static ssize_t unexport_store(const struct class *class, const struct class_attribute *attr, const char *buf, size_t count) {
    char name[MAX_NAME_LEN];
    int i;
    struct gpio_comm_dev *dev = NULL;
    
    sscanf(buf, "%19s", name);
    
    for (i = 0; i < MAX_DEVICES; i++) {
        if (g_dev_table[i] && strcmp(g_dev_table[i]->name, name) == 0) {
            dev = g_dev_table[i];
            g_dev_table[i] = NULL;
            break;
        }
    }

    if (dev) {
        release_all_resources(dev);
        pr_info("[%s] Device '%s' removed.\n", DRIVER_NAME, name);
    } else {
        return -ENOENT;
    }
    return count;
}

static CLASS_ATTR_WO(export);
static CLASS_ATTR_WO(unexport);
 


 // =================================================================
 // 9. 모듈 초기화/종료 및 정보
 // =================================================================
 
 static int __init gpio_comm_init(void) {
    int ret;

     ret = alloc_chrdev_region(&g_major_num, 0, 1, DRIVER_NAME);
     if (ret) return ret;

     g_major_num = MAJOR(g_major_num);
     g_dev_class = class_create(CLASS_NAME);

     class_create_file(g_dev_class, &class_attr_export);
     class_create_file(g_dev_class, &class_attr_unexport);
     
     return 0;
 }
 
 static void __exit gpio_comm_exit(void) {
     for (int i = 0; i < MAX_DEVICES; i++) {
         if (g_dev_table[i]) {
             release_all_resources(g_dev_table[i]);

             device_destroy(g_dev_class, MKDEV(g_major_num, i));
             kfree(g_dev_table[i]);
             g_dev_table[i] = NULL;
         }
     }
     
     class_remove_file(g_dev_class, &class_attr_export);
     class_remove_file(g_dev_class, &class_attr_unexport);  
     class_destroy(g_dev_class);

     unregister_chrdev_region(MKDEV(g_major_num, 0), 1);
 }
 
 module_init(gpio_comm_init);
 module_exit(gpio_comm_exit);
 
 MODULE_LICENSE("GPL");
 MODULE_AUTHOR("HJH & Gemini");
 MODULE_DESCRIPTION("P2P GPIO Comm Driver v2.1");
 