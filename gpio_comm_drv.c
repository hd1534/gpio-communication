/**
 * @file gpio_comm_drv.c
 * @brief 5-wire 멀티-디바이스 P2P GPIO 통신 드라이버
 * @author HJH (Original Concept), Gemini (Refactored & Implemented)
 * @version 2.0
 *
 * @note
 * - 이 드라이버는 최대 4개의 장치가 1개의 제어핀과 4개의 데이터핀을 공유하는
 * P2P 통신 프로토콜을 구현합니다.
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
 #include <linux/timer.h>
 #include <linux/wait.h>
 #include <linux/spinlock.h>
 #include <linux/crc16.h>
 #include <linux/ctype.h>
 


 // =================================================================
 // 1. 상수 및 매크로 정의
 // =================================================================
 #define CLASS_NAME "gpio_comm"
 #define DRIVER_NAME "gpio_comm_drv"
 
 #define MAX_DEVICES 10
 #define NUM_DATA_PINS 4
 #define MAX_NAME_LEN 20
 
 #define RX_BUFFER_SIZE 256

 #define TIMEOUT_MS 500
 #define BUS_CLEAR_TIMEOUT_MS 100
 


 // =================================================================
 // 2. Enum 및 구조체 정의
 // =================================================================
 
 // 디바이스 연결 모드
 enum connect_mode {
     MODE_READ_ONLY,
     MODE_READ_WRITE,
 };
 
 // 공유 버스의 상태
 enum bus_state {
     BUS_IDLE,
     BUS_REQUESTED, // 누군가 전송을 요청한 상태
     BUS_ACTIVE,    // 실제 데이터 전송이 일어나는 상태
 };
 
 // 전역 버스 상태를 관리하는 구조체
 struct gpio_comm_bus {
     struct gpio_desc *ctrl_pin;
     struct gpio_desc *data_pins[NUM_DATA_PINS];
     int irqs[NUM_DATA_PINS + 1]; // 0: ctrl, 1-4: data
 
     atomic_t state; // 버스 상태 (BUS_IDLE 등)
     spinlock_t lock; // 구조체 보호를 위한 스핀락
 
     // 어떤 디바이스가 어떤 데이터 핀을 점유했는지 추적
     struct gpio_comm_dev *participants[NUM_DATA_PINS];
     // 버스 사용을 요청한 디바이스
     struct gpio_comm_dev *requester;
 };
 
 // 개별 디바이스를 나타내는 구조체
 struct gpio_comm_dev {
     char name[MAX_NAME_LEN];
     struct cdev cdev;
     struct device *device;
     struct gpio_comm_bus *bus; // 공유 버스에 대한 포인터
 
     enum connect_mode mode;
     int assigned_pin_idx; // READ_WRITE 모드일 때 할당된 데이터 핀 인덱스 (0-3), -1은 R/O
 
     u8 rx_buffer[RX_BUFFER_SIZE];
     atomic_t rx_byte_count; // 수신된 바이트 수
     atomic_t rx_bit_count;  // 현재 바이트 내 수신된 비트 수
     atomic_t data_ready;    // 수신 완료 플래그
 
     wait_queue_head_t wq; // read 대기를 위한 큐
     struct timer_list timeout_timer;
 };
 


 // =================================================================
 // 3. 전역 변수
 // =================================================================
 
 static struct gpio_comm_bus g_bus;
 static struct gpio_comm_dev *g_dev_table[MAX_DEVICES];

 static int g_major_num;
 static struct class *g_dev_class;
 
static int used_gpio_bcm[MAX_GPIO_BCM] = {0};



 // =================================================================
 // 4. 프로토타입 선언
 // =================================================================
 static irqreturn_t ctrl_pin_irq_handler(int irq, void *dev_id);
 static irqreturn_t data_pin_irq_handler(int irq, void *dev_id);
 


 // =================================================================
 // 5. 하위 레벨 통신 함수
 // =================================================================

 static void set_data_pins_direction(enum gpiod_flags dir) {
     int i;
     for (i = 0; i < NUM_DATA_PINS; i++) {
         if (g_bus.data_pins[i]) {
             gpiod_direction_input(g_bus.data_pins[i]);
         }
     }
 }
 
 // 4-bit 데이터를 병렬로 전송
 static void write_4bits(u8 data) {
     gpiod_set_value(g_bus.data_pins[0], data & 0x01);
     gpiod_set_value(g_bus.data_pins[1], (data >> 1) & 0x01);
     gpiod_set_value(g_bus.data_pins[2], (data >> 2) & 0x01);
     gpiod_set_value(g_bus.data_pins[3], (data >> 3) & 0x01);
 }
 
 // 4-bit 데이터를 병렬로 수신
 static u8 read_4bits(void) {
     u8 data = 0;
     data |= gpiod_get_value(g_bus.data_pins[0]);
     data |= gpiod_get_value(g_bus.data_pins[1]) << 1;
     data |= gpiod_get_value(g_bus.data_pins[2]) << 2;
     data |= gpiod_get_value(g_bus.data_pins[3]) << 3;
     return data;
 }
 
 // 제어 핀 클럭 토글
 static void toggle_ctrl_clock(int delay_us) {
     gpiod_set_value(g_bus.ctrl_pin, 0);
     udelay(delay_us);
     gpiod_set_value(g_bus.ctrl_pin, 1);
     udelay(delay_us);
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
  * @brief 제어 핀 IRQ 핸들러 (데이터 수신)
  */
 static irqreturn_t ctrl_pin_irq_handler(int irq, void *dev_id) {
     int i;
     u8 received_bits;
     u8 current_byte;
     int byte_count;
 
     // 버스가 활성 상태가 아니면 무시
     if (atomic_read(&g_bus.state) != BUS_ACTIVE) return IRQ_NONE;
     
     // 데이터 핀에서 4-bit 읽기
     received_bits = read_4bits();
 
     // 모든 참여자에게 데이터 배포
     for (i = 0; i < MAX_DEVICES; i++) {
         struct gpio_comm_dev *dev = g_dev_table[i];
         if (!dev) continue;
         
         // 타이머 리셋
         del_timer(&dev->timeout_timer);
         
         byte_count = atomic_read(&dev->rx_byte_count);
         current_byte = dev->rx_buffer[byte_count];
 
         // 4-bit씩 2번에 걸쳐 1바이트 완성
         if (atomic_read(&dev->rx_bit_count) == 0) { // 상위 4비트
             current_byte = received_bits << 4;
             atomic_inc(&dev->rx_bit_count);
         } else { // 하위 4비트
             current_byte |= received_bits;
             atomic_set(&dev->rx_bit_count, 0);
             byte_count = atomic_inc_return(&dev->rx_byte_count);
         }
         dev->rx_buffer[byte_count] = current_byte;
 
         // 패킷 완료 조건 (예: 128바이트)
         if (byte_count >= 128) {
             atomic_set(&dev->data_ready, 1);
             wake_up_interruptible(&dev->wq);
         } else {
             mod_timer(&dev->timeout_timer, jiffies + msecs_to_jiffies(TIMEOUT_MS));
         }
     }
     return IRQ_HANDLED;
 }
 
 /**
  * @brief 데이터 핀 IRQ 핸들러 (전송 요청 감지)
  */
 static irqreturn_t data_pin_irq_handler(int irq, void *dev_id) {
     struct gpio_comm_dev *self = (struct gpio_comm_dev *)dev_id;
     int i;
 
     // 자신의 IRQ는 무시
     if (self->assigned_pin_idx >= 0 && irq == g_bus.irqs[self->assigned_pin_idx + 1]) {
         return IRQ_NONE;
     }
     
     // 전송 요청이 아니면 무시
     if (atomic_read(&g_bus.state) != BUS_IDLE) return IRQ_NONE;
 
     pr_info("[%s] %s: Bus request detected from another device.\n", DRIVER_NAME, self->name);
 
     // 자신의 데이터 핀을 입력으로 전환하여 버스를 비워줌
     if (self->mode == MODE_READ_WRITE && self->assigned_pin_idx >= 0) {
         gpiod_direction_input(g_bus.data_pins[self->assigned_pin_idx]);
     }
     
     // 버스 상태를 REQUESTED로 변경 (최초 한 번만)
     atomic_cmpxchg(&g_bus.state, BUS_IDLE, BUS_REQUESTED);
 
     // 누가 요청했는지 찾기
     for(i = 0; i < NUM_DATA_PINS; i++) {
         if(irq == g_bus.irqs[i+1]) {
             spin_lock(&g_bus.lock);
             g_bus.requester = g_bus.participants[i];
             spin_unlock(&g_bus.lock);
             break;
         }
     }
 
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
     u8 tx_buf[128];
     int i, ret;
     unsigned long start_time;
 
     if (dev->mode != MODE_READ_WRITE) return -EPERM;
     if (count > 128) return -EINVAL;
 
     // 1. 버스 소유권 획득 시도
     spin_lock(&g_bus.lock);
     if (atomic_read(&g_bus.state) != BUS_IDLE) {
         spin_unlock(&g_bus.lock);
         return -EBUSY;
     }
     atomic_set(&g_bus.state, BUS_REQUESTED);
     g_bus.requester = dev;
     spin_unlock(&g_bus.lock);
 
     // 2. 자신의 핀을 클럭킹하여 버스 사용 요청
     pr_info("[%s] %s: Requesting bus...\n", DRIVER_NAME, dev->name);
     gpiod_set_value(g_bus.data_pins[dev->assigned_pin_idx], 0);
     udelay(50);
     gpiod_set_value(g_bus.data_pins[dev->assigned_pin_idx], 1);
 
     // 3. 다른 모든 데이터 핀이 Low(입력 상태)가 될 때까지 대기
     start_time = jiffies;
     while (true) {
         bool all_low = true;
         for (i = 0; i < NUM_DATA_PINS; i++) {
             if (i == dev->assigned_pin_idx) continue;
             if (gpiod_get_value(g_bus.data_pins[i]) == 1) {
                 all_low = false;
                 break;
             }
         }
         if (all_low) break;
 
         if (time_is_after_jiffies(start_time + msecs_to_jiffies(BUS_CLEAR_TIMEOUT_MS))) {
             pr_err("[%s] %s: Bus clear timeout!\n", DRIVER_NAME, dev->name);
             atomic_set(&g_bus.state, BUS_IDLE);
             return -ETIMEDOUT;
         }
         msleep(1);
     }
     
     // 4. 버스를 점유하고 데이터 전송 준비
     pr_info("[%s] %s: Bus acquired. Starting transmission.\n", DRIVER_NAME, dev->name);
     atomic_set(&g_bus.state, BUS_ACTIVE);
     set_data_pins_direction(GPIOD_OUT_LOW); // 모든 데이터 핀을 출력으로 전환
 
     // 5. 제어 핀 3회 예비 클럭킹
     for (i = 0; i < 3; i++) toggle_ctrl_clock(50);
     
     if (copy_from_user(tx_buf, buf, count)) return -EFAULT;
     // (CRC 계산 및 추가 로직 필요)
 
     // 6. 데이터 전송 (4-bit씩 2번에 1바이트)
     for (i = 0; i < count; i++) {
         write_4bits(tx_buf[i] >> 4); // 상위 4비트
         toggle_ctrl_clock(100);
         write_4bits(tx_buf[i] & 0x0F); // 하위 4비트
         toggle_ctrl_clock(100);
     }
 
     // 7. 전송 완료 및 버스 해제
     set_data_pins_direction(GPIOD_IN); // 모든 데이터 핀을 다시 입력으로
     // 자신의 핀은 다시 출력 High로
     gpiod_direction_output(g_bus.data_pins[dev->assigned_pin_idx], 1);
     atomic_set(&g_bus.state, BUS_IDLE);
 
     pr_info("[%s] %s: Transmission complete. Bus released.\n", DRIVER_NAME, dev->name);
 
     return count;
 }
 
 static ssize_t gpio_comm_read(struct file *filp, char __user *buf, size_t len, loff_t *off) {
     struct gpio_comm_dev *dev = filp->private_data;
     int ret, bytes_read;
 
     // 1. 수신 준비
     atomic_set(&dev->data_ready, 0);
     atomic_set(&dev->rx_byte_count, 0);
     atomic_set(&dev->rx_bit_count, 0);
     mod_timer(&dev->timeout_timer, jiffies + msecs_to_jiffies(5000)); // 5초 대기
 
     // 2. 데이터가 준비될 때까지 대기
     ret = wait_event_interruptible(dev->wq, atomic_read(&dev->data_ready) != 0);
     del_timer_sync(&dev->timeout_timer); // 깨어나면 타이머 제거
 
     if (ret < 0) return -ERESTARTSYS;
     if (atomic_read(&dev->data_ready) < 0) return atomic_read(&dev->data_ready);
 
     // 3. 데이터 복사
     bytes_read = atomic_read(&dev->rx_byte_count);
     if (len < bytes_read) bytes_read = len;
     if (copy_to_user(buf, dev->rx_buffer, bytes_read)) return -EFAULT;
 
     return bytes_read;
 }
 
 static const struct file_operations gpio_comm_fops = {
     .owner = THIS_MODULE,
     .open = gpio_comm_open,
     .release = gpio_comm_release,
     .read = gpio_comm_read,
     .write = gpio_comm_write,
 };
 
 // =================================================================
 // 8. Sysfs 및 모듈 초기화/종료
 // =================================================================
 
 static void cleanup_device(struct gpio_comm_dev *dev) {
     if (!dev) return;
 
     if (dev->assigned_pin_idx != -1) {
         g_bus.participants[dev->assigned_pin_idx] = NULL;
     }
     
     del_timer_sync(&dev->timeout_timer);
     device_destroy(g_dev_class, dev->device->devt);
     cdev_del(&dev->cdev);
     kfree(dev);
 }
 
 static ssize_t export_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count) {
     char name[MAX_NAME_LEN], mode_str[10];
     int pins[NUM_DATA_PINS + 1];
     struct gpio_comm_dev *new_dev = NULL;
     int i, dev_idx = -1, pin_idx = -1, ret;
 
     // 1. 입력 파싱: "name,mode,ctrl,d0,d1,d2,d3"
     // 입력 예시: echo "my_dev1,rw,17,27,22,23,24" > /sys/class/gpio_comm/export
     ret = sscanf(buf, "%19[^,],%9[^,],%d,%d,%d,%d,%d", 
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
    for (int i = 0; i <= NUMBER_OF_DATA_PINS; i++) {
        if (used_gpio_bcm[bcm[i]]) {
            pr_err("[%s] GPIO pin %d already in use.\n", DRIVER_NAME, bcm[i]);

            return -EEXIST;
        }
    }

    // 이미 초기화된 디바이스인지 확인

    for (int i = 0; i < MAX_DEV; i++) {
        if (gpio_comm_table[i] && strcmp(gpio_comm_table[i]->name, name) == 0) {
        pr_warn("[%s] Device %s already initialized.\n", DRIVER_NAME, name);

        return -EEXIST;
        }
    }

 
     // 3. 새 디바이스 구조체 할당 및 초기화
     new_dev = kzalloc(sizeof(struct gpio_comm_dev), GFP_KERNEL);
     if (!new_dev) { return -ENOMEM; }
     strcpy(new_dev->name, name);
     new_dev->bus = &g_bus;
     
     // 모드 설정
     for (i = 0; mode_str[i]; i++) mode_str[i] = tolower(mode_str[i]);
     if (strcmp(mode_str, "rw") == 0) new_dev->mode = MODE_READ_WRITE;
     else if (strcmp(mode_str, "r") == 0) new_dev->mode = MODE_READ_ONLY;
     else { ret = -EINVAL; goto err_free_dev; }

     // gpio 할당
     new_dev->ctrl_pin = gpiod_get(NULL, NULL, pins[0], GPIOD_ASIS);
     for(i=0; i < NUM_DATA_PINS; i++) new_dev->data_pins[i] = gpiod_get(NULL, NULL, pins[i+1], GPIOD_ASIS);
     set_all_pins_direction(new_dev, GPIOD_IN);
 
     // RW 모드인 경우, 비어있는 데이터 핀 할당
     if (new_dev->mode == MODE_READ_WRITE) {
         for(i = 0; i < NUM_DATA_PINS; i++) {
             if (gpiod_get_value(new_dev->data_pins[i]) == 1) continue;
             
             pr_info("[%s] %s: Using data pin %d for transmission.\n", DRIVER_NAME, name, i);
             new_dev->assigned_pin_idx = i;

             // TODO: 조금더 안전하게 일정기간 관찰하기?
             gpiod_direction_output(new_dev->data_pins[i], 1);
             break;
         }
         if (i == NUM_DATA_PINS) { ret = -EBUSY; goto err_free_dev; }

     } else {
         new_dev->assigned_pin_idx = -1;
     }

     // 4. cdev 및 device 파일 생성
     cdev_init(&new_dev->cdev, &gpio_comm_fops);
     new_dev->device = device_create(g_dev_class, NULL, MKDEV(g_major_num, dev_idx), NULL, new_dev->name);
     if (IS_ERR(new_dev->device)) { ret = PTR_ERR(new_dev->device); goto err_free_dev; }
     ret = cdev_add(&new_dev->cdev, new_dev->device->devt, 1);
     if (ret) goto err_destroy_device;
 
     // 5. 타이머 및 대기 큐 초기화
     timer_setup(&new_dev->timeout_timer, comm_timeout_callback, 0);
     init_waitqueue_head(&new_dev->wq);
 
     // 6. IRQ 설정 (RW 모드만 데이터 핀 IRQ 필요)
     new_dev->irqs[0] = gpiod_to_irq(new_dev->ctrl_pin);
     ret = request_irq(new_dev->irqs[0], ctrl_pin_irq_handler, IRQF_TRIGGER_RISING, "ctrl_pin", new_dev);
     disable_irq(new_dev->irqs[0]);
        
     if (new_dev->mode == MODE_READ_WRITE) {
         gpiod_direction_output(new_dev->data_pins[pin_idx], 1); // 자신의 핀은 출력 High
         for (i = 0; i < NUM_DATA_PINS; i++) {
             if (i == pin_idx) continue;
             ret = request_irq(g_bus.irqs[i+1], data_pin_irq_handler, IRQF_TRIGGER_RISING, name, new_dev);
             // ... (에러 처리) ...
         }
     }


     spin_lock_init(&new_dev->lock);
     init_waitqueue_head(&new_dev->wq);
     atomic_set(&new_dev->state, 0);
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
 
 static ssize_t unexport_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count) {
     char name[MAX_NAME_LEN];
     int i;
     struct gpio_comm_dev *dev = NULL;
     
     sscanf(buf, "%19s", name);
     
     spin_lock(&g_bus.lock);
     for (i = 0; i < MAX_DEVICES; i++) {
         if (g_dev_table[i] && strcmp(g_dev_table[i]->name, name) == 0) {
             dev = g_dev_table[i];
             g_dev_table[i] = NULL;
             break;
         }
     }
     spin_unlock(&g_bus.lock);
 
     if (dev) {
         cleanup_device(dev);
         pr_info("[%s] Device '%s' removed.\n", DRIVER_NAME, name);
     } else {
         return -ENOENT;
     }
     return count;
 }
 
 static CLASS_ATTR_WO(export);
 static CLASS_ATTR_WO(unexport);
 
 static struct attribute *gpio_comm_class_attrs[] = {
     &class_attr_export.attr,
     &class_attr_unexport.attr,
     NULL,
 };
 ATTRIBUTE_GROUPS(gpio_comm_class);
 
 
 /**
  * @brief 드라이버 로드 시 호출
  */
 static int __init gpio_comm_init(void) {
     int ret;
     pr_info("[%s] module loading\n", DRIVER_NAME);
 
     // 1. 전역 버스 구조체 초기화
     memset(&g_bus, 0, sizeof(g_bus));
     spin_lock_init(&g_bus.lock);
     atomic_set(&g_bus.state, BUS_IDLE);
 
     // 2. 캐릭터 디바이스 영역 할당
     ret = alloc_chrdev_region(&g_major_num, 0, MAX_DEVICES, DRIVER_NAME);
     if (ret < 0) return ret;
     g_major_num = MAJOR(g_major_num);
 
     // 3. 클래스 생성 및 sysfs 파일 등록
     g_dev_class = class_create(CLASS_NAME);
     if (IS_ERR(g_dev_class)) {
         unregister_chrdev_region(MKDEV(g_major_num, 0), MAX_DEVICES);
         return PTR_ERR(g_dev_class);
     }
     g_dev_class->class_groups = gpio_comm_class_groups;
 
     return 0;
 }
 
 /**
  * @brief 드라이버 언로드 시 호출
  */
 static void __exit gpio_comm_exit(void) {
     int i;
     for (i = 0; i < MAX_DEVICES; i++) {
         if (g_dev_table[i]) {
             cleanup_device(g_dev_table[i]);
             g_dev_table[i] = NULL;
         }
     }
     
     // 버스 리소스 해제 (GPIO, IRQ 등)
     if(g_bus.ctrl_pin) {
         free_irq(g_bus.irqs[0], NULL);
         gpiod_put(g_bus.ctrl_pin);
     }
     for(i = 0; i < NUM_DATA_PINS; i++) {
         if(g_bus.data_pins[i]) {
              // 데이터 핀 IRQ는 개별 디바이스에서 해제됨
             gpiod_put(g_bus.data_pins[i]);
         }
     }
 
     class_destroy(g_dev_class);
     unregister_chrdev_region(MKDEV(g_major_num, 0), MAX_DEVICES);
     
     pr_info("[%s] module unloaded\n", DRIVER_NAME);
 }
 
 module_init(gpio_comm_init);
 module_exit(gpio_comm_exit);
 
 MODULE_LICENSE("GPL");
 MODULE_AUTHOR("HJH & Gemini");
 MODULE_DESCRIPTION("Multi-device P2P GPIO Communication Driver");
 
 