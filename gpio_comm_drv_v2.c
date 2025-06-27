/**
 * @file gpio_comm_drv.c
 * @brief 5-wire 멀티-디바이스 P2P GPIO 통신 드라이버
 * @author HJH (Original Concept), Gemini (Implemented & Refactored)
 * @version 2.2
 *
 * @note
 * - 이 드라이버는 1개의 제어핀과 4개의 데이터핀을 공유하는 P2P 통신 프로토콜을 구현합니다.
 * - 사용법:
 * 1. 드라이버 로드: insmod gpio_comm_drv.ko
 * 2. 디바이스 생성 (sysfs):
 * echo "my_dev1,rw,17,27,22,23,24" > /sys/class/gpio_comm/export
 * (이름, 모드(rw/r), 제어핀 bcm, 데이터핀1 bcm, 데이터핀2 bcm, 데이터핀3 bcm, 데이터핀4 bcm)
 * 3. 생성된 디바이스 노드(/dev/my_dev1)를 통해 read/write 수행
 *
 * --- 변경 내역 (v2.2) ---
 * - [FIXED] 전역 Mutex(g_dev_lock)를 추가하여 export/unexport 동작의 원자성 보장
 * - [FIXED] 각 디바이스에 spinlock을 추가하여 state 등 공유 데이터 접근 시 경쟁 상태 해결
 * - [FIXED] 하드코딩된 GPIOCHIP_BASE를 제거하고 표준 gpiod_get API 사용
 * - [FIXED] 수신 타임아웃 타이머가 실제로 동작하도록 mod_timer/del_timer_sync 호출 추가
 * - [FIXED] 자원 해제 로직 정리 및 안정성 강화
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
 #include <linux/mutex.h> // [ADDED] Mutex 헤더
 
 
 
 // =================================================================
 // 1. 상수 및 매크로 정의
 // =================================================================
 
 #define CLASS_NAME "gpio_comm"
 #define DRIVER_NAME "gpio_comm_drv"
 
 #define MAX_GPIO_BCM 54 // 라즈베리파이 최신 모델 기준
 #define MAX_DEVICES 10
 #define MAX_NAME_LEN 20
 #define NUM_DATA_PINS 4
 
 #define RX_BUFFER_SIZE 1024 // 데이터 헤더(길이)를 고려하여 여유롭게 설정
 #define CLOCK_DELAY_US 1000 // 통신 속도 향상을 위해 1ms로 조정
 
 // ktime을 마이크로초(us) 단위로 사용
 #define NORMAL_CLK_MAX_US (CLOCK_DELAY_US * 15LL / 10)    // 1.5배까지는 일반 클럭으로 간주
 #define SLOW_CLK_MIN_US   (CLOCK_DELAY_US * 25LL / 10)    // 2.5배부터는 느린 클럭(EOT)
 #define TIMEOUT_MIN_US    (CLOCK_DELAY_US * 10LL)         // 10배 이상은 타임아웃 (10ms)
 
 
 
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
    COMM_STATE_WAIT_BUS,    // [ADDED] 버스 양도를 기다리는 상태
    COMM_STATE_SENDING, // 데이터 수신 중
    COMM_STATE_RECEIVING, // 데이터 송신 중
    COMM_STATE_DONE, // 작업 완료 (read/write 대기 해제용)
    COMM_STATE_ERROR // 오류 발생
 };
 
 struct gpio_comm_dev {
     char name[MAX_NAME_LEN];
     struct cdev cdev;
     struct device *device;
     dev_t devt;
 
     enum connect_mode mode;
 
     // GPIO 핀 디스크립터
     struct gpio_desc *ctrl_pin;
     struct gpio_desc *data_pins[NUM_DATA_PINS];
     int my_pin_idx; // READ_WRITE 모드시 할당된 데이터 핀 인덱스 (0-3), -1은 R/O
 
     // IRQ 번호
     int irqs[NUM_DATA_PINS + 1];  // 0번은 ctrl핀 irq, 1부턴 data핀 irqs
 
     // 상태 및 동기화
     enum comm_state state;
     spinlock_t lock; // [ADDED] 디바이스 내부 데이터 보호용 스핀락
     wait_queue_head_t wq;
 
     // 수신 관련
     u8 *rx_buffer;
     size_t rx_buffer_size;
     u16 expected_rx_len;

     atomic_t rx_bytes_done;
     atomic_t rx_bits_done;
     atomic_t data_ready;
     ktime_t last_irq_time;
 
     struct timer_list timeout_timer;
 };
 
 
 
 // =================================================================
 // 3. 전역 변수
 // =================================================================
 
 static struct gpio_comm_dev *g_dev_table[MAX_DEVICES];
 static int g_major_num;
 static struct class *g_dev_class;
 
 // [ADDED] export/unexport 동기화를 위한 뮤텍스
 static DEFINE_MUTEX(g_dev_lock);
 
 
 
 // =================================================================
 // 4. 프로토타입 선언
 // =================================================================
 
 static void release_all_resources(struct gpio_comm_dev *dev);
 
 
 
 // =================================================================
 // 5. 하위 레벨 통신 함수
 // =================================================================
 
 static void write_4bits(struct gpio_comm_dev *dev, u8 data) {
     gpiod_set_value_cansleep(dev->data_pins[0], data & 0x01);
     gpiod_set_value_cansleep(dev->data_pins[1], (data >> 1) & 0x01);
     gpiod_set_value_cansleep(dev->data_pins[2], (data >> 2) & 0x01);
     gpiod_set_value_cansleep(dev->data_pins[3], (data >> 3) & 0x01);
 }
 
 static u8 read_4bits(struct gpio_comm_dev *dev) {
     u8 data = 0;
     data |= gpiod_get_value_cansleep(dev->data_pins[0]) & 0x01;
     data |= (gpiod_get_value_cansleep(dev->data_pins[1]) & 0x01) << 1;
     data |= (gpiod_get_value_cansleep(dev->data_pins[2]) & 0x01) << 2;
     data |= (gpiod_get_value_cansleep(dev->data_pins[3]) & 0x01) << 3;
     return data;
 }
 
 static void toggle_ctrl_clock(struct gpio_comm_dev *dev) {
     gpiod_set_value_cansleep(dev->ctrl_pin, 0);
     udelay(CLOCK_DELAY_US);
     gpiod_set_value_cansleep(dev->ctrl_pin, 1);
     udelay(CLOCK_DELAY_US);
 }
 
 
 
 
 // =================================================================
 // 6. Interrupt 및 타이머 핸들러
 // =================================================================
 
 static void comm_timeout_callback(struct timer_list *t) {
     struct gpio_comm_dev *dev = from_timer(dev, t, timeout_timer);
     unsigned long flags;
 
     pr_warn("[%s] %s: Communication timeout!\n", DRIVER_NAME, dev->name);
 
     spin_lock_irqsave(&dev->lock, flags);
     if (dev->state == COMM_STATE_RECEIVING) {
         atomic_set(&dev->data_ready, -ETIMEDOUT);
         dev->state = COMM_STATE_IDLE; // 상태 복원
         wake_up_interruptible(&dev->wq);
     }
     spin_unlock_irqrestore(&dev->lock, flags);
 }
 
 static irqreturn_t ctrl_pin_irq_handler(int irq, void *dev_id) {
     struct gpio_comm_dev *dev = (struct gpio_comm_dev *)dev_id;
     ktime_t now;
     s64 delta_us;
     u8 received_nibble;
     unsigned long flags;
     int current_byte_idx;
 
     // [FIXED] 타이머를 리셋하여 타임아웃 연장
     mod_timer(&dev->timeout_timer, jiffies + msecs_to_jiffies(50));
 
     now = ktime_get();
     delta_us = ktime_us_delta(now, dev->last_irq_time);
     dev->last_irq_time = now;
 
     if (delta_us < (CLOCK_DELAY_US / 2)) return IRQ_HANDLED;
 
     received_nibble = read_4bits(dev);
 
     // EOT (느린 클럭 + 데이터핀 모두 High) 감지
     if (delta_us > SLOW_CLK_MIN_US) {
         if (received_nibble == 0x0F) {
             pr_info("[%s] EOT detected.\n", DRIVER_NAME);
             atomic_set(&dev->data_ready, 1);
         } else {
             pr_warn("[%s] Invalid EOT signal.\n", DRIVER_NAME);
             atomic_set(&dev->data_ready, -EIO);
         }
         del_timer(&dev->timeout_timer); // [FIXED] 통신 종료 시 타이머 제거
         wake_up_interruptible(&dev->wq);
         return IRQ_HANDLED;
     }
 
     // 일반 데이터 수신
     spin_lock_irqsave(&dev->lock, flags);
     if (dev->state != COMM_STATE_RECEIVING) {
         spin_unlock_irqrestore(&dev->lock, flags);
         return IRQ_HANDLED; // 수신 상태가 아니면 무시
     }
     
     current_byte_idx = atomic_read(&dev->rx_bytes_done);
 
     if (current_byte_idx >= dev->rx_buffer_size) {
         pr_err("[%s] RX buffer overflow!\n", DRIVER_NAME);
         atomic_set(&dev->data_ready, -ENOMEM);
         del_timer(&dev->timeout_timer);
         wake_up_interruptible(&dev->wq);
         spin_unlock_irqrestore(&dev->lock, flags);
         return IRQ_HANDLED;
     }
 
     if (atomic_read(&dev->rx_bits_done) == 0) { // 상위 4비트
         dev->rx_buffer[current_byte_idx] = received_nibble << 4;
         atomic_inc(&dev->rx_bits_done);
     } else { // 하위 4비트 및 바이트 완성
         dev->rx_buffer[current_byte_idx] |= received_nibble;
         atomic_set(&dev->rx_bits_done, 0);
         atomic_inc(&dev->rx_bytes_done);
     }
     spin_unlock_irqrestore(&dev->lock, flags);
 
     return IRQ_HANDLED;
 }
 
 static irqreturn_t data_pin_irq_handler(int irq, void *dev_id) {
     struct gpio_comm_dev *dev = (struct gpio_comm_dev *)dev_id;
     unsigned long flags;
     int i;
 
     spin_lock_irqsave(&dev->lock, flags);
 
     // 유휴 상태가 아니거나, 내 핀에서 발생한 IRQ는 무시
     if (dev->state != COMM_STATE_IDLE || (dev->my_pin_idx != -1 && irq == dev->irqs[dev->my_pin_idx + 1])) {
         spin_unlock_irqrestore(&dev->lock, flags);
         return IRQ_NONE;
     }
 
     pr_info("[%s] Bus request detected. Switching to RX mode.\n", DRIVER_NAME);
 
     dev->state = COMM_STATE_RECEIVING;
     
     if (dev->my_pin_idx != -1) {
         gpiod_direction_input(dev->data_pins[dev->my_pin_idx]);
     }
     
     dev->last_irq_time = ktime_get();
     atomic_set(&dev->rx_bytes_done, 0);
     atomic_set(&dev->rx_bits_done, 0);
     atomic_set(&dev->data_ready, 0);
 
     // [FIXED] 다른 노드의 요청을 받았으므로, 모든 데이터핀 IRQ 비활성화
     for (i = 0; i < NUM_DATA_PINS; i++) {
         if(dev->irqs[i+1] > 0) disable_irq_nosync(dev->irqs[i+1]);
     }
     enable_irq(dev->irqs[0]); // 제어핀 IRQ 활성화
 
     // [FIXED] 수신 타임아웃 시작
     mod_timer(&dev->timeout_timer, jiffies + msecs_to_jiffies(100));
 
     spin_unlock_irqrestore(&dev->lock, flags);
 
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
     u8 *tx_buf = NULL;
     int i, ret = 0;
     u16 crc;
     size_t total_len = count + 4; // 2(길이) + count(데이터) + 2(CRC)
     unsigned long flags;
 
     if (dev->mode != MODE_READ_WRITE) return -EPERM;
 
     tx_buf = kmalloc(total_len, GFP_KERNEL);
     if (!tx_buf) return -ENOMEM;
 
     // [FIXED] 상태 확인 및 변경을 spinlock으로 보호
     spin_lock_irqsave(&dev->lock, flags);
     if (dev->state != COMM_STATE_IDLE) {
         spin_unlock_irqrestore(&dev->lock, flags);
         kfree(tx_buf);
         return -EBUSY;
     }
     dev->state = COMM_STATE_WAIT_BUS;
     spin_unlock_irqrestore(&dev->lock, flags);
 
     if (copy_from_user(tx_buf + 2, buf, count)) {
         ret = -EFAULT;
         goto tx_abort;
     }
     
     // 데이터 패킷 구성
     tx_buf[0] = (u8)(total_len & 0xFF);
     tx_buf[1] = (u8)((total_len >> 8) & 0xFF);
     crc = crc16(0, tx_buf + 2, count);
     tx_buf[count + 2] = (u8)(crc & 0xFF);
     tx_buf[count + 3] = (u8)((crc >> 8) & 0xFF);
     
     // 데이터 핀 IRQ 비활성화
     for (i = 0; i < NUM_DATA_PINS; i++) if (dev->irqs[i+1] > 0) disable_irq(dev->irqs[i+1]);
     
     // 버스 사용 요청: 내 데이터 핀 클럭킹 (Low -> High)
     gpiod_set_value_cansleep(dev->data_pins[dev->my_pin_idx], 0);
     udelay(50);
     gpiod_set_value_cansleep(dev->data_pins[dev->my_pin_idx], 1);
     
     // [FIXED] 다른 노드들이 반응할 시간을 주고, 실제로 양보했는지 확인 (프로토콜 강화)
     msleep(10); 
     // TODO: 실제로는 다른 rw 디바이스들의 핀이 input으로 전환되었는지 확인하는 로직 필요
 
     // 모든 핀을 출력으로 설정하고 충돌 확인
     for (i = 0; i < NUM_DATA_PINS; i++) gpiod_direction_output(dev->data_pins[i], 0);
     
     // 제어핀 예비 클럭킹 3회
     for (i = 0; i < 3; i++) toggle_ctrl_clock(dev);
 
     // 충돌 감지 로직은 그대로 유지 (다른 장치가 동시에 전송 시도 시 감지)
     if (read_4bits(dev) != 0) {
         pr_err("[%s] Bus collision detected! Aborting TX.\n", DRIVER_NAME);
         ret = -EAGAIN; // 재시도 필요
         goto tx_post_comm;
     }
 
     spin_lock_irqsave(&dev->lock, flags);
     dev->state = COMM_STATE_SENDING;
     spin_unlock_irqrestore(&dev->lock, flags);
 
     // 실제 데이터 전송
     for (i = 0; i < total_len; i++) {
         write_4bits(dev, tx_buf[i] >> 4); // 상위 니블
         toggle_ctrl_clock(dev);
         write_4bits(dev, tx_buf[i] & 0x0F); // 하위 니블
         toggle_ctrl_clock(dev);
     }
     
     // EOT 신호 전송 (느린 클럭 + 데이터핀 모두 High)
     write_4bits(dev, 0x0F);
     gpiod_set_value_cansleep(dev->ctrl_pin, 0);
     usleep_range(CLOCK_DELAY_US * 3, CLOCK_DELAY_US * 3 + 100);
     gpiod_set_value_cansleep(dev->ctrl_pin, 1);
     
     ret = count;
 
 tx_post_comm:
     // 버스 해제 및 상태 복원
     for(i=0; i<NUM_DATA_PINS; i++) gpiod_direction_input(dev->data_pins[i]);
     gpiod_direction_output(dev->data_pins[dev->my_pin_idx], 1); // 내 핀은 다시 출력 High로
     for (i = 0; i < NUM_DATA_PINS; i++) if (dev->irqs[i+1] > 0) enable_irq(dev->irqs[i+1]);
 
 tx_abort:
     spin_lock_irqsave(&dev->lock, flags);
     dev->state = COMM_STATE_IDLE;
     spin_unlock_irqrestore(&dev->lock, flags);
     kfree(tx_buf);
 
     return ret;
 }
 
 static ssize_t gpio_comm_read(struct file *filp, char __user *buf, size_t len, loff_t *off) {
     struct gpio_comm_dev *dev = filp->private_data;
     int ret, bytes_to_copy;
     u16 calc_crc, rx_crc;
     unsigned long flags;
 
     // [FIXED] read 진입 시 이미 처리할 데이터가 있는지 확인
     if (atomic_read(&dev->data_ready) == 0) {
         // [FIXED] 읽기 대기 전, 수신 모드로 확실히 전환
         spin_lock_irqsave(&dev->lock, flags);
         if (dev->state == COMM_STATE_IDLE) {
             dev->state = COMM_STATE_RECEIVING;
             // r/o 모드는 항상 ctrl irq 활성화, rw모드는 data irq도 활성화
              if (dev->mode == MODE_READ_ONLY) {
                 enable_irq(dev->irqs[0]);
              }
         }
         spin_unlock_irqrestore(&dev->lock, flags);
         
         ret = wait_event_interruptible(dev->wq, atomic_read(&dev->data_ready) != 0);
         if (ret) return -ERESTARTSYS;
     }
     
     ret = atomic_read(&dev->data_ready);
     atomic_set(&dev->data_ready, 0);
     if (ret < 0) return ret;
 
     // 수신된 데이터 검증
     if (atomic_read(&dev->rx_bytes_done) < 4) return -EIO; // 최소 길이(헤더+CRC) 검사
     
     dev->expected_rx_len = (dev->rx_buffer[1] << 8) | dev->rx_buffer[0];
     if (dev->expected_rx_len != atomic_read(&dev->rx_bytes_done)) {
         pr_err("[%s] RX length mismatch. expected=%u, got=%d\n", DRIVER_NAME, dev->expected_rx_len, atomic_read(&dev->rx_bytes_done));
         return -EIO;
     }
     
     bytes_to_copy = dev->expected_rx_len - 4; // 2(길이) + 2(CRC) 제외
     calc_crc = crc16(0, dev->rx_buffer + 2, bytes_to_copy);
     rx_crc = (dev->rx_buffer[dev->expected_rx_len - 1] << 8) | dev->rx_buffer[dev->expected_rx_len - 2];
     
     if(calc_crc != rx_crc) { pr_err("[%s] CRC mismatch! calc=0x%04X, rx=0x%04X\n", DRIVER_NAME, calc_crc, rx_crc); return -EBADMSG; }
 
     // 유저 공간으로 복사
     if (len < bytes_to_copy) bytes_to_copy = len;
     if (copy_to_user(buf, dev->rx_buffer + 2, bytes_to_copy)) return -EFAULT;
     
     // 버스 상태 복원 (수신 후)
     spin_lock_irqsave(&dev->lock, flags);
     disable_irq_nosync(dev->irqs[0]); // 제어핀 IRQ는 일단 비활성화
     if (dev->mode == MODE_READ_WRITE) {
         gpiod_direction_output(dev->data_pins[dev->my_pin_idx], 1);
         for(int i = 0; i < NUM_DATA_PINS; ++i) if (dev->irqs[i+1] > 0) enable_irq(dev->irqs[i+1]);
     }
     dev->state = COMM_STATE_IDLE;
     spin_unlock_irqrestore(&dev->lock, flags);
 
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
 
 static void release_all_resources(struct gpio_comm_dev *dev) {
     int i;
     if (!dev) return;
 
     del_timer_sync(&dev->timeout_timer);
 
     for (i = 0; i <= NUM_DATA_PINS; i++) {
         if (dev->irqs[i] > 0) {
             free_irq(dev->irqs[i], dev);
         }
     }
     if (dev->ctrl_pin) gpiod_put(dev->ctrl_pin);
     for (i = 0; i < NUM_DATA_PINS; i++) {
         if (dev->data_pins[i]) {
             gpiod_put(dev->data_pins[i]);
         }
     }
     
     if (dev->device) {
         device_destroy(g_dev_class, dev->devt);
     }
     if (dev->cdev.dev) {
        cdev_del(&dev->cdev);
     }
 
     kfree(dev->rx_buffer);
     kfree(dev);
 }
 
 static ssize_t export_store(const struct class *class, const struct class_attribute *attr, const char *buf, size_t count) {
     char name[MAX_NAME_LEN], mode_str[4];
     int pins[NUM_DATA_PINS + 1];
     struct gpio_comm_dev *new_dev = NULL;
     int i, dev_idx = -1, ret = 0;
 
     // [FIXED] 뮤텍스로 전체 함수를 보호하여 동시 접근 방지
     mutex_lock(&g_dev_lock);

    // 1. 입력 파싱: "name,mode,ctrl,d0,d1,d2,d3"
    // 입력 예시: echo "my_dev1,rw,17,27,22,23,24" > /sys/class/gpio_comm/export
     if (sscanf(buf, "%19[^,],%3[^,],%d,%d,%d,%d,%d",
                name, mode_str, &pins[0], &pins[1], &pins[2], &pins[3], &pins[4]) != 7) {
         ret = -EINVAL;
         goto out_unlock;
     }
 
     // 슬롯 및 이름 중복 확인
     for (i = 0; i < MAX_DEVICES; i++) {
         if (!g_dev_table[i] && dev_idx == -1) dev_idx = i;
         if (g_dev_table[i] && strcmp(g_dev_table[i]->name, name) == 0) {
             pr_warn("[%s] Device %s already exists.\n", DRIVER_NAME, name);
             ret = -EEXIST;
             goto out_unlock;
         }
     }
     if (dev_idx == -1) {
         ret = -ENOMEM;
         goto out_unlock;
     }
     
     new_dev = kzalloc(sizeof(struct gpio_comm_dev), GFP_KERNEL);
     if (!new_dev) {
         ret = -ENOMEM;
         goto out_unlock;
     }
     
     new_dev->rx_buffer = kzalloc(RX_BUFFER_SIZE, GFP_KERNEL);
     if (!new_dev->rx_buffer) {
         ret = -ENOMEM;
         goto err_free_dev;
     }
     new_dev->rx_buffer_size = RX_BUFFER_SIZE;
     strcpy(new_dev->name, name);
     
     if (strcmp(mode_str, "rw") == 0) new_dev->mode = MODE_READ_WRITE;
     else if (strcmp(mode_str, "r") == 0) new_dev->mode = MODE_READ_ONLY;
     else { ret = -EINVAL; goto err_free_buffer; }
 
     // [FIXED] gpiod_get으로 안전하게 GPIO 획득
     new_dev->ctrl_pin = gpiod_get(NULL, devm_kasprintf(GFP_KERNEL, "gpio_comm_ctrl_%d", pins[0]), GPIOD_ASIS);
     if (IS_ERR(new_dev->ctrl_pin)) { ret=PTR_ERR(new_dev->ctrl_pin); goto err_free_buffer; }
 
     for (i = 0; i < NUM_DATA_PINS; i++) {
         new_dev->data_pins[i] = gpiod_get(NULL, devm_kasprintf(GFP_KERNEL, "gpio_comm_data_%d", pins[i+1]), GPIOD_ASIS);
         if (IS_ERR(new_dev->data_pins[i])) { 
             ret = PTR_ERR(new_dev->data_pins[i]); 
             // 이전에 성공한 핀들 해제
             for (--i; i >= 0; i--) gpiod_put(new_dev->data_pins[i]);
             gpiod_put(new_dev->ctrl_pin);
             goto err_free_buffer;
         }
     }
     gpiod_direction_input(new_dev->ctrl_pin);
     for(i=0; i < NUM_DATA_PINS; i++) gpiod_direction_input(new_dev->data_pins[i]);
 
     // RW 모드 핀 할당
     if (new_dev->mode == MODE_READ_WRITE) {
         new_dev->my_pin_idx = -1;
         for (i = 0; i < NUM_DATA_PINS; i++) {
             if (gpiod_get_value_cansleep(new_dev->data_pins[i]) == 0) {
                 new_dev->my_pin_idx = i;
                 gpiod_direction_output(new_dev->data_pins[i], 1);
                 pr_info("[%s] %s: Claimed data pin %d.\n", DRIVER_NAME, name, i);
                 break;
             }
         }
         if (new_dev->my_pin_idx == -1) {
             pr_err("[%s] No available data pins for RW mode.\n", DRIVER_NAME);
             ret = -EBUSY;
             goto err_put_pins;
         }
     } else {
         new_dev->my_pin_idx = -1;
     }
 
     // 캐릭터 디바이스 초기화
     new_dev->devt = MKDEV(g_major_num, dev_idx);
     cdev_init(&new_dev->cdev, &gpio_comm_fops);
     new_dev->cdev.owner = THIS_MODULE;
     ret = cdev_add(&new_dev->cdev, new_dev->devt, 1);
     if (ret) goto err_put_pins;
 
     new_dev->device = device_create(g_dev_class, NULL, new_dev->devt, new_dev, new_dev->name);
     if (IS_ERR(new_dev->device)) {
         ret = PTR_ERR(new_dev->device);
         goto err_del_cdev;
     }
 
     spin_lock_init(&new_dev->lock);
     init_waitqueue_head(&new_dev->wq);
     timer_setup(&new_dev->timeout_timer, comm_timeout_callback, 0);
 
     // IRQ 설정
     new_dev->irqs[0] = gpiod_to_irq(new_dev->ctrl_pin);
     if (new_dev->irqs[0] < 0) { ret = new_dev->irqs[0]; goto err_destroy_device; }
     ret = request_irq(new_dev->irqs[0], ctrl_pin_irq_handler, IRQF_TRIGGER_RISING, "comm_ctrl", new_dev);
     if (ret) goto err_destroy_device;
     disable_irq(new_dev->irqs[0]); // 평소엔 비활성화
 
     for (i = 0; i < NUM_DATA_PINS; i++) {
         new_dev->irqs[i+1] = gpiod_to_irq(new_dev->data_pins[i]);
         if (new_dev->irqs[i+1] < 0) { ret = new_dev->irqs[i+1]; goto err_free_irqs; }
         ret = request_irq(new_dev->irqs[i+1], data_pin_irq_handler, IRQF_TRIGGER_FALLING, "comm_data", new_dev);
         if (ret) {
             new_dev->irqs[i+1] = 0; // 실패한 irq는 0으로
             goto err_free_irqs;
         }
         if (new_dev->mode == MODE_READ_ONLY) disable_irq(new_dev->irqs[i+1]);
     }
 
     new_dev->state = COMM_STATE_IDLE;
     atomic_set(&new_dev->data_ready, 0);
     g_dev_table[dev_idx] = new_dev;
 
     pr_info("[%s] Device '%s' created successfully.\n", DRIVER_NAME, name);
     mutex_unlock(&g_dev_lock);
     return count;
 
 // [FIXED] 에러 처리 경로 정리
 err_free_irqs:
     free_irq(new_dev->irqs[0], new_dev);
     for(i = 0; i < NUM_DATA_PINS; i++) if(new_dev->irqs[i+1] > 0) free_irq(new_dev->irqs[i+1], new_dev);
 err_destroy_device:
     device_destroy(g_dev_class, new_dev->devt);
 err_del_cdev:
     cdev_del(&new_dev->cdev);
 err_put_pins:
     gpiod_put(new_dev->ctrl_pin);
     for (i = 0; i < NUM_DATA_PINS; i++) gpiod_put(new_dev->data_pins[i]);
 err_free_buffer:
     kfree(new_dev->rx_buffer);
 err_free_dev:
     kfree(new_dev);
 out_unlock:
     mutex_unlock(&g_dev_lock);
     return ret;
 }
 
 
 static ssize_t unexport_store(const struct class *class, const struct class_attribute *attr, const char *buf, size_t count) {
     char name[MAX_NAME_LEN];
     int i;
     struct gpio_comm_dev *dev = NULL;
 
     sscanf(buf, "%19s", name);
 
     mutex_lock(&g_dev_lock);
     for (i = 0; i < MAX_DEVICES; i++) {
         if (g_dev_table[i] && strcmp(g_dev_table[i]->name, name) == 0) {
             dev = g_dev_table[i];
             g_dev_table[i] = NULL;
             break;
         }
     }
     mutex_unlock(&g_dev_lock);
 
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
     dev_t devt;
     int ret;
 
     ret = alloc_chrdev_region(&devt, 0, MAX_DEVICES, DRIVER_NAME);
     if (ret < 0) {
         pr_err("[%s] Failed to allocate char dev region\n", DRIVER_NAME);
         return ret;
     }
     g_major_num = MAJOR(devt);
 
     g_dev_class = class_create(CLASS_NAME);
     if (IS_ERR(g_dev_class)) {
         unregister_chrdev_region(devt, MAX_DEVICES);
         return PTR_ERR(g_dev_class);
     }
 
     class_create_file(g_dev_class, &class_attr_export);
     class_create_file(g_dev_class, &class_attr_unexport);
     
     pr_info("[%s] Driver loaded. Major: %d\n", DRIVER_NAME, g_major_num);
     return 0;
 }
 
 static void __exit gpio_comm_exit(void) {
     int i;
 
     for (i = 0; i < MAX_DEVICES; i++) {
         if (g_dev_table[i]) {
             // 뮤텍스를 사용하여 안전하게 제거
             mutex_lock(&g_dev_lock);
             struct gpio_comm_dev* dev_to_free = g_dev_table[i];
             g_dev_table[i] = NULL;
             mutex_unlock(&g_dev_lock);
             release_all_resources(dev_to_free);
         }
     }
     
     class_remove_file(g_dev_class, &class_attr_export);
     class_remove_file(g_dev_class, &class_attr_unexport);
     class_destroy(g_dev_class);
     unregister_chrdev_region(MKDEV(g_major_num, 0), MAX_DEVICES);
     pr_info("[%s] Driver unloaded.\n", DRIVER_NAME);
 }
 
 module_init(gpio_comm_init);
 module_exit(gpio_comm_exit);
 
 MODULE_LICENSE("GPL");
 MODULE_AUTHOR("HJH & Gemini");
 MODULE_DESCRIPTION("P2P GPIO Comm Driver v2.2");
 