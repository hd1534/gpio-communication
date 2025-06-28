/**
 * @file gpio_comm_drv.c
 * @brief 5-wire 멀티-디바이스 P2P GPIO 통신 드라이버
 * @author HJH (Original Concept), Gemini (Implemented & Refactored)
 * @version 3.0
 *
 * @note
 * - 이 드라이버는 1개의 제어핀과 4개의 데이터핀을 공유하는 P2P 통신 프로토콜을 구현합니다.
 * - 사용법:
 * 1. 드라이버 로드: insmod gpio_comm_drv.ko
 * 2. 디바이스 생성 (sysfs):
 * echo "my_dev1,-1,17,27,22,23,24" > /sys/class/gpio_comm/export
 * (이름, 전송 시그널을 보낼 핀(-1은 readonly), 제어핀 bcm, 데이터핀1 bcm, 데이터핀2 bcm, 데이터핀3 bcm, 데이터핀4 bcm)
 * 3. 생성된 디바이스 노드(/dev/my_dev1)를 통해 read/write 수행
 * 
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
 
 #define MAX_DEVICES 10          // 생성 가능한 최대 디바이스 수
 #define MAX_NAME_LEN 20         // 디바이스 이름 최대 길이
 #define NUM_DATA_PINS 4         // 데이터 핀 개수
 
 #define RX_BUFFER_SIZE 1024     // 수신 버퍼 크기. 최대 패킷 크기보다 커야 함.
 #define CLOCK_DELAY_US 500     // 1 클럭의 길이 (us). 이 값이 통신 속도를 결정.  (테스트 해보니 500은 안정적으로 되는것 같고 300부터는 가끔식 끊김)
 
 // 클럭 신호의 유효성을 판단하기 위한 시간 경계값 정의
 // ktime_t는 64비트 정수(ns)이므로, 오버플로우를 방지하기 위해 LL(long long) 접미사 사용
 #define NORMAL_CLK_MAX_US (CLOCK_DELAY_US * 15LL / 10)    // 1.5배까지는 일반 클럭으로 간주
 #define SLOW_CLK_MIN_US   (CLOCK_DELAY_US * 25LL / 10)    // 2.5배부터는 느린 클럭(EOT 신호)으로 간주
 #define TIMEOUT_MIN_US    (CLOCK_DELAY_US * 10LL)         // 10배 이상은 타임아웃으로 간주 (사용되지 않음, timeout_timer로 대체)
 

 // 라즈베리 파이의 GPIO 컨트롤러(BCM2835)의 기본 번호. 커널 내부에서 GPIO를 식별할 때 사용.
 // `gpioinfo` 명령어로 실제 base를 확인해야 할 수 있음.
 #define GPIOCHIP_BASE 512

 // Start Of Transmission, End Of Transmission 니블 정의
 #define SOT_NIBBLE 0b1111
 #define EOT_NIBBLE 0b1010


 // =================================================================
 // 2. Enum 및 구조체 정의
 // =================================================================
 
 // 디바이스의 현재 통신 상태를 나타내는 상태 머신
 enum comm_state {
    COMM_STATE_UNINITIALIZED, // 초기화 이전 상태
    COMM_STATE_IDLE,          // 유휴 상태. 읽기/쓰기 시작 가능.
    COMM_STATE_WAIT_BUS,      // [v2.2] 쓰기 요청 후, 다른 장치가 버스를 양보하기를 기다리는 상태
    COMM_STATE_SENDING,       // 데이터 송신 중인 상태
    COMM_STATE_WAIT_SOT,      // [v2.3] SOT 신호를 기다리는 상태
    COMM_STATE_RECEIVING,     // 데이터 수신 중인 상태
    COMM_STATE_DONE,          // 작업 완료 (사용되지 않음)
    COMM_STATE_ERROR          // 오류 발생 상태 (사용되지 않음)
 };
 
 // 개별 GPIO 통신 디바이스를 표현하는 핵심 구조체
 struct gpio_comm_dev {
     char name[MAX_NAME_LEN];   // 디바이스 이름 (/dev/`name`)
     struct cdev cdev;          // 캐릭터 디바이스 구조체
     struct device *device;     // sysfs에 등록된 디바이스 구조체
     dev_t devt;                // 디바이스 번호 (주+부번호)
 
     // GPIO 핀 디스크립터. gpiod API를 통해 핀을 제어하기 위한 핸들.
     struct gpio_desc *ctrl_pin;
     struct gpio_desc *data_pins[NUM_DATA_PINS];
     int my_pin_idx;            // 전송 요청시 사용할 핀의 인덱스 (0-3). -1은 read only.
 
     int ctrl_irq; // 제어핀의 IRQ 번호
 
     // --- 동기화 및 상태 관리 ---
     enum comm_state state;     // 현재 통신 상태
     spinlock_t lock;           // 디바이스 내부 데이터(state 등) 보호용 스핀락. IRQ 핸들러와 동시 접근을 막기 위함.
     wait_queue_head_t wq;      // `read` 동작 시 데이터가 준비될 때까지 프로세스를 재우기 위한 대기 큐.
 
     // --- 수신 관련 ---
     u8 *rx_buffer;             // 수신된 데이터를 저장하는 버퍼
     size_t rx_buffer_size;     // rx_buffer의 크기
     u16 expected_rx_len;       // 수신할 것으로 예상되는 총 패킷 길이 (헤더에서 읽음)
     atomic_t rx_bytes_done;    // 수신 완료된 바이트 수
     atomic_t rx_bits_done;     // 현재 바이트 내에서 수신된 비트 수 (니블 단위 수신)
     atomic_t data_ready;       // 데이터 수신 완료 플래그 (0: 대기, 1: 성공, <0: 에러). wait_queue의 조건 변수.
     ktime_t last_irq_time;     // 마지막 IRQ 발생 시간. 클럭 간격(속도)을 계산하기 위함.
 
     struct timer_list timeout_timer; // 통신 타임아웃을 감지하기 위한 커널 타이머

     // timestamp
     ktime_t rx_start_time;
 };
 


 // =================================================================
 // 3. 전역 변수
 // =================================================================
 
 // 생성된 디바이스들의 포인터를 저장하는 테이블
 static struct gpio_comm_dev *g_dev_table[MAX_DEVICES];
 static int g_major_num;        // 할당받은 주 번호
 static struct class *g_dev_class; // sysfs에 "/sys/class/gpio_comm"을 생성하기 위한 클래스 구조체

 static int g_assigned_pins[NUM_DATA_PINS + 1] = {0};
 
 // [v2.2] export/unexport 동작 중 g_dev_table 접근을 보호하기 위한 전역 뮤텍스.
 // sysfs 콜백은 sleep이 가능하므로 spinlock이 아닌 mutex를 사용.
 static DEFINE_MUTEX(g_dev_lock);
 


 // =================================================================
 // 4. 프로토타입 선언
 // =================================================================
 
 static int release_all_resources(struct gpio_comm_dev *dev);
 

 
 // =================================================================
 // 5. 하위 레벨 통신 함수 (실제 GPIO 제어)
 // =================================================================

 // 4개의 데이터 핀에 4비트(니블) 데이터를 출력
 static void write_4bits(struct gpio_comm_dev *dev, u8 data) {
     if (unlikely(!dev)) {
         pr_err("[%s] ERROR: write_4bits: NULL device pointer\n", DRIVER_NAME);
         return;
     }
     pr_debug("[%s] %s: write_4bits: Writing data: 0x%02X\n", DRIVER_NAME, dev->name, data);
     gpiod_set_value_cansleep(dev->data_pins[0], data & 0x01);
     gpiod_set_value_cansleep(dev->data_pins[1], (data >> 1) & 0x01);
     gpiod_set_value_cansleep(dev->data_pins[2], (data >> 2) & 0x01);
     gpiod_set_value_cansleep(dev->data_pins[3], (data >> 3) & 0x01);
 }
 
 // 4개의 데이터 핀에서 4비트(니블) 데이터를 읽어옴
 static u8 read_4bits(struct gpio_comm_dev *dev) {
     u8 data = 0;
     if (unlikely(!dev)) {
         pr_err("[%s] ERROR: read_4bits: NULL device pointer\n", DRIVER_NAME);
         return 0xFF;
     }
     data |= gpiod_get_value_cansleep(dev->data_pins[0]) & 0x01;
     data |= (gpiod_get_value_cansleep(dev->data_pins[1]) & 0x01) << 1;
     data |= (gpiod_get_value_cansleep(dev->data_pins[2]) & 0x01) << 2;
     data |= (gpiod_get_value_cansleep(dev->data_pins[3]) & 0x01) << 3;
     pr_debug("[%s] %s: read_4bits: Read data: 0x%02X\n", DRIVER_NAME, dev->name, data);
     return data;
 }
 
 // 제어 핀을 Low -> High로 토글하여 1 클럭 신호를 생성
 static void toggle_ctrl_clock(struct gpio_comm_dev *dev) {
     if (unlikely(!dev || !dev->ctrl_pin)) {
         pr_err("[%s] %s: ERROR: toggle_ctrl_clock: Invalid device or control pin\n", DRIVER_NAME, dev->name);
         return;
     }
     pr_debug("[%s] %s: toggle_ctrl_clock: Toggling control pin\n", DRIVER_NAME, dev->name);

     gpiod_set_value_cansleep(dev->ctrl_pin, 0);
     udelay(CLOCK_DELAY_US);
     gpiod_set_value_cansleep(dev->ctrl_pin, 1);
     udelay(CLOCK_DELAY_US);
 }
 

 
 // =================================================================
 // 6. Interrupt 및 타이머 핸들러
 // =================================================================
 
 /**
  * @brief 통신 타임아웃 콜백 함수.
  * @param t 타임아웃이 발생한 타이머
  * @note mod_timer로 설정된 시간이 지나면 이 함수가 호출됨.
  * 주로 수신 상태에서 상대방이 데이터를 보내다 멈췄을 때 발생.
  */
 static void comm_timeout_callback(struct timer_list *t) {
     struct gpio_comm_dev *dev = from_timer(dev, t, timeout_timer);
     unsigned long flags;
     
     if (unlikely(!dev)) {
         pr_err("[%s] ERROR: comm_timeout_callback: NULL device pointer\n", DRIVER_NAME);
         return;
     }
     
     pr_warn("[%s] %s: Communication timeout! Current state: %d\n", 
             DRIVER_NAME, dev->name, dev->state);
 
     spin_lock_irqsave(&dev->lock, flags);
     if (dev->state == COMM_STATE_RECEIVING) {
         pr_debug("[%s] %s: Waking up read() with timeout error\n", DRIVER_NAME, dev->name);
         atomic_set(&dev->data_ready, -ETIMEDOUT); // 에러 코드로 wake_up
         dev->state = COMM_STATE_IDLE; // 상태를 유휴로 복원
         wake_up_interruptible(&dev->wq); // 대기 중인 read()를 깨움
     } else {
         pr_debug("[%s] %s: Timeout in state %d, ignoring\n", 
                 DRIVER_NAME, dev->name, dev->state);
     }
     spin_unlock_irqrestore(&dev->lock, flags);
 }
 
 /**
  * @brief 제어 핀(Ctrl)의 Rising-edge 인터럽트 핸들러.
  * @param irq 발생한 IRQ 번호
  * @param dev_id IRQ 등록 시 전달된 디바이스 포인터
  * @note 데이터 니블 수신, EOT(End of Transmission) 감지 역할을 수행.
  */
 static irqreturn_t ctrl_pin_irq_handler(int irq, void *dev_id) {
     if (!dev_id) {
         pr_err("[%s] ERROR: ctrl_pin_irq_handler: dev_id is NULL\n", DRIVER_NAME);
         return IRQ_NONE;
     }
     struct gpio_comm_dev *dev = (struct gpio_comm_dev *)dev_id;
     ktime_t now;
     s64 delta_us; // 이전 클럭과의 시간 차이 (us)
     u8 received_nibble;
     unsigned long flags;
     int current_byte_idx;
     
     pr_debug("[%s] %s: ctrl_pin_irq_handler: dev=%p, state=%d\n", DRIVER_NAME, dev->name, dev, dev->state);
 
     // 클럭이 감지되었으므로, 타임아웃 타이머를 리셋하여 시간을 연장.
     pr_debug("[%s] %s: ctrl_pin_irq_handler: Resetting timeout timer\n", DRIVER_NAME, dev->name);
     mod_timer(&dev->timeout_timer, jiffies + msecs_to_jiffies(50));
 
     now = ktime_get();
     delta_us = ktime_us_delta(now, dev->last_irq_time);
     dev->last_irq_time = now;
     pr_debug("[%s] %s: ctrl_pin_irq_handler: Delta time = %lld us\n", DRIVER_NAME, dev->name, delta_us);
 
     received_nibble = read_4bits(dev);

     // SOT 감지
     if (dev->state == COMM_STATE_WAIT_SOT) {
         if (received_nibble == SOT_NIBBLE) {
             dev->state = COMM_STATE_RECEIVING;
             dev->rx_start_time = ktime_get();
             pr_debug("[%s] %s: ctrl_pin_irq_handler: SOT detected. Switched to RECEIVING state\n", DRIVER_NAME, dev->name);
         }
         else {
            pr_debug("[%s] %s: ctrl_pin_irq_handler: SOT not detected. waiting for SOT\n", DRIVER_NAME, dev->name);
         }
         return IRQ_HANDLED;
     }
     
     // 너무 짧은 간격의 신호는 노이즈로 간주하고 무시 (디바운싱)
     if (delta_us < (CLOCK_DELAY_US / 2)) return IRQ_HANDLED;
 
 
     // EOT 감지: 클럭 간격이 정상보다 길고(SLOW_CLK), received_nibble == EOT_NIBBLE이면 EOT.
     if (delta_us > SLOW_CLK_MIN_US) {
         if (received_nibble == EOT_NIBBLE) {
             pr_info("[%s] %s: EOT detected.\n", DRIVER_NAME, dev->name);
             atomic_set(&dev->data_ready, 1); // 성공적으로 수신 완료
         } else {
             pr_warn("[%s] %s: Invalid EOT signal (data: 0x%02X).\n", DRIVER_NAME, dev->name, received_nibble);
             atomic_set(&dev->data_ready, -EIO); // 잘못된 신호
         }
         del_timer_sync(&dev->timeout_timer); // 통신이 끝났으므로 타이머 완전 제거
         wake_up_interruptible(&dev->wq); // 대기 중인 read() 깨우기
         return IRQ_HANDLED;
     }
 
     // 일반 데이터 수신 처리 (정상 클럭 간격)
     spin_lock_irqsave(&dev->lock, flags);
     if (dev->state != COMM_STATE_RECEIVING) {
         spin_unlock_irqrestore(&dev->lock, flags);
         return IRQ_HANDLED; // 수신 상태가 아닐 때 들어온 클럭은 무시
     }
     
     current_byte_idx = atomic_read(&dev->rx_bytes_done);
 
     // 버퍼 오버플로우 방지
     if (current_byte_idx >= dev->rx_buffer_size) {
         pr_err("[%s] %s: RX buffer overflow!\n", DRIVER_NAME, dev->name);
         atomic_set(&dev->data_ready, -ENOMEM);
         del_timer_sync(&dev->timeout_timer);
         wake_up_interruptible(&dev->wq);
         spin_unlock_irqrestore(&dev->lock, flags);
         return IRQ_HANDLED;
     }
 
     // 1바이트 = 2니블. 첫번째 니블은 상위 4비트, 두번째 니블은 하위 4비트에 저장.
     if (atomic_read(&dev->rx_bits_done) == 0) { // 상위 4비트 (첫번째 니블)
         dev->rx_buffer[current_byte_idx] = received_nibble << 4;
         atomic_inc(&dev->rx_bits_done);
     } else { // 하위 4비트 (두번째 니블)
         dev->rx_buffer[current_byte_idx] |= received_nibble;
         atomic_set(&dev->rx_bits_done, 0); // 비트 카운터 리셋
         atomic_inc(&dev->rx_bytes_done);   // 바이트 카운터 증가
     }
     spin_unlock_irqrestore(&dev->lock, flags);
 
     return IRQ_HANDLED;
 }


 // =================================================================
 // 7. file_operations 구현
 // =================================================================
 
 static int gpio_comm_open(struct inode *inode, struct file *filp) {
     struct gpio_comm_dev *dev;
     
     if (!inode || !filp) {
         pr_err("[%s] %s: ERROR: gpio_comm_open: inode or filp is NULL\n", DRIVER_NAME, dev->name);
         return -EINVAL;
     }
     
     // container_of 매크로를 이용해 cdev 멤버 변수 주소로부터 부모 구조체(gpio_comm_dev)의 주소를 계산.
     dev = container_of(inode->i_cdev, struct gpio_comm_dev, cdev);
     if (!dev) {
         pr_err("[%s] %s: ERROR: gpio_comm_open: Failed to get device from inode\n", DRIVER_NAME, dev->name);
         return -ENODEV;
     }
     
     filp->private_data = dev; // file 구조체에 디바이스 포인터를 저장하여 read/write 등에서 사용.
     pr_info("[%s] %s: Device '%s' opened. Device ptr: %p\n", DRIVER_NAME, dev->name, dev->name, dev);
     return 0;
 }
 
 static int gpio_comm_release(struct inode *inode, struct file *filp) {
     struct gpio_comm_dev *dev = filp->private_data;
     
     if (!dev) {
         pr_err("[%s] %s: ERROR: gpio_comm_release: NULL device pointer\n", DRIVER_NAME, dev->name);
         return -EINVAL;
     }
     
     pr_info("[%s] %s: Device '%s' (ptr: %p) released.\n", 
             DRIVER_NAME, dev->name, dev->name, dev);
     
     // 상태 초기화
     spin_lock_irq(&dev->lock);
     if (dev->state != COMM_STATE_IDLE) {
         pr_debug("[%s] %s: Cleaning up non-IDLE state: %d\n", 
                 DRIVER_NAME, dev->name, dev->state);
         dev->state = COMM_STATE_IDLE;
     }
     // 모든 핀을 입력으로 전환 (충돌 방지?)
     gpiod_direction_input(dev->ctrl_pin);
     for(int i=0; i<NUM_DATA_PINS; i++) gpiod_direction_input(dev->data_pins[i]);
     spin_unlock_irq(&dev->lock);
     
     return 0;
 }
 
 /**
  * @brief write() 시스템 콜 핸들러
  * @note 프로토콜에 따라 데이터를 패킷으로 만들어 GPIO로 전송.
  */
 static ssize_t gpio_comm_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos) {
     pr_debug("[%s] gpio_comm_write: Entry. count=%zu, f_pos=%lld\n", 
              DRIVER_NAME, count, f_pos ? *f_pos : 0);
     
     if (!filp || !filp->private_data) {
         pr_err("[%s] ERROR: gpio_comm_write: Invalid file pointer or private_data\n", DRIVER_NAME);
         return -EBADF;
     }
     if (!buf) {
         pr_err("[%s] ERROR: gpio_comm_write: NULL buffer pointer\n", DRIVER_NAME);
         return -EFAULT;
     }
     
     struct gpio_comm_dev *dev = filp->private_data;
     pr_debug("[%s] %s: gpio_comm_write: dev=%p, count=%zu, my_pin_idx=%d, current state=%d\n", 
              DRIVER_NAME, dev->name, dev, count, dev->my_pin_idx, dev->state);
     
     u8 *tx_buf = NULL;
     int i, ret = 0;
     u16 crc;
     size_t total_len = count + 4; // 2(길이 헤더) + count(데이터) + 2(CRC)
     unsigned long flags;
     
     pr_debug("[%s] %s: Total packet length: %zu (data: %zu + header: 2 + CRC: 2)\n",
              DRIVER_NAME, dev->name, total_len, count);
 
     // mypin이 -1이면 쓰기 불가
     if (dev->my_pin_idx == -1) {
         pr_err("[%s] %s: Write attempted without mypin (mypin=%d)\n", DRIVER_NAME, dev->name, dev->my_pin_idx);
         return -EPERM;
     }

 
     // 1. 커널 공간에 전송 버퍼 할당
     pr_debug("[%s] %s: Allocating TX buffer of size %zu bytes\n", DRIVER_NAME, dev->name, total_len);
     tx_buf = kmalloc(total_len, GFP_KERNEL);
     if (!tx_buf) {
         pr_err("[%s] %s: Failed to allocate TX buffer (size=%zu)\n", DRIVER_NAME, dev->name, total_len);
         return -ENOMEM;
     }
     pr_debug("[%s] %s: TX buffer allocated at %p\n", DRIVER_NAME, dev->name, tx_buf);


     // 2. 유저 공간에서 커널 버퍼로 데이터 복사 및 패킷 구성
     pr_debug("[%s] %s: Copying %zu bytes from user space to kernel buffer\n", DRIVER_NAME, dev->name, count);
     if (copy_from_user(tx_buf + 2, buf, count)) {
         pr_err("[%s] %s: Failed to copy %zu bytes from user space\n", DRIVER_NAME, dev->name, count);
         ret = -EFAULT;
         goto tx_abort;
     }
     pr_debug("[%s] %s: Successfully copied data from user space\n", DRIVER_NAME, dev->name);
     tx_buf[0] = (u8)(total_len & 0xFF);         // 헤더: 길이(하위)
     tx_buf[1] = (u8)((total_len >> 8) & 0xFF);     // 3. CRC 계산 및 패킷 완성
     crc = crc16(0, tx_buf, total_len - 2); // CRC16 계산 (마지막 2바이트 제외)
     tx_buf[total_len - 2] = (u8)(crc & 0xFF);
     tx_buf[total_len - 1] = (u8)((crc >> 8) & 0xFF);
     pr_debug("[%s] %s: CRC calculated: 0x%04X\n", DRIVER_NAME, dev->name, crc); // CRC(상위)
     

     // 3. 버스 점유를 위한 준비
     // 버스 상태 확인 및 점유 시작 (경쟁 상태 방지)
     spin_lock_irqsave(&dev->lock, flags);
     if (dev->state != COMM_STATE_IDLE) { // 다른 작업 중이면 -EBUSY
         spin_unlock_irqrestore(&dev->lock, flags);
         pr_err("[%s] %s: Device is busy (state=%d), cannot start write\n", DRIVER_NAME, dev->name, dev->state);
         kfree(tx_buf);
         return -EBUSY;
     }
     dev->state = COMM_STATE_WAIT_BUS; // 상태 변경
     spin_unlock_irqrestore(&dev->lock, flags);
     pr_debug("[%s] %s: Device state changed to WAIT_BUS for write operation\n", DRIVER_NAME, dev->name);

     // 안전하게 3번정도 확인
     for (int i = 0; i < 3; i++) {
        if (read_4bits(dev) != 0x00 || gpiod_get_value(dev->ctrl_pin) != 0) {
            pr_err("[%s] %s: Bus already in use! Aborting TX.\n", DRIVER_NAME, dev->name);
            ret = -EBUSY; // 다른 디바이스가 버스 점유 중
            goto tx_post_comm;
        }
        udelay(CLOCK_DELAY_US);
     }

     // 버스 사용 요청: 내 핀을 High로 설정.
     gpiod_direction_output(dev->data_pins[dev->my_pin_idx], 1);
     udelay(CLOCK_DELAY_US * 5);
     
     // 충돌 감지: 요청후 버스를 읽었을 때 본인 핀을 제외한 핀이 High이면 다른 장치도 동시에 전송을 시도했다는 의미.
     if (gpiod_get_value(dev->ctrl_pin) != 0) {
        pr_err("[%s] %s: Bus collision detected!, ctrl pin is high. Aborting TX.\n", DRIVER_NAME, dev->name);
        ret = -EBUSY; // 다른 디바이스랑 충돌
        goto tx_post_comm;
     }
     for (int i = 0; i < NUM_DATA_PINS; i++) {
        if (i != dev->my_pin_idx && gpiod_get_value(dev->data_pins[i]) != 0) {
            pr_err("[%s] %s: Bus collision detected! %d pin is high. Aborting TX.\n", DRIVER_NAME, dev->name, i);
            ret = -EBUSY; // 다른 디바이스랑 충돌
            goto tx_post_comm;
        }
    }

     // 4. 전송 시작
     // 모든 데이터 핀을 출력(Low)으로 설정.
     gpiod_direction_output(dev->ctrl_pin, 0);
     for (i = 0; i < NUM_DATA_PINS; i++) gpiod_direction_output(dev->data_pins[i], 0);
     
     // 전송 시작을 알리는 예비 클럭킹 3회.
     for (i = 0; i < 3; i++) toggle_ctrl_clock(dev);
 
     // 충돌 감지: 예비 클럭 후 버스를 읽었을 때 0이 아니면 다른 장치도 동시에 전송을 시도했다는 의미.
     if (read_4bits(dev) != 0) {
         pr_err("[%s] %s: Bus collision detected2! Aborting TX.\n", DRIVER_NAME, dev->name);
         ret = -EAGAIN; // 충돌 발생, 재시도 필요
         goto tx_post_comm;
     }
 
     // 상태를 SENDING으로 변경
     spin_lock_irqsave(&dev->lock, flags);
     dev->state = COMM_STATE_SENDING;
     spin_unlock_irqrestore(&dev->lock, flags);

     // SOT 신호 전송
     write_4bits(dev, SOT_NIBBLE);
     toggle_ctrl_clock(dev);
 

     // 5. 실제 데이터 전송 (1바이트 = 2클럭)
     pr_debug("[%s] %s: Starting data transmission (%zu bytes total)\n",
              DRIVER_NAME, dev->name, total_len);
     
     for (i = 0; i < total_len; i++) {
         u8 high_nibble = tx_buf[i] >> 4;
         u8 low_nibble = tx_buf[i] & 0x0F;
         
         pr_debug("[%s] %s: TX[%d/%zu] = 0x%02X (0x%X 0x%X)\n",
                 DRIVER_NAME, dev->name, i+1, total_len, 
                 tx_buf[i], high_nibble, low_nibble);
         
         // 상위 4비트 전송
         write_4bits(dev, high_nibble);
         toggle_ctrl_clock(dev);
         
         // 하위 4비트 전송
         write_4bits(dev, low_nibble);
         toggle_ctrl_clock(dev);
     }
     

     // 6. EOT 신호 전송 (느린 클럭 + 데이터핀 모두 High)
     pr_debug("[%s] %s: Sending EOT signal\n", DRIVER_NAME, dev->name);
     write_4bits(dev, EOT_NIBBLE);
     gpiod_set_value_cansleep(dev->ctrl_pin, 0);
     usleep_range(CLOCK_DELAY_US * 3, CLOCK_DELAY_US * 3 + 100);
     gpiod_set_value_cansleep(dev->ctrl_pin, 1);
     usleep_range(CLOCK_DELAY_US * 3, CLOCK_DELAY_US * 3 + 100);
     gpiod_set_value_cansleep(dev->ctrl_pin, 0);
     write_4bits(dev, 0x00);
     
     ret = count; // 성공 시 실제 데이터 길이 반환
 
 tx_post_comm:
     // 7. 버스 해제 및 상태 복원 (전송 종료 후)
     gpiod_direction_input(dev->ctrl_pin);
     for(i=0; i<NUM_DATA_PINS; i++) {
        gpiod_direction_input(dev->data_pins[i]); // 모든 핀을 다시 입력으로
     }
 
     
 tx_abort:
     // 8. 최종 상태 복원 및 자원 해제
     spin_lock_irqsave(&dev->lock, flags);
     dev->state = COMM_STATE_IDLE;
     spin_unlock_irqrestore(&dev->lock, flags);
     pr_debug("[%s] %s: Transmission complete, freeing buffer\n",
              DRIVER_NAME, dev->name);
     kfree(tx_buf);
     
     pr_debug("[%s] %s: gpio_comm_write: Exit. Wrote %zu bytes\n",
              DRIVER_NAME, dev->name, count);

    
     if (ret < 0) {
         return ret;
     }
     
     return count;
 }
 
 /**
  * @brief read() 시스템 콜 핸들러
  * @note 완전한 패킷이 수신될 때까지 대기(block)하고, 수신 완료 시 유저 공간으로 복사.
  */
 static ssize_t gpio_comm_read(struct file *filp, char __user *buf, size_t len, loff_t *off) {
     pr_debug("[%s] gpio_comm_read: Entry. len=%zu, off=%lld\n", 
              DRIVER_NAME, len, off ? *off : 0);
     
     if (!filp || !filp->private_data) {
         pr_err("[%s] ERROR: gpio_comm_read: Invalid file pointer or private_data\n", DRIVER_NAME);
         return -EBADF;
     }
     if (!buf) {
         pr_err("[%s] ERROR: gpio_comm_read: NULL buffer pointer\n", DRIVER_NAME);
         return -EFAULT;
     }
     
     struct gpio_comm_dev *dev = filp->private_data;
     int ret, bytes_to_copy;
     u16 calc_crc, rx_crc;
     unsigned long flags;
     
    // clean up: 버퍼/카운터 리셋
    pr_debug("[%s] %s: gpio_comm_read: clean up\n", DRIVER_NAME, dev->name);
    dev->last_irq_time = ktime_get();
    atomic_set(&dev->rx_bytes_done, 0);
    atomic_set(&dev->rx_bits_done, 0);
    atomic_set(&dev->data_ready, 0);
    
    // idle 상태가 아니면 read 요청 거부.
    spin_lock_irqsave(&dev->lock, flags);
    if (dev->state != COMM_STATE_IDLE) {
        pr_err("[%s] %s: Device is not idle, rejected. current state: %d.\n", DRIVER_NAME, dev->name, dev->state);
        return -EBUSY;
    }

    // idle 상태에서 수신 대기 상태로 전환.
    dev->state = COMM_STATE_WAIT_SOT;
    pr_debug("[%s] %s: Changed state to WAIT_SOT\n", DRIVER_NAME, dev->name);

    // control irq 활성화.
    enable_irq(dev->ctrl_irq);
    pr_debug("[%s] %s: Enabling control IRQ\n", DRIVER_NAME, dev->name);
    spin_unlock_irqrestore(&dev->lock, flags);
    
    // data_ready 플래그가 0이 아닐 때까지 대기 큐에서 잠듦.
    // 인터럽트로 깨어날 수 있음 (-ERESTARTSYS).
    pr_debug("[%s] %s: Waiting for data (current data_ready=%d)...\n", DRIVER_NAME, dev->name, atomic_read(&dev->data_ready));
    ret = wait_event_interruptible(dev->wq, atomic_read(&dev->data_ready) != 0);
    if (ret) {
        pr_debug("[%s] %s: Read wait interrupted by signal (ret=%d)\n", DRIVER_NAME, dev->name, ret);
        return -ERESTARTSYS;
    }

    // control irq 비활성화.
    pr_debug("[%s] %s: Disabling control IRQ\n", DRIVER_NAME, dev->name);
    spin_lock_irqsave(&dev->lock, flags);
    disable_irq_nosync(dev->ctrl_irq);
    spin_unlock_irqrestore(&dev->lock, flags);
    
     
    // 2. 수신 결과 확인
    ret = atomic_read(&dev->data_ready);
    pr_debug("[%s] %s: Data ready with status: %d\n", DRIVER_NAME, dev->name, ret);
    pr_info("[%s] %s: total time: %lld ms, bytes: %d \n", DRIVER_NAME, dev->name, ktime_ms_delta(ktime_get(), dev->rx_start_time), atomic_read(&dev->rx_bytes_done));
            
    atomic_set(&dev->data_ready, 0); // 플래그를 다시 0으로 리셋 (다음 read를 위해)
     
     if (ret < 0) {
         pr_err("[%s] %s: Read error occurred: %d\n", 
                DRIVER_NAME, dev->name, ret);
         return ret; // 타임아웃 등 에러 발생 시 음수 값이 반환됨.
     }
 
     // 3. 수신된 데이터 검증
     if (atomic_read(&dev->rx_bytes_done) < 4) {
         pr_err("[%s] %s: Received data too short: %d bytes (minimum 4 bytes required)\n", 
                DRIVER_NAME, dev->name, atomic_read(&dev->rx_bytes_done));
         return -EIO; // 최소 길이(헤더+CRC) 검사
     }
     
     // 헤더에서 전체 길이 정보를 읽음
     dev->expected_rx_len = (dev->rx_buffer[1] << 8) | dev->rx_buffer[0];
     pr_debug("[%s] %s: Packet header - expected length: %u, actual received: %d\n", DRIVER_NAME, dev->name, dev->expected_rx_len, atomic_read(&dev->rx_bytes_done));
     
     if (dev->expected_rx_len != atomic_read(&dev->rx_bytes_done)) {
         pr_err("[%s] %s: RX length mismatch. expected=%u, got=%d\n", DRIVER_NAME, dev->name, dev->expected_rx_len, atomic_read(&dev->rx_bytes_done));
         pr_debug("[%s] %s: Dumping first 16 bytes of rx_buffer: %*ph\n", DRIVER_NAME, dev->name, 16, dev->rx_buffer);
         return -EIO; // 길이 불일치 에러
     }
     
     // CRC 검증
     bytes_to_copy = dev->expected_rx_len - 4;
     pr_debug("[%s] %s: Verifying CRC for %d bytes of payload\n", 
             DRIVER_NAME, dev->name, bytes_to_copy);
     
     calc_crc = crc16(0, dev->rx_buffer, 2 + bytes_to_copy); // CRC16 계산 (헤더 포함)

     rx_crc = (dev->rx_buffer[dev->expected_rx_len - 1] << 8) | dev->rx_buffer[dev->expected_rx_len - 2];
     
     pr_debug("[%s] %s: CRC - calculated=0x%04X, received=0x%04X\n", DRIVER_NAME, dev->name, calc_crc, rx_crc);
     
     if(calc_crc != rx_crc) { 
         pr_err("[%s] %s: CRC mismatch! calc=0x%04X, rx=0x%04X\n", DRIVER_NAME, dev->name, calc_crc, rx_crc); 
         pr_debug("[%s] %s: Dumping packet (16char): %.16s\n", DRIVER_NAME, dev->name, dev->expected_rx_len, dev->rx_buffer);
         return -EBADMSG; // CRC 불일치 에러
     }
 
     // 4. 유저 공간으로 데이터 복사
     if (len < bytes_to_copy) bytes_to_copy = len; // 사용자가 요청한 길이가 더 작으면 그만큼만 복사
     if (copy_to_user(buf, dev->rx_buffer + 2, bytes_to_copy)) return -EFAULT;
     dev->state = COMM_STATE_IDLE; // 유휴 상태로 전환

     pr_debug("[%s] %s: Read complete, set state to IDLE, returning %d bytes\n", DRIVER_NAME, dev->name, bytes_to_copy);
     return bytes_to_copy; // 복사한 바이트 수 반환
 }
 
 // 이 드라이버가 제공하는 파일 오퍼레이션 함수들의 집합
 static const struct file_operations gpio_comm_fops = {
     .owner = THIS_MODULE,
     .open = gpio_comm_open,
     .release = gpio_comm_release,
     .read = gpio_comm_read,
     .write = gpio_comm_write,
 };
 

 
 // =================================================================
 // 8. Sysfs 인터페이스 (`/sys/class/gpio_comm/export`)
 // =================================================================
 
 /**
  * @brief 디바이스와 관련된 모든 할당된 자원을 해제.
  * @note unexport 시 또는 모듈 종료 시 호출됨.
  */
 static int release_all_resources(struct gpio_comm_dev *dev) {
     int i, bcm;
     if (!dev) {
         pr_err("[%s] ERROR: release_all_resources called with NULL device\n", DRIVER_NAME);
         return -EINVAL;
     }
     pr_info("[%s] Releasing all resources for device %s (%p)\n", 
             DRIVER_NAME, dev->name, dev);
 
     // 타이머, IRQ, GPIO, 디바이스, cdev, 메모리 순으로 할당의 역순으로 해제.
     pr_debug("[%s] %s: Releasing timer\n", DRIVER_NAME, dev->name);
     del_timer_sync(&dev->timeout_timer);
 
     pr_debug("[%s] %s: Releasing IRQ\n", DRIVER_NAME, dev->name);
     if (dev->ctrl_irq) {
         free_irq(dev->ctrl_irq, dev);
     }

     pr_debug("[%s] %s: Releasing GPIOs\n", DRIVER_NAME, dev->name);
     if (dev->ctrl_pin) {
        if (gpiod_direction_input(dev->ctrl_pin)) {
            pr_err("[%s] %s: Failed to set ctrl pin as input\n", 
                   DRIVER_NAME, dev->name);
            return -EINVAL;
        }
     }
     for (i = 0; i < NUM_DATA_PINS; i++) {
         if (dev->data_pins[i]) {
            if (gpiod_direction_input(dev->data_pins[i])) {
                pr_err("[%s] %s: Failed to set data pin %d as input\n", 
                       DRIVER_NAME, dev->name, i);
                return -EINVAL;
            }
            bcm = desc_to_gpio(dev->data_pins[i]) - GPIOCHIP_BASE;
            g_assigned_pins[bcm] = 0;
         }
     }

     pr_debug("[%s] %s: Releasing device\n", DRIVER_NAME, dev->name);
     if (dev->device) {
         device_destroy(g_dev_class, dev->devt);
     }
     if (dev->cdev.dev) {
        cdev_del(&dev->cdev);
     }

     pr_debug("[%s] %s: Releasing memory\n", DRIVER_NAME, dev->name); 
     kfree(dev->rx_buffer);
     kfree(dev);
     return 0;
 }
 
 /**
  * @brief `export` 파일에 write 할 때 호출되는 함수 (디바이스 생성).
  * @note "이름,모드,핀번호,..." 형식의 문자열을 파싱하여 새 디바이스를 초기화.
  */
 static ssize_t export_store(const struct class *class, const struct class_attribute *attr, const char *buf, size_t count) {
     char name[MAX_NAME_LEN];
     int my_pin_idx, pins[NUM_DATA_PINS + 1];
     struct gpio_comm_dev *new_dev = NULL;
     int i, dev_idx = -1, ret = 0;
     
     pr_info("[%s] export_store called, buf: %.*s\n", 
             DRIVER_NAME, (int)min(count, (size_t)50), buf);
             
     if (!buf) {
         pr_err("[%s] ERROR: export_store: NULL buffer\n", DRIVER_NAME);
         return -EINVAL;
     }
 
     // 전역 뮤텍스로 전체 함수를 보호하여, 여러 프로세스가 동시에 디바이스를 생성/삭제하는 것을 방지.
     mutex_lock(&g_dev_lock);
 

     // 1. 입력 파싱: "name,my_pin_idx,ctrl,d0,d1,d2,d3"
     // 입력 예시: echo "my_dev1,-1,17,27,22,23,24" > /sys/class/gpio_comm/export
     pr_debug("[%s] export_store: parsing input string: %.*s\n", DRIVER_NAME, (int)min(count, (size_t)50), buf);
     if (sscanf(buf, "%19[^,],%d,%d,%d,%d,%d,%d",
                name, &my_pin_idx, &pins[0], &pins[1], &pins[2], &pins[3], &pins[4]) != 7) {
         ret = -EINVAL;
         goto out_unlock;
     }
 
     // 2. 유효성 검사 (빈 슬롯, 이름 중복)
     if (my_pin_idx < -1 || NUM_DATA_PINS <my_pin_idx) {
         pr_err("[%s] ERROR: export_store: Invalid my_pin_idx: %d\n", DRIVER_NAME, my_pin_idx);
         ret = -EINVAL;
         goto out_unlock;
     }

     pr_debug("[%s] export_store: checking for available slot and duplicate name\n", DRIVER_NAME);
     for (i = 0; i < MAX_DEVICES; i++) {
         if (!g_dev_table[i] && dev_idx == -1) dev_idx = i;
         if (g_dev_table[i] && strcmp(g_dev_table[i]->name, name) == 0) {
             pr_warn("[%s] Device %s already exists.\n", DRIVER_NAME, name);
             ret = -EEXIST;
             goto out_unlock;
         }
     }
     if (dev_idx == -1) {
         ret = -ENOMEM; // 빈 슬롯 없음
         goto out_unlock;
     }
     
     // 3. 자원 할당 시작
     pr_debug("[%s] export_store: allocating resources for new device\n", DRIVER_NAME);
     new_dev = kzalloc(sizeof(struct gpio_comm_dev), GFP_KERNEL);
     if (!new_dev) { ret = -ENOMEM; goto out_unlock; }
     
     new_dev->rx_buffer = kzalloc(RX_BUFFER_SIZE, GFP_KERNEL);
     if (!new_dev->rx_buffer) { ret = -ENOMEM; goto err_free_dev; }
     new_dev->rx_buffer_size = RX_BUFFER_SIZE;
     strcpy(new_dev->name, name);
     
     pr_debug("[%s] export_store: setting my_pin_idx to %d\n", DRIVER_NAME, my_pin_idx);
     new_dev->my_pin_idx = my_pin_idx;
 
     // GPIO 핀 획득
     pr_debug("[%s] export_store: getting control pin: %d\n", DRIVER_NAME, pins[0]);
     new_dev->ctrl_pin = gpio_to_desc(GPIOCHIP_BASE + pins[0]);
     if (IS_ERR(new_dev->ctrl_pin)) { 
        ret=PTR_ERR(new_dev->ctrl_pin); 
        goto err_free_buffer; 
    } 
     pr_debug("[%s] export_store: getting data pins\n", DRIVER_NAME);
     for (i = 0; i < NUM_DATA_PINS; i++) {
         new_dev->data_pins[i] = gpio_to_desc(GPIOCHIP_BASE + pins[i+1]);
         if (IS_ERR(new_dev->data_pins[i])) { 
             ret = PTR_ERR(new_dev->data_pins[i]); 
             pr_err("[%s] Failed to get data pin %d: %d\n", DRIVER_NAME, i, ret);
             goto err_free_buffer;
         }
     }

     // 할당된 핀들을 기록
     for(i = 0; i <= NUM_DATA_PINS; i++) {
        g_assigned_pins[pins[i]] = 1;
     }

     // 핀을 전부 input으로 설정
     pr_debug("[%s] export_store: setting pin directions\n", DRIVER_NAME);
     gpiod_direction_input(new_dev->ctrl_pin);
     for(i=0; i < NUM_DATA_PINS; i++) {
        gpiod_direction_input(new_dev->data_pins[i]);
     }
 
     // 캐릭터 디바이스 초기화 및 등록
     pr_debug("[%s] export_store: registering character device\n", DRIVER_NAME);
     new_dev->devt = MKDEV(g_major_num, dev_idx);
     cdev_init(&new_dev->cdev, &gpio_comm_fops);
     new_dev->cdev.owner = THIS_MODULE;
     ret = cdev_add(&new_dev->cdev, new_dev->devt, 1);
     if (ret) goto err_put_pins;
 
     // sysfs에 디바이스 파일(/dev/`name`) 생성
     pr_debug("[%s] export_store: creating sysfs device file\n", DRIVER_NAME);
     new_dev->device = device_create(g_dev_class, NULL, new_dev->devt, new_dev, new_dev->name);
     if (IS_ERR(new_dev->device)) {
         ret = PTR_ERR(new_dev->device);
         goto err_del_cdev;
     }
 
     // 동기화 객체 초기화
     pr_debug("[%s] export_store: initializing synchronization objects\n", DRIVER_NAME);
     spin_lock_init(&new_dev->lock);
     init_waitqueue_head(&new_dev->wq);
     timer_setup(&new_dev->timeout_timer, comm_timeout_callback, 0);
 
     // 제어핀 IRQ 생성 (read시 활성화)
     pr_debug("[%s] export_store: create IRQ for control pin\n", DRIVER_NAME);
     new_dev->ctrl_irq = gpiod_to_irq(new_dev->ctrl_pin);
     if (new_dev->ctrl_irq < 0) { 
        pr_err("[%s] Failed to get IRQ for control pin: %d\n", DRIVER_NAME, new_dev->ctrl_irq);
        ret = new_dev->ctrl_irq; 
        goto err_destroy_device; 
    }
     ret = request_irq(new_dev->ctrl_irq, ctrl_pin_irq_handler, IRQF_TRIGGER_RISING, "comm_ctrl", new_dev);
     if (ret) {
        pr_err("[%s] Failed to request IRQ for control pin: %d\n", DRIVER_NAME, ret);
        goto err_destroy_device;
     }
     disable_irq(new_dev->ctrl_irq); // read전까진 비활성화
 
     // 4. 모든 설정 완료. 전역 테이블에 등록하고 상태를 IDLE로 설정.
     pr_debug("[%s] export_store: setting device state\n", DRIVER_NAME); 
     new_dev->state = COMM_STATE_IDLE;
     atomic_set(&new_dev->data_ready, 0);
     g_dev_table[dev_idx] = new_dev;
 
     pr_info("[%s] Device '%s' created successfully.\n", DRIVER_NAME, name);
     mutex_unlock(&g_dev_lock);
     return count;
 
 // --- 에러 처리 경로 (goto 문) ---
 // 할당에 실패했을 경우, 이미 할당된 자원들을 역순으로 깨끗하게 해제.
 err_free_irqs:
     pr_debug("[%s] export_store: freeing IRQs\n", DRIVER_NAME);
     free_irq(new_dev->ctrl_irq, new_dev);
 err_destroy_device:
     pr_debug("[%s] export_store: destroying device\n", DRIVER_NAME);
     device_destroy(g_dev_class, new_dev->devt);
 err_del_cdev:
     pr_debug("[%s] export_store: deleting cdev\n", DRIVER_NAME);
     cdev_del(&new_dev->cdev);
 err_put_pins:
     pr_debug("[%s] export_store: putting pins\n", DRIVER_NAME);
     gpiod_put(new_dev->ctrl_pin);
     for (i = 0; i < NUM_DATA_PINS; i++) if(new_dev->data_pins[i]) gpiod_put(new_dev->data_pins[i]);
 err_free_buffer:
     pr_debug("[%s] export_store: freeing buffer\n", DRIVER_NAME);
     kfree(new_dev->rx_buffer);
 err_free_dev:
     pr_debug("[%s] export_store: freeing device\n", DRIVER_NAME);
     kfree(new_dev);
 out_unlock:
     pr_debug("[%s] export_store: unlocking mutex\n", DRIVER_NAME);
     mutex_unlock(&g_dev_lock);
     return ret;
 }
 
 
 static ssize_t unexport_store(const struct class *class, const struct class_attribute *attr, const char *buf, size_t count) {
     pr_info("[%s] unexport_store called, buf: %.*s\n", 
             DRIVER_NAME, (int)min(count, (size_t)50), buf);
             
     if (!buf) {
         pr_err("[%s] ERROR: unexport_store: NULL buffer\n", DRIVER_NAME);
         return -EINVAL;
     }
     char name[MAX_NAME_LEN];
     int i, ret;
     struct gpio_comm_dev *dev = NULL;
 
     sscanf(buf, "%19s", name);
 
     mutex_lock(&g_dev_lock);
     for (i = 0; i < MAX_DEVICES; i++) {
         if (g_dev_table[i] && strcmp(g_dev_table[i]->name, name) == 0) {
             dev = g_dev_table[i];
             g_dev_table[i] = NULL; // 테이블에서 먼저 제거하여 다른 프로세스가 접근하지 못하게 함
             break;
         }
     }
     mutex_unlock(&g_dev_lock);
 
     if (dev) {
         // 찾은 디바이스의 모든 자원 해제
         ret = release_all_resources(dev);
         if (ret != 0) {
            pr_err("[%s] Device '%s' failed to remove.\n", DRIVER_NAME, name);
            return ret;
         }
         pr_info("[%s] Device '%s' removed.\n", DRIVER_NAME, name);
     } else {
         return -ENOENT; // 해당 이름의 디바이스 없음
     }
     return count;
 }
 
 // sysfs에 `export`와 `unexport` 파일을 생성하기 위한 매크로.
 // _WO (Write-Only)는 쓰기만 가능한 파일을 만든다는 의미.
 static CLASS_ATTR_WO(export);
 static CLASS_ATTR_WO(unexport);
 


 // =================================================================
 // 9. 모듈 초기화/종료
 // =================================================================
 
 static int __init gpio_comm_init(void) {
     dev_t devt;
     int ret;
 
     // 1. 캐릭터 디바이스 번호(주, 부번호)를 커널로부터 동적으로 할당받음
     ret = alloc_chrdev_region(&devt, 0, MAX_DEVICES, DRIVER_NAME);
     if (ret < 0) {
         pr_err("[%s] Failed to allocate char dev region\n", DRIVER_NAME);
         return ret;
     }
     g_major_num = MAJOR(devt); // 주 번호 저장
 
     // 2. sysfs에 디바이스 클래스(/sys/class/gpio_comm) 생성
     g_dev_class = class_create(CLASS_NAME);
     if (IS_ERR(g_dev_class)) {
         unregister_chrdev_region(devt, MAX_DEVICES);
         return PTR_ERR(g_dev_class);
     }
 
     // 3. 생성된 클래스 밑에 export/unexport 파일 생성
     class_create_file(g_dev_class, &class_attr_export);
     class_create_file(g_dev_class, &class_attr_unexport);
     
     pr_info("[%s] Driver loaded. Major: %d\n", DRIVER_NAME, g_major_num);
     return 0;
 }
 
 static void __exit gpio_comm_exit(void) {
     int i, ret;
 
     // 모든 생성된 디바이스의 리소스를 해제
     for (i = 0; i < MAX_DEVICES; i++) {
         if (g_dev_table[i]) {
             mutex_lock(&g_dev_lock);
             struct gpio_comm_dev* dev_to_free = g_dev_table[i];
             g_dev_table[i] = NULL;
             mutex_unlock(&g_dev_lock);
             if (release_all_resources(dev_to_free)) {
                pr_err("[%s] Device '%s' failed to remove.\n", DRIVER_NAME, dev_to_free->name);
             }
         }
     }
     
     // sysfs 파일 및 클래스 제거
     class_remove_file(g_dev_class, &class_attr_export);
     class_remove_file(g_dev_class, &class_attr_unexport);
     class_destroy(g_dev_class);
 
     // 할당받았던 디바이스 번호 반납
     unregister_chrdev_region(MKDEV(g_major_num, 0), MAX_DEVICES);
     pr_info("[%s] Driver unloaded.\n", DRIVER_NAME);
 }
 
 // 모듈 진입점/종료점 등록
 module_init(gpio_comm_init);
 module_exit(gpio_comm_exit);
 
 // 모듈 정보
 MODULE_LICENSE("GPL");
 MODULE_AUTHOR("HJH & Gemini");
 MODULE_DESCRIPTION("P2P GPIO Comm Driver v3.0");
 