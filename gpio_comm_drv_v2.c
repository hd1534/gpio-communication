/**
 * @file gpio_comm_drv.c
 * @brief 5-wire GPIO를 이용한 반이중 동기식 통신 드라이버
 * @author HJH (Original), Gemini (Refactored)
 * @version 2.0
 *
 * @note
 * - sysfs를 통해 핀 번호를 받아 초기화합니다.
 * ex) echo "17,27,22,23,24" > /sys/class/gpio_comm/gpio_comm_dev/init
 * - open() 시점에 핸드셰이크 및 클럭 보정을 수행합니다.
 * - read()/write()를 통해 패킷 데이터를 교환합니다.
 */

#include <ctype.h>
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
#include <linux/crc16.h> // CRC-16 사용을 위해 추가




// =================================================================
// 1. 프로토콜 및 드라이버 상수 정의
// =================================================================
#define CLASS_NAME  "gpio_comm"
#define DRIVER_NAME "gpio_comm_drv"


// // 사용할 GPIO 핀 번호 (BCM 기준, 예시)
// #define GPIO_PIN_D0 17 // 데이터 0
// #define GPIO_PIN_D1 27 // 데이터 1
// #define GPIO_PIN_D2 22 // 데이터 2
// #define GPIO_PIN_S_CLK 23 // 슬레이브 클럭 (마스터의 입력)
// #define GPIO_PIN_M_CLK 24 // 마스터 제어/클럭 (마스터의 출력)

#define MAX_NAME_LEN 16
#define MAX_DEV 10
#define NUMBER_OF_DATA_PINS 4
#define MAX_MODE_LEN 16


#define MAX_GPIO_BCM 30

#define RX_BUFFER_SIZE 256
#define TX_BUFFER_SIZE 256

#define TIMEOUT_MS 500 // IRQ 타임아웃 (ms)


// 라즈베리 파이의 GPIO 컨트롤러(BCM2835)의 기본 번호. 커널 내부에서 GPIO를 식별할 때 사용.
// `gpioinfo` 명령어로 실제 base를 확인해야 할 수 있음.
#define GPIOCHIP_BASE 512




// =================================================================
// 2. Enum 정의
// =================================================================


enum connect_mode {
    READ_ONLY,
    READ_WRITE,
};


// 드라이버의 내부 상태
enum comm_state {
    COMM_STATE_UNINITIALIZED, // 초기화 이전
    COMM_STATE_IDLE,          // 유휴 상태 (데이터 교환 가능)
    COMM_STATE_RECEIVING,     // 데이터 수신 중
    COMM_STATE_SENDING,       // 데이터 송신 중
    COMM_STATE_DONE,          // 작업 완료 (read/write 대기 해제용)
    COMM_STATE_ERROR          // 오류 발생
};




// =================================================================
// 3. 구조체 정의
// =================================================================


// 관리하는 각 GPIO 핀의 정보를 담는 구조체.
struct gpio_entry {
    int bcm_num;                  // BCM 핀 번호 (예: 17, 27)
    struct gpio_desc *desc;       // 해당 GPIO를 가리키는 디스크립터 (최신 gpiod 인터페이스의 핵심)
    // struct device *dev;           // 이 GPIO에 할당된 디바이스 구조체 (/dev 및 /sys 파일과 연결됨)
    int irq_num;                  // 이 GPIO에 할당된 인터럽트 번호
    bool irq_enabled;             // 인터럽트가 현재 활성화되었는지 여부 플래그
    struct fasync_struct *async_queue; // 비동기 알림(SIGIO)을 보낼 프로세스 큐
};


// 디바이스 상태를 관리할 통합 구조체
struct gpio_comm_dev {
    char name[MAX_NAME_LEN];
    struct cdev cdev;
    struct device *device;

    // GPIO entry
    struct gpio_entry *ctrl_pin; // 제어 핀
    struct gpio_entry *assigned_pin; // 할당된 핀, 이는 data_pins 중 하나
    struct gpio_entry *data_pins[NUMBER_OF_DATA_PINS]; // 데이터 핀

    // 연결 상태
    enum comm_state state;

    // 클럭 딜레이
    int clock_delay_us;  // 마이크로초 단위

    // 데이터 버퍼
    u8 rx_buffer[RX_BUFFER_SIZE];
    int rx_bit_count;
    int rx_byte_count;

    u8 tx_buffer[TX_BUFFER_SIZE];
    int tx_len;

    // 동기화를 위한 락과 대기 큐
    spinlock_t lock;
    wait_queue_head_t wq;

    // 타임아웃 처리를 위한 커널 타이머
    struct timer_list timeout_timer;
};



// =================================================================
// 4. 전역 변수들
// =================================================================



static struct gpio_comm_dev *gpio_comm_dev_table[MAX_DEV];

static struct gpio_comm_dev *master_dev, *slave_dev;
static dev_t g_master_dev_num, g_slave_dev_num;
static struct class *g_master_dev_class, *g_slave_dev_class;


static int num_dev = 0;
static int used_gpio_bcm[MAX_GPIO_BCM] = {0};



// =================================================================
// 4. 하위 레벨 통신 함수 (실제 GPIO 제어)
// =================================================================


// 4-bit 데이터를 병렬로 setting하는 함수. 전송시 사용
static void gpio_set_4bit(char data_bits) {
    gpiod_set_value(master_dev->datapins[0], data_bits & 0x01);
    gpiod_set_value(master_dev->datapins[1], (data_bits >> 1) & 0x01);
    gpiod_set_value(master_dev->datapins[2], (data_bits >> 2) & 0x01);
    gpiod_set_value(master_dev->datapins[3], (data_bits >> 3) & 0x01);  // master는 slave의 clock을 data 선으로 사용한다.
}

// 4-bit 데이터를 병렬로 reading 하는 함수. 수신시 사용
static char gpio_get_4bit(void) {
    char data_bits = 0;
    data_bits |= gpiod_get_value(master_dev->datapins[0]);
    data_bits |= gpiod_get_value(master_dev->datapins[1]) << 1;
    data_bits |= gpiod_get_value(master_dev->datapins[2]) << 2;
    data_bits |= gpiod_get_value(master_dev->datapins[3]) << 3;
    return data_bits;
}

/**
 * @brief 현재 모드에 맞는 클럭 핀을 토글 (송신 시 사용).
 */
static void toggle_clock(gpio_comm_dev *g_dev) {
    gpiod_set_value(g_dev->ctrl_pin, 0);
    udelay(g_dev->clock_delay_us);
    gpiod_set_value(g_dev->ctrl_pin, 1);
    udelay(g_dev->clock_delay_us);
}

// =================================================================
// 5. Interrupt 및 타이머 핸들러
// TODO:
// =================================================================

/**
 * @brief 타이머 만료 시 호출되는 콜백 함수 (예외 처리).
 */
static void comm_timeout_callback(struct timer_list *t) {
    struct gpio_comm_dev *dev = from_timer(dev, t, timeout_timer);
    unsigned long flags;

    pr_warn("[%s] IRQ Timeout!\n", DRIVER_NAME);

    spin_lock_irqsave(&dev->lock, flags);
    if (dev->state == COMM_STATE_RECEIVING) {
        dev->state = COMM_STATE_ERROR;
        wake_up_interruptible(&dev->wq); // 대기 중인 프로세스를 에러 상태로 깨움
    }
    spin_unlock_irqrestore(&dev->lock, flags);
}

/**
 * @brief 클럭 핀의 Rising Edge에 호출되는 통합 IRQ 핸들러.
 */
static irqreturn_t comm_irq_handler(int irq, void *dev_id) {
    struct gpio_comm_dev *dev = (struct gpio_comm_dev *)dev_id;
    unsigned long flags;
    u8 current_byte;
    u8 received_bits;

    spin_lock_irqsave(&dev->lock, flags);

    // IRQ가 도착했으므로 타임아웃 타이머를 비활성화
    del_timer(&dev->timeout_timer);

    if (dev->state != COMM_STATE_RECEIVING) {
        spin_unlock_irqrestore(&dev->lock, flags);
        return IRQ_NONE; // 수신 대기 상태가 아니면 무시
    }

    // 데이터 핀에서 4-bit 읽기
    received_bits = gpio_get_4bit();

    // 현재 바이트에 비트 결합
    current_byte = dev->rx_buffer[dev->rx_byte_count];
    current_byte |= (received_bits << dev->rx_bit_count);
    dev->rx_buffer[dev->rx_byte_count] = current_byte;
    
    dev->rx_bit_count += 3;

    // 1바이트(8비트 이상)가 완성되면 다음 바이트로 이동
    if (dev->rx_bit_count >= 8) {
        dev->rx_bit_count = 0;
        dev->rx_byte_count++;
    }

    // 전체 패킷 수신이 완료되었는지 확인 (예: 128바이트)
    if (dev->rx_byte_count >= 128) {
        pr_info("[%s] Full packet received.\n", DRIVER_NAME);
        dev->state = COMM_STATE_DONE;
        wake_up_interruptible(&dev->wq); // 수신 완료, read() 대기 해제
    } else {
        // 다음 비트 수신을 위해 타임아웃 타이머를 다시 설정
        mod_timer(&dev->timeout_timer, jiffies + msecs_to_jiffies(TIMEOUT_MS));
    }
    
    spin_unlock_irqrestore(&dev->lock, flags);

    return IRQ_HANDLED;
}



// =================================================================
// 6. file_operations 구현
// =================================================================

/**
 * @brief 디바이스 파일 open 시 호출. 핸드셰이크 및 클럭 보정 수행.
 */
static int gpio_comm_open(struct inode *inode, struct file *filp) {
    if (!g_dev || g_dev->state == COMM_STATE_UNINITIALIZED) {
        pr_err("[%s] Device not initialized. `echo` pins to sysfs first.\n", DRIVER_NAME);
        return -ENODEV;
    }
    
    // open 시점에 이미 다른 프로세스가 사용 중인지 확인 (간단한 구현)
    if (g_dev->state != COMM_STATE_IDLE) {
        pr_warn("[%s] Device is busy.\n", DRIVER_NAME);
        return -EBUSY;
    }

    pr_info("[%s] Device opened. Starting handshake and clock calibration...\n", DRIVER_NAME);
    
    // TODO: README에 명세된 핸드셰이크 및 클럭 보정 로직 구현
    // 이 과정은 복잡하며, 양측이 동기화되어 진행되어야 함.
    // 1. Master/Slave 역할에 맞춰 초기 신호 보내기
    // 2. 상대방 신호 감지 (타임아웃과 함께)
    // 3. 보정 시작: 가장 느린 속도부터 데이터(0~7) 보내고 ACK/NACK 확인
    // 4. 속도 점차 높이며 반복
    // 5. 최적 속도(clock_delay_us) 결정
    // 모든 과정이 성공하면 g_dev->state를 IDLE로 유지, 실패 시 에러 반환
    g_dev->clock_delay_us = 100; // 임시로 100us 설정

    filp->private_data = g_dev;
    pr_info("[%s] Calibration complete. Clock delay set to %d us.\n", DRIVER_NAME, g_dev->clock_delay_us);
    
    return 0;
}

/**
 * @brief write() 호출 시, 데이터를 패킷화하여 전송 상태로 전환.
 */
static ssize_t gpio_comm_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos) {
    struct gpio_comm_dev *dev = filp->private_data;
    unsigned long flags;
    int i;
    u16 crc;

    if (count > 118) { // 페이로드 최대 크기
        pr_warn("[%s] Write size too large. Max payload is 118 bytes.\n", DRIVER_NAME);
        return -EINVAL;
    }
    
    // 1. 패킷 생성 (SOH, Type, Length 등은 생략하고 데이터, CRC만 구현)
    if (copy_from_user(dev->tx_buffer, buf, count)) {
        return -EFAULT;
    }
    dev->tx_len = count;
    
    // 2. CRC-16 계산
    crc = crc16(0, dev->tx_buffer, dev->tx_len);
    dev->tx_buffer[dev->tx_len] = (u8)(crc & 0xFF);
    dev->tx_buffer[dev->tx_len + 1] = (u8)((crc >> 8) & 0xFF);
    dev->tx_len += 2; // CRC 2바이트 추가
    
    pr_info("[%s] Writing %d bytes (payload: %zu, crc: 2).\n", DRIVER_NAME, dev->tx_len, count);
    
    spin_lock_irqsave(&dev->lock, flags);
    dev->state = COMM_STATE_SENDING;
    spin_unlock_irqrestore(&dev->lock, flags);

    // 3. 데이터 전송 (3-bit씩 잘라서)
    for (i = 0; i < dev->tx_len; i++) {
        u8 byte_to_send = dev->tx_buffer[i];
        
        // 3-bit씩 3번에 걸쳐 전송 (마지막은 2-bit)
        set_data_pins(byte_to_send & 0x07);
        toggle_clock();
        
        set_data_pins((byte_to_send >> 3) & 0x07);
        toggle_clock();
        
        set_data_pins((byte_to_send >> 6) & 0x03);
        toggle_clock();
    }
    
    pr_info("[%s] Write complete.\n", DRIVER_NAME);

    spin_lock_irqsave(&dev->lock, flags);
    dev->state = COMM_STATE_IDLE; // 전송 후 IDLE 상태로 복귀
    spin_unlock_irqrestore(&dev->lock, flags);
    
    return count; // 실제 페이로드 크기 반환
}

/**
 * @brief read() 호출 시, 데이터 수신이 완료될 때까지 대기.
 */
static ssize_t gpio_comm_read(struct file *filp, char __user *buf, size_t len, loff_t *off) {
    struct gpio_comm_dev *dev = filp->private_data;
    unsigned long flags;
    int ret;
    int bytes_to_copy;
    u16 received_crc, calculated_crc;

    pr_info("[%s] Waiting for data...\n", DRIVER_NAME);
    
    // 1. 수신 상태로 전환하고 타임아웃 설정
    spin_lock_irqsave(&dev->lock, flags);
    dev->state = COMM_STATE_RECEIVING;
    dev->rx_byte_count = 0;
    dev->rx_bit_count = 0;
    memset(dev->rx_buffer, 0, RX_BUFFER_SIZE);
    // 첫 IRQ를 기다리기 위해 타이머 설정
    mod_timer(&dev->timeout_timer, jiffies + msecs_to_jiffies(TIMEOUT_MS * 5)); // 첫 대기는 길게
    spin_unlock_irqrestore(&dev->lock, flags);

    // 2. 데이터가 준비되거나 에러가 발생할 때까지 대기
    ret = wait_event_interruptible(dev->wq, dev->state == COMM_STATE_DONE || dev->state == COMM_STATE_ERROR);
    if (ret) {
        return -ERESTARTSYS; // 시그널에 의해 중단
    }

    if (dev->state == COMM_STATE_ERROR) {
        pr_err("[%s] Read failed due to timeout or error.\n", DRIVER_NAME);
        return -ETIMEDOUT;
    }
    
    // 3. 수신된 데이터의 CRC 검증
    bytes_to_copy = dev->rx_byte_count - 2;
    if (bytes_to_copy < 0) return -EIO; // CRC보다 적게 받음

    received_crc = (dev->rx_buffer[bytes_to_copy+1] << 8) | dev->rx_buffer[bytes_to_copy];
    calculated_crc = crc16(0, dev->rx_buffer, bytes_to_copy);
    
    if (received_crc != calculated_crc) {
        pr_err("[%s] CRC mismatch! Received: 0x%04x, Calculated: 0x%04x\n", DRIVER_NAME, received_crc, calculated_crc);
        // TODO: NACK 전송 로직 필요
        return -EBADMSG; // Bad Message
    }
    pr_info("[%s] CRC check passed.\n", DRIVER_NAME);
    
    // 4. 유저 공간으로 데이터 복사
    if (len < bytes_to_copy) bytes_to_copy = len;
    if (copy_to_user(buf, dev->rx_buffer, bytes_to_copy)) {
        return -EFAULT;
    }
    
    // 5. 상태를 다시 IDLE로 변경
    spin_lock_irqsave(&dev->lock, flags);
    dev->state = COMM_STATE_IDLE;
    spin_unlock_irqrestore(&dev->lock, flags);
    
    return bytes_to_copy;
}

static int gpio_comm_release(struct inode *inode, struct file *filp) {
    pr_info("[%s] Device released.\n", DRIVER_NAME);
    // 필요 시, 연결 종료 신호 전송
    return 0;
}

static const struct file_operations gpio_comm_fops = {
    .owner = THIS_MODULE,
    .open = gpio_comm_open,
    .write = gpio_comm_write,
    .read = gpio_comm_read,
    .release = gpio_comm_release,
};



// =================================================================
// 7. gpio 할당
// =================================================================


static ssize_t assign_gpio(int bcm, struct gpio_entry **entry_out) {
    int kernel_gpio;
    struct gpio_entry *entry;

    // 이미 export된 핀인지 확인
    if (used_gpio_bcm[bcm])
        return -EEXIST;

    // 새로운 gpio_entry를 위한 메모리를 할당하고 0으로 초기화
    entry = kzalloc(sizeof(*entry), GFP_KERNEL);
    if (!entry) return -ENOMEM;

    // BCM 번호를 커널 내부의 GPIO 번호로 변환
    kernel_gpio = GPIOCHIP_BASE + bcm;
    entry->bcm_num = bcm;
    // 커널 GPIO 번호로 GPIO 디스크립터를 얻어옴
    entry->desc = gpio_to_desc(kernel_gpio);
    if (!entry->desc) { // 유효하지 않은 GPIO 번호일 경우
        kfree(entry);
        return -EINVAL;
    }

    // 기본적으로 입력 모드로 설정
    gpiod_direction_input(entry->desc);

    pr_info("[%s] Assigned GPIO %d\n", DRIVER_NAME, bcm);

    *entry_out = entry;

    return count;
}



// =================================================================
// 7. class_operations 구현
// =================================================================


// `echo 17 > /sys/class/gpio_comm/connect` 시 호출
static ssize_t connect_store(const struct class *class, const struct class_attribute *attr, const char *buf, size_t count) {
    int ret;
    int bcm[NUMBER_OF_DATA_PINS + 1]; // 0번이 ctrl
    struct gpio_comm_dev *new_dev;
    char name[MAX_NAME_LEN], mode[MAX_MODE_LEN];
    enum connect_mode connect_mode;

    if (num_dev >= MAX_DEV) {
        pr_err("[%s] Maximum number of devices reached.\n", DRIVER_NAME);
        return -ENOSPC;
    }


    // 0. 값 읽기
    if (sscanf(buf, "%s,%s,%d,%d,%d,%d,%d", name, mode, bcm, bcm+1, bcm+2, bcm+3, bcm+4) != 6) {
        return -EINVAL;
    }


    // 1. 값 검증

    // 이미 사용 중인 핀인지 확인
    for (int i = 0; i <= NUMBER_OF_DATA_PINS; i++) {
        if (used_gpio_bcm[bcm[i]]) {
            pr_err("[%s] GPIO pin %d already in use.\n", DRIVER_NAME, bcm[i]);
            return -EEXIST;
        }
    }
    for (int i = 0; i <= NUMBER_OF_DATA_PINS; i++) {
        used_gpio_bcm[bcm[i]] = 1;
    }

    // 모드 검증
    if (strcmp(tolower(mode), "r") == 0) {
        connect_mode = READ_ONLY;
    } else if (strcmp(tolower(mode), "rw") == 0) {
        connect_mode = READ_WRITE;
    } else {
        pr_err("[%s] Invalid mode: %s\n", DRIVER_NAME, mode);
        return -EINVAL;
    }

    // 이미 초기화된 디바이스인지 확인
    for (int i = 0; i < MAX_DEV; i++) {
        if (gpio_comm_table[i] && strcmp(gpio_comm_table[i]->name, name) == 0) {
            pr_warn("[%s] Device %s already initialized.\n", DRIVER_NAME, name);
            return -EEXIST;
        }
    }

    // 2. 디바이스 구조체 메모리 할당
    new_dev = kzalloc(sizeof(struct gpio_comm_dev), GFP_KERNEL);
    if (!new_dev) return -ENOMEM;
    new_dev->mode = mode; // 모듈 파라미터로 받은 모드 설정

    // 3. GPIO 핀 할당
    if (assign_gpio(bcm[0], &new_dev->ctrl) < 0) return -EINVAL;
    if (assign_gpio(bcm[1], &new_dev->datapins[0]) < 0) return -EINVAL;
    if (assign_gpio(bcm[2], &new_dev->datapins[1]) < 0) return -EINVAL;
    if (assign_gpio(bcm[3], &new_dev->datapins[2]) < 0) return -EINVAL;
    if (assign_gpio(bcm[4], &new_dev->datapins[3]) < 0) return -EINVAL;

    // 4. IRQ 등록
    new_dev->irq = gpiod_to_irq(irq_pin_desc);
    if (new_dev->irq < 0) {
        pr_err("[%s] Failed to get IRQ from pin.\n", DRIVER_NAME);
        // (리소스 해제 로직 필요)
        return new_dev->irq;
    }
    ret = request_irq(new_dev->irq, comm_irq_handler, IRQF_TRIGGER_RISING, DRIVER_NAME, new_dev);
    if (ret) {
        pr_err("[%s] Failed to request IRQ.\n", DRIVER_NAME);
        // (리소스 해제 로직 필요)
        return ret;
    }
    disable_irq(new_dev->irq); // 우선 비활성화 해두고, 필요할 때 활성화

    // 5. 기타 멤버 초기화
    spin_lock_init(&new_dev->lock);
    init_waitqueue_head(&new_dev->wq);
    timer_setup(&new_dev->timeout_timer, comm_timeout_callback, 0);
    new_dev->state = COMM_STATE_IDLE; // 초기화 완료 후 IDLE 상태로


    if (connect_mode == READ_WRITE) {
        // TODO: 모든 핀을 확인해서 idle 상태인거 확인하고, d0~d3중에 비어있는거 하나에 접근
    }


    // 생성된 디바이스에 private 데이터로 entry를 저장
    dev_set_drvdata(entry->dev, entry);

    // sysfs에 value와 direction 속성 파일을 생성
    device_create_file(entry->dev, &name);


    // 빈 공간에 집어넣기
    for (int i = 0; i < MAX_DEV; i++) {
        if (!gpio_comm_table[i]) {
            gpio_comm_table[i] = new_dev;
            break;
        }
    }

    pr_info("[%s] %s device initialized with pins M_CTRL: %d; D0-4: %d,%d,%d,%d;\n",
        DRIVER_NAME, name, bcm[0], bcm[1], bcm[2], bcm[3], bcm[4]);

    return count;
}

// `echo 17 > /sys/class/gpio_comm/unexport` 시 호출
static ssize_t disconnect_store(const struct class *class, const struct class_attribute *attr, const char *buf, size_t count) {
    int idx;
    char name[MAX_NAME_LEN];

    // 0. 값 읽기
    if (sscanf(buf, "%s", name) != 1) {
        return -EINVAL;
    }

    // 1. 값 검증
    // 있는지 확인
    for (idx = 0; idx < MAX_DEV; idx++) {
        if (gpio_comm_table[idx] && strcmp(gpio_comm_table[idx]->name, name) == 0) {
            break;
        }
    }
    if (idx == MAX_DEV) return -ENOENT; // 해당 항목 없음

    // export_store에서 했던 작업들을 역순으로 수행
    device_remove_file(gpio_comm_table[idx]->dev, &name);
    device_destroy(gpiod_class, MKDEV(major_num, idx));
    kfree(gpio_comm_table[idx]);
    gpio_comm_table[idx] = NULL;

    pr_info("[%s] Unexported GPIO %d\n", DRIVER_NAME, bcm);
    return count;
}

// export와 unexport 클래스 속성을 쓰기 전용(Write-Only)으로 정의하는 매크로.
static CLASS_ATTR_WO(connect);
static CLASS_ATTR_WO(disconnect);


/**
 * @brief 드라이버 로드 시 호출
 */
static int __init gpio_comm_init(void) {
    int ret;
    pr_info("[%s] module loading (mode: %s)\n", DRIVER_NAME, (mode == COMM_MODE_MASTER) ? "Master" : "Slave");

    // 캐릭터 디바이스 번호 할당
    ret = alloc_chrdev_region(&g_master_dev_num, 0, 1, DRIVER_NAME);
    if (ret < 0) return ret;

    // 클래스 생성
    g_dev_class = class_create(CLASS_NAME);
    if (IS_ERR(g_dev_class)) {
        unregister_chrdev_region(g_master_dev_num, 1);
        return PTR_ERR(g_dev_class);
    }
    
    // 디바이스 파일 생성 및 sysfs 속성 추가
    g_dev = kzalloc(sizeof(struct gpio_comm_dev), GFP_KERNEL); // 임시 할당
    if (!g_dev) { /* ... */ }
    g_dev->device = device_create(g_dev_class, NULL, g_master_dev_num, NULL, DEVICE_NAME);
    if (IS_ERR(g_dev->device)) { /* ... */ }
    ret = device_create_file(g_dev->device, &dev_attr_init);
    if (ret) { /* ... */ }

    // cdev 등록
    cdev_init(&g_dev->cdev, &gpio_comm_fops);
    ret = cdev_add(&g_dev->cdev, g_master_dev_num, 1);
    if (ret < 0) { /* ... */ }

    pr_info("[%s] sysfs entry created at /sys/class/%s/%s\n", DRIVER_NAME, CLASS_NAME, DEVICE_NAME);
    return 0;
}

/**
 * @brief 드라이버 언로드 시 호출
 */
static void __exit gpio_comm_exit(void) {
    if (g_dev) {
        if (g_dev->irq > 0) {
            free_irq(g_dev->irq, g_dev);
        }
        del_timer_sync(&g_dev->timeout_timer);
        gpiod_put(g_dev->d0);
        gpiod_put(g_dev->d1);
        gpiod_put(g_dev->d2);
        gpiod_put(g_dev->s_clk);
        gpiod_put(g_dev->ctrl);
        kfree(g_dev);
    }
    
    device_remove_file(g_dev->device, &dev_attr_init);
    device_destroy(g_dev_class, g_master_dev_num);
    class_destroy(g_dev_class);
    cdev_del(&g_dev->cdev);
    unregister_chrdev_region(g_master_dev_num, 1);
    
    pr_info("[%s] module unloaded\n", DRIVER_NAME);
}

module_init(gpio_comm_init);
module_exit(gpio_comm_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("HJH & Gemini");
MODULE_DESCRIPTION("GPIO Communication Protocol Driver v2.0");