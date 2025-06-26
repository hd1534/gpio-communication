#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/gpio/consumer.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/timer.h>


// =================================================================
// 1. 프로토콜 상수 정의
// =================================================================
#define DRIVER_NAME "gpio_comm"
#define CLASS_NAME  "gpio_comm_class"

// // 사용할 GPIO 핀 번호 (BCM 기준, 예시)
// #define GPIO_PIN_D0 17 // 데이터 0
// #define GPIO_PIN_D1 27 // 데이터 1
// #define GPIO_PIN_D2 22 // 데이터 2
// #define GPIO_PIN_S_CLK 23 // 슬레이브 클럭 (마스터의 입력)
// #define GPIO_PIN_M_CLK 24 // 마스터 제어/클럭 (마스터의 출력)

// 패킷 구조 관련
#define PACKET_SIZE 128
#define HEADER_SIZE 8
#define CRC_SIZE 2
#define PAYLOAD_SIZE (PACKET_SIZE - HEADER_SIZE - CRC_SIZE)

#define BUFFER_SIZE 256

#define DEFUALT_CLOCK_DELAY_US 1000 * 1000  // 약 1초



// =================================================================
// 2.enum 정의
// =================================================================

// 통신 모드를 나타내는 enum
enum comm_mode {
    COMM_MODE_MASTER,
    COMM_MODE_SLAVE
};

// 통신 상태를 나타내는 enum
enum comm_state {
    COMM_LISTENING,
    COMM_SENDING,
    COMM_RECEIVING,
    COMM_DONE
};



// =================================================================
// 3. 구조체 정의
// =================================================================

// 디바이스 상태를 관리할 구조체
struct gpio_comm_dev {
    struct cdev cdev;
    dev_t dev_num;
    struct class *dev_class;

    // GPIO 디스크립터
    struct gpio_desc *d0, *d1, *d2; // 데이터 핀
    struct gpio_desc *s_clk;    // 슬레이브 클럭 핀
    struct gpio_desc *m_clk;    // 마스터 제어/클럭 핀

    // IRQ 번호
    int clk_irq;

    // 통신 상태
    enum comm_state state;
    int clock_delay_us;
    
    // 데이터 수신을 위한 버퍼 및 카운터
    char rx_buffer[BUFFER_SIZE];
    char *rx_buffer_ptr;

    // 유저 공간과의 동기화를 위한 대기 큐
    wait_queue_head_t wq;

    // 타임아웃 처리를 위한 커널 타이머
    struct timer_list irq_timeout_timer;
};

static struct gpio_comm_dev *master_dev, *slave_dev;




// =================================================================
// 4. 하위 레벨 통신 함수 (실제 GPIO 제어)
// =================================================================

// 3-bit 데이터를 병렬로 setting하는 함수. slave -> master시 사용
static void gpio_set_3bit(char data_bits) {
    gpiod_set_value(slave_dev->d0, data_bits & 0x01);
    gpiod_set_value(slave_dev->d1, (data_bits >> 1) & 0x01);
    gpiod_set_value(slave_dev->d2, (data_bits >> 2) & 0x01);
}

// 4-bit 데이터를 병렬로 setting하는 함수. master -> slave시 사용
static void gpio_set_4bit(char data_bits) {
    gpiod_set_value(master_dev->d0, data_bits & 0x01);
    gpiod_set_value(master_dev->d1, (data_bits >> 1) & 0x01);
    gpiod_set_value(master_dev->d2, (data_bits >> 2) & 0x01);
    gpiod_set_value(master_dev->s_clk, (data_bits >> 3) & 0x01);  // master는 slave의 clock을 data 선으로 사용한다.
}

// 3-bit 데이터를 병렬로 reading 하는 함수. slave -> master시 사용
static char gpio_get_3bit(void) {
    char data_bits = 0;
    data_bits |= gpiod_get_value(slave_dev->d0);
    data_bits |= gpiod_get_value(slave_dev->d1) << 1;
    data_bits |= gpiod_get_value(slave_dev->d2) << 2;
    return data_bits;
}   

// 4-bit 데이터를 병렬로 reading 하는 함수. master -> slave시 사용
static char gpio_get_4bit(void) {
    char data_bits = 0;
    data_bits |= gpiod_get_value(master_dev->d0);
    data_bits |= gpiod_get_value(master_dev->d1) << 1;
    data_bits |= gpiod_get_value(master_dev->d2) << 2;
    data_bits |= gpiod_get_value(master_dev->s_clk) << 3;
    return data_bits;
}

// 1바이트를 전송하는 함수
static int gpio_comm_send_byte(u8 byte, struct gpio_comm_dev *dev) {
    if(dev->mode == COMM_MODE_MASTER) {  // 4bit식
        // falling edge
        gpiod_set_value(dev->m_clk, 0);
        udelay(dev->clock_delay_us);

        // send upper 4 bits
        gpio_set_4bit(byte >> 4);

        // rising edge
        gpiod_set_value(dev->m_clk, 1);
        udelay(dev->clock_delay_us);

        // falling edge
        gpiod_set_value(dev->m_clk, 0);
        udelay(dev->clock_delay_us);

        // send lower 4 bits
        gpio_set_4bit(byte & 0b1111);

        // rising edge
        gpiod_set_value(dev->m_clk, 1);
        udelay(dev->clock_delay_us);
    } 
    else {
        // falling edge
        gpiod_set_value(dev->s_clk, 0);
        udelay(dev->clock_delay_us);

        // send upper 3 bits
        gpio_set_3bit(byte >> 5);

        // rising edge
        gpiod_set_value(dev->s_clk, 1);
        udelay(dev->clock_delay_us);

        // falling edge
        gpiod_set_value(dev->s_clk, 0);
        udelay(dev->clock_delay_us);

        // send middle 3 bits
        gpio_set_3bit((byte >> 2) & 0b0111);

        // rising edge
        gpiod_set_value(dev->s_clk, 1);
        udelay(dev->clock_delay_us);

        // falling edge
        gpiod_set_value(dev->s_clk, 0);
        udelay(dev->clock_delay_us);    

        // send lower 2 bits
        gpio_set_3bit(byte & 0b0011);

        // rising edge
        gpiod_set_value(dev->s_clk, 1);
        udelay(dev->clock_delay_us);
    }
    return 0;
}

// 1바이트를 수신하는 함수
// TODO: clock에 맞추어 irq로 되도록 수정해야함.
static int gpio_comm_receive_byte(u8 *byte, struct gpio_comm_dev *dev) {
    if(dev->mode == COMM_MODE_MASTER) {  // 4bit식
        *byte = gpio_get_4bit() << 4 | gpio_get_4bit();
    }        
    else {
        *byte = gpio_get_3bit() << 5 | gpio_get_3bit() << 2 | gpio_get_3bit();
    }
    return 0;
}   



// =================================================================
// 5. Interrupt 핸들러
// =================================================================


// s_clk 하강 엣지 핸들러 (통신 시작용)
static irqreturn_t comm_start_irq_handler(int irq, void *dev_id) {
    struct gpio_comm_dev *dev = dev_id;
    
    if (dev->state != COMM_IDLE) return IRQ_NONE;

    // 1. 상태를 RECEIVING으로 변경
    dev->state = COMM_RECEIVING;
    dev->bit_count = 0;
    // memset(dev->rx_buffer, 0, sizeof(dev->rx_buffer)); // 버퍼 초기화

    // 2. 이 IRQ(하강 엣지)는 비활성화하고, 상승 엣지 IRQ로 교체
    // 중요: 핸들러 내에서는 _nosync 버전을 사용
    disable_irq_nosync(irq); 

    // comm_receive_data_irq_handler를 rising edge일때 호출하도록 추가

    
    // 이제 상승 엣지 IRQ를 기다리기 시작하므로, 타임아웃 타이머를 설정
    // 예: 500ms 후 만료
    mod_timer(&dev->irq_timeout_timer, jiffies + msecs_to_jiffies(500));
    

    pr_info("Communication Started.\n");

    return IRQ_HANDLED;
}

// s_clk rising edge handler
static irqreturn_t comm_receive_data_irq_handler(int irq, void *dev_id) {
    struct gpio_comm_dev *dev = dev_id;

    // *** 가장 중요한 부분 ***
    // IRQ가 제시간에 도착했으므로 타임아웃 타이머를 삭제하여
    // 콜백 함수가 실행되지 않도록 함.
    // del_timer()는 타이머가 활성화되어 있지 않아도 안전하게 호출 가능.
    del_timer(&dev->irq_timeout_timer);

    
    if (dev->state != COMM_RECEIVING) return IRQ_NONE;

    // 1. 데이터를 수신하고 버퍼에 저장
    // 2. bit_count를 증가
    // 3. bit_count가 8이 되면 상태를 RECEIVING으로 변경
    // 4. 이 IRQ(상승 엣지)는 비활성화하고, 하강 엣지 IRQ로 교체
    // 중요: 핸들러 내에서는 _nosync 버전을 사용
    disable_irq_nosync(irq); 
    // irq_set_irq_type()으로 트리거 타입을 하강 엣지로 변경
    irq_set_irq_type(irq, IRQ_TYPE_EDGE_FALLING);
    // IRQ 다시 활성화
    enable_irq(irq);

    pr_info("Data Received: %c\n", dev->rx_buffer[dev->bit_count]);

    // ... 기존의 데이터 읽기 및 처리 로직 ...
    // if (통신이 완료되었다면) {
    //     data_complete = 1;
    // }

    if (data_complete) {
        // 통신이 성공적으로 끝났을 때의 처리
        dev->state = COMM_DONE;
        wake_up_interruptible(&dev->wq);
    } else {
        // 아직 데이터가 더 남았다면, 다음 IRQ를 위해 타이머를 다시 설정
        mod_timer(&dev->irq_timeout_timer, jiffies + msecs_to_jiffies(500));
    }

    pr_info("Waiting for data IRQ, timeout set to 500ms\n");

    return IRQ_HANDLED;
}

// 타이머 콜백 함수
static void comm_timeout_callback(struct timer_list *t) {
    // from_timer 매크로를 사용해 timer_list 구조체에서
    // 부모 구조체(gpio_comm_dev)의 포인터를 얻어옴
    struct gpio_comm_dev *dev = from_timer(dev, t, irq_timeout_timer);

    pr_warn("IRQ Timeout! No response from the device.\n");

    // 예외 처리 로직:
    // 1. 상태를 IDLE로 되돌려놓음
    dev->state = COMM_IDLE;
    // 2. 혹시 모를 IRQ를 비활성화 (필요 시)
    //    이 시점에 IRQ가 발생하면 꼬일 수 있으므로 락(lock) 사용이 권장됨 (아래 참조)
    disable_irq(dev->s_clk_irq); 
    
    // 3. read()에서 대기 중인 프로세스가 있다면 에러를 반환하며 깨움
    //    이 경우, read()는 -ETIMEDOUT 같은 에러를 반환하게 처리할 수 있음
    dev->state = COMM_ERROR; // 예: 에러 상태 추가
    wake_up_interruptible(&dev->wq);
}


// =================================================================
// 6. file_operations 구현
// =================================================================

static int gpio_comm_open(struct inode *inode, struct file *filp) {
    pr_info("[%s] device open\n", DRIVER_NAME);
    
    // TODO:
    // 1. 모든 핀의 direction을 'in'으로 초기화
    // 2. 모든 핀의 value가 low인걸 일정시간 확인. 아니면 error
    // 3. Master 제어핀(m_clk)을 'out', 'high'로 설정


    // TODO: 따로 함수로 빼는게 좋을듯?
    // 4. 클럭 속도 보정(Handshake & Calibration) 로직 수행
    //    - gpio_comm_send_byte, gpio_comm_wait_for_ack 등을 사용하여 구현
    //    - 결정된 clock_delay_us를 my_dev->clock_delay_us에 저장
    
    return 0;
}

static ssize_t gpio_comm_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos) {
    char *k_buf;
    size_t total_sent = 0;
    
    k_buf = kmalloc(count, GFP_KERNEL);
    if (!k_buf) return -ENOMEM;
    
    if (copy_from_user(k_buf, buf, count)) {
        kfree(k_buf);
        return -EFAULT;
    }
    
    pr_info("[%s] write %zu bytes\n", DRIVER_NAME, count);

    // TODO:
    //    a. 헤더 생성 (데이터의 크기)
    //    b. 데이터와 CRC 추가
    //    c. gpio_comm_send_byte 함수를 반복 호출하여 패킷 전송

    
    kfree(k_buf);
    return total_sent; // 실제 전송된 바이트 수 반환
}

// read() 함수 예시
static ssize_t comm_read(struct file *filp, char __user *buf, size_t len, loff_t *off) {
    struct gpio_comm_dev *dev = filp->private_data;

    // 데이터가 준비될 때까지 대기 (프로세스는 잠듦)
    // wait_event_interruptible(대기큐, 조건);
    // 조건: dev->state == COMM_DONE
    if (wait_event_interruptible(dev->wq, dev->state == COMM_DONE)) {
        return -ERESTARTSYS; // 시그널에 의해 중단됨
    }
    
    // 데이터 복사...
    // copy_to_user(...)

    // 상태를 다시 IDLE로 변경하여 다음 통신 준비
    dev->state = COMM_IDLE;
    
    return bytes_copied;
}

static int gpio_comm_release(struct inode *inode, struct file *filp) {
    pr_info("[%s] device release\n", DRIVER_NAME);

    // TODO:
    return 0;
}

// file_operations 구조체 정의
static const struct file_operations gpio_comm_fops = {
    .owner = THIS_MODULE,
    .open = gpio_comm_open,
    .write = gpio_comm_write,
    .read = gpio_comm_read,
    .release = gpio_comm_release,
};



// =================================================================
// 7. 모듈 초기화 및 종료
// =================================================================


// Forward declarations
static int gpio_master_init_pins(int d0, int d1, int d2, int s_clk, int m_clk);
static int gpio_slave_init_pins(int d0, int d1, int d2, int s_clk, int m_clk);

// Sysfs `init` attribute handler
static ssize_t init_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    int d0, d1, d2, s_clk, m_clk;
    int ret;

    if (sscanf(buf, "%d, %d, %d, %d, %d", &d0, &d1, &d2, &s_clk, &m_clk) != 5) {
        pr_err("[%s] Invalid arguments. Expected 5 pin numbers.\n", DRIVER_NAME);
        return -EINVAL;
    }

    // Check if we are initializing master or slave
    if (strcmp(dev_name(dev), "master") == 0) {
        pr_info("[%s] Initializing Master with pins D0-2: %d,%d,%d; S_CLK: %d; M_CTRL: %d\n",
                DRIVER_NAME, d0, d1, d2, s_clk, m_clk);
        ret = gpio_master_init_pins(d0, d1, d2, s_clk, m_clk);
    } else if (strcmp(dev_name(dev), "slave") == 0) {
        pr_info("[%s] Initializing Slave with pins D0-2: %d,%d,%d; S_CLK: %d; M_CTRL: %d\n",
                DRIVER_NAME, d0, d1, d2, s_clk, m_clk);
        ret = gpio_slave_init_pins(d0, d1, d2, s_clk, m_clk);
    } else {
        pr_err("[%s] Unknown device trying to init: %s\n", DRIVER_NAME, dev_name(dev));
        return -EINVAL;
    }

    if (ret) {
        pr_err("[%s] Failed to initialize pins.\n", DRIVER_NAME);
        return ret;
    }

    return count;
}

// TODO: Implement the full state machine, workqueues, timers, and IRQ handlers
// The following are placeholders for the full logic described in the spec.
int gpio_master_init_pins(int d0, int d1, int d2, int s_clk, int m_clk) {
    // 1. gpiod_get for all 5 pins
    // 2. Set directions: M_CTRL=OUT, others=IN
    // 3. Set M_CTRL to HIGH
    // 4. Setup IRQ for S_CLK to detect slave connection (rising edge)
    // 5. In IRQ handler, start the handshake workqueue
    pr_info("[%s] Master Initialized and waiting for slave.\n", DRIVER_NAME);
    return 0;
}

int gpio_slave_init_pins(int d0, int d1, int d2, int s_clk, int m_clk) {
    // 1. gpiod_get for all 5 pins
    // 2. Set directions: M_CTRL=IN, others=OUT
    // 3. Set S_CLK=HIGH, D0-2=LOW
    // 4. Setup IRQ for M_CTRL to detect master's response
    // 5. In IRQ handler, start the handshake workqueue
    pr_info("[%s] Slave Initialized and signaling master.\n", DRIVER_NAME);
    return 0;
}

// Create device attributes
static DEVICE_ATTR(init, S_IWUSR, NULL, init_store);

// Devices
static struct device *master_dev;
static struct device *slave_dev;
static struct class *gpiocom_class;

// file_operations for the character device
// TODO: Implement read/write logic based on the state machine
static ssize_t gpiocom_read(struct file *filp, char __user *buf, size_t len, loff_t *off) {
    pr_info("gpiocom_read called\n");
    // Use wait_queue to block until data is received from the other device
    return 0;
}

static ssize_t gpiocom_write(struct file *filp, const char __user *buf, size_t len, loff_t *off) {
    pr_info("gpiocom_write called\n");
    // Packetize data and trigger the transmission state machine
    return len;
}

static const struct file_operations gpiocom_fops = {
    .owner = THIS_MODULE,
    .read = gpiocom_read,
    .write = gpiocom_write,
};

// Character device setup
#define MAX_DEVICES 2
static dev_t dev_num_base;
static struct cdev gpiocom_cdev;

static int __init gpiocom_driver_init(void) {
    int ret;
    pr_info("[%s] module loading\n", DRIVER_NAME);

    // 1. Create class
    gpiocom_class = class_create(DRIVER_NAME);
    if (IS_ERR(gpiocom_class)) {
        pr_err("Failed to create class\n");
        return PTR_ERR(gpiocom_class);
    }

    // 2. Create master device and its 'init' file
    master_dev = device_create(gpiocom_class, NULL, MKDEV(0, 0), NULL, "master");
    if (IS_ERR(master_dev)) {
        class_destroy(gpiocom_class);
        return PTR_ERR(master_dev);
    }
    ret = device_create_file(master_dev, &dev_attr_init);
    if (ret) {
        device_destroy(gpiocom_class, MKDEV(0, 0));
        class_destroy(gpiocom_class);
        return ret;
    }

    // 3. Create slave device and its 'init' file
    slave_dev = device_create(gpiocom_class, NULL, MKDEV(0, 1), NULL, "slave");
    // ... (error handling) ...
    ret = device_create_file(slave_dev, &dev_attr_init);
    // ... (error handling) ...

    // 4. Allocate and register character device for data I/O
    ret = alloc_chrdev_region(&dev_num_base, 0, MAX_DEVICES, DRIVER_NAME);
    // ... (error handling) ...
    cdev_init(&gpiocom_cdev, &gpiocom_fops);
    ret = cdev_add(&gpiocom_cdev, dev_num_base, MAX_DEVICES);
    // ... (error handling) ...

    pr_info("[%s] sysfs devices created at /sys/class/%s/\n", DRIVER_NAME, CLASS_NAME);
    return 0;
}

static void __exit gpiocom_driver_exit(void) {
    device_remove_file(slave_dev, &dev_attr_init);
    device_destroy(gpiocom_class, MKDEV(0, 1));
    device_remove_file(master_dev, &dev_attr_init);
    device_destroy(gpiocom_class, MKDEV(0, 0));
    class_destroy(gpiocom_class);
    cdev_del(&gpiocom_cdev);
    unregister_chrdev_region(dev_num_base, MAX_DEVICES);
    pr_info("[%s] module unloaded\n", DRIVER_NAME);
}

module_init(gpiocom_driver_init);
module_exit(gpiocom_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("HJH");
MODULE_DESCRIPTION("GPIO Communication Protocol Driver");