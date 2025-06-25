#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/gpio/consumer.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/delay.h>

// 1. 프로토콜 상수 정의
#define DRIVER_NAME "gpio_comm"
#define CLASS_NAME  "gpio_comm_class"

// 사용할 GPIO 핀 번호 (BCM 기준, 예시)
#define GPIO_PIN_D0 17 // 데이터 0
#define GPIO_PIN_D1 27 // 데이터 1
#define GPIO_PIN_D2 22 // 데이터 2
#define GPIO_PIN_S_CLK 23 // 슬레이브 클럭 (마스터의 입력)
#define GPIO_PIN_M_CLK 24 // 마스터 제어/클럭 (마스터의 출력)

// 패킷 구조 관련
#define PACKET_SIZE 128
#define HEADER_SIZE 8
#define CRC_SIZE 2

// 2. 디바이스 상태를 관리할 구조체
struct gpio_comm_dev {
    struct cdev cdev;
    dev_t dev_num;
    struct class *dev_class;
    
    // GPIO 디스크립터
    struct gpio_desc *d0, *d1, *d2; // 데이터 핀
    struct gpio_desc *s_clk;       // 슬레이브 클럭 핀
    struct gpio_desc *m_clk;       // 마스터 제어/클럭 핀
    
    // 통신 상태
    int clock_delay_us; // 보정된 클럭 딜레이 (마이크로초)
    // TODO: 통신 상태 (IDLE, SENDING, WAITING_ACK 등)를 나타내는 enum 추가 가능
};

static struct gpio_comm_dev *my_dev;

// =================================================================
// 3. 하위 레벨 통신 함수 (실제 GPIO 제어)
// =================================================================

// 3-bit 데이터를 병렬로 출력하는 함수
static void gpio_comm_write_parallel(char data_bits) {
    gpiod_set_value(my_dev->d0, data_bits & 0x01);
    gpiod_set_value(my_dev->d1, (data_bits >> 1) & 0x01);
    gpiod_set_value(my_dev->d2, (data_bits >> 2) & 0x01);
}

// 1바이트를 전송하는 함수 (Master -> Slave)
static int gpio_comm_send_byte(u8 byte) {
    int i;
    for (i = 0; i < 8; i++) { // 8-bit 데이터를 3-bit/3-bit/2-bit로 쪼개 보낼 수도 있고, 한 비트씩 보낼 수도 있음. 여기서는 1비트씩 예시
        // 데이터 비트 설정 (실제로는 병렬로 3비트씩 보내야 함)
        // ... gpio_comm_write_parallel(...) 호출 ...

        // 클럭 펄스 (Rising Edge)
        gpiod_set_value(my_dev->m_clk, 0);
        udelay(my_dev->clock_delay_us);
        gpiod_set_value(my_dev->m_clk, 1);
        udelay(my_dev->clock_delay_us);
    }
    return 0;
}

// ACK/NACK을 기다리는 함수
static int gpio_comm_wait_for_ack(void) {
    // TODO: m_clk를 high로 설정하고 s_clk가 rising edge가 될 때까지 대기
    // Timeout 구현 필요
    // 읽은 데이터가 ACK인지 NACK인지 반환
    return 0; // 0: ACK, -1: NACK or Timeout
}

// =================================================================
// 4. file_operations 구현
// =================================================================

static int gpio_comm_open(struct inode *inode, struct file *filp) {
    pr_info("[%s] device open\n", DRIVER_NAME);
    
    // TODO:
    // 1. 모든 핀의 direction을 'in'으로 초기화
    // 2. Master 제어핀(m_clk)을 'out', 'high'로 설정
    // 3. 클럭 속도 보정(Handshake & Calibration) 로직 수행
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
    // 1. 'count' 크기의 데이터를 'PACKET_SIZE'로 분할
    // 2. 각 패킷에 대해 루프 실행:
    //    a. 헤더 생성 (총 패킷 수, 현재 패킷 번호, 타입 등)
    //    b. 데이터와 CRC 추가
    //    c. gpio_comm_send_byte 함수를 반복 호출하여 패킷 전송
    //    d. gpio_comm_wait_for_ack 함수로 응답 대기
    //    e. NACK 또는 Timeout 시 재전송 로직 수행
    
    kfree(k_buf);
    return total_sent; // 실제 전송된 바이트 수 반환
}

static int gpio_comm_release(struct inode *inode, struct file *filp) {
    pr_info("[%s] device release\n", DRIVER_NAME);
    return 0;
}

// file_operations 구조체 정의
static const struct file_operations gpio_comm_fops = {
    .owner = THIS_MODULE,
    .open = gpio_comm_open,
    .write = gpio_comm_write,
    // .read = gpio_comm_read, // 슬레이브로부터 데이터 읽기 기능
    .release = gpio_comm_release,
};


// =================================================================
// 5. 모듈 초기화 및 종료
// =================================================================

static int __init gpio_comm_init(void) {
    // 메모리 할당
    my_dev = kmalloc(sizeof(struct gpio_comm_dev), GFP_KERNEL);
    if (!my_dev) return -ENOMEM;
    
    // 문자 디바이스 등록
    alloc_chrdev_region(&my_dev->dev_num, 0, 1, DRIVER_NAME);
    cdev_init(&my_dev->cdev, &gpio_comm_fops);
    cdev_add(&my_dev->cdev, my_dev->dev_num, 1);
    
    // 디바이스 파일 생성을 위한 클래스 생성
    my_dev->dev_class = class_create(CLASS_NAME);
    device_create(my_dev->dev_class, NULL, my_dev->dev_num, NULL, DRIVER_NAME);
    
    // GPIO 핀 초기화 (gpiod_get 함수 사용)
    // 예시: my_dev->d0 = gpiod_get(NULL, GPIO_PIN_D0, GPIOD_OUT_LOW);
    // TODO: 모든 핀을 gpiod_get으로 가져와서 my_dev 구조체에 저장
    
    pr_info("[%s] module loaded\n", DRIVER_NAME);
    return 0;
}

static void __exit gpio_comm_exit(void) {
    // TODO: 할당된 GPIO 핀들을 gpiod_put으로 해제
    
    device_destroy(my_dev->dev_class, my_dev->dev_num);
    class_destroy(my_dev->dev_class);
    cdev_del(&my_dev->cdev);
    unregister_chrdev_region(my_dev->dev_num, 1);
    kfree(my_dev);
    
    pr_info("[%s] module unloaded\n", DRIVER_NAME);
}

module_init(gpio_comm_init);
module_exit(gpio_comm_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("HJH");
MODULE_DESCRIPTION("GPIO Communication Protocol Driver");