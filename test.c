/*
 * =====================================================================================
 * 헤더 파일 (Header Files)
 * =====================================================================================
 * 각 헤더 파일이 어떤 기능을 제공하는지 이해하는 것이 중요합니다.
 */

// 커널 모듈 작성의 필수 헤더. module_init(), module_exit(), MODULE_LICENSE() 등 매크로 포함.
#include <linux/module.h>

// __init, __exit 매크로 정의. 모듈 초기화/종료 함수에 사용하여 메모리 최적화.
#include <linux/init.h>

// gpiod_* 함수들 (gpiod_get_value, gpiod_direction_input 등)을 사용하기 위한 최신 GPIO 인터페이스.
#include <linux/gpio/consumer.h>

// 디바이스 드라이버 모델 관련 헤더. struct class, struct device, device_create(), class_create() 등 포함.
#include <linux/device.h>

// 파일 시스템 관련 헤더. file_operations 구조체, alloc_chrdev_region(), register_chrdev_region() 등 포함.
#include <linux/fs.h>

// 커널 공간과 유저 공간 간의 안전한 데이터 복사를 위한 함수 (copy_to_user, copy_from_user) 포함.
#include <linux/uaccess.h>

// 커널 메모리 할당 함수 (kmalloc, kzalloc, kfree) 포함. 'slab'은 커널의 메모리 할당자 이름.
#include <linux/slab.h>

// 캐릭터 디바이스(Character Device) 관리를 위한 cdev 구조체 및 관련 함수 (cdev_init, cdev_add, cdev_del) 포함.
#include <linux/cdev.h>

// 인터럽트 처리 관련 함수 (request_irq, free_irq) 및 플래그(IRQF_*) 포함.
#include <linux/interrupt.h>

// 비동기 I/O(asynchronous I/O) 지원을 위한 fasync_helper 함수 포함.
#include <linux/fcntl.h>

// kill_fasync 함수와 시그널(SIGIO) 관련 정의 포함.
#include <linux/signal.h>

// poll 시스템 콜 지원을 위한 poll_table, poll_wait 등 관련 기능 포함.
#include <linux/poll.h>

/*
 * =====================================================================================
 * 매크로 정의 (Macro Definitions)
 * =====================================================================================
 */

// 이 드라이버가 생성할 디바이스 클래스의 이름. /sys/class/sysprog_gpio/ 경로로 나타남.
#define CLASS_NAME "sysprog_gpio"
// 이 드라이버가 최대로 관리할 수 있는 GPIO 핀의 개수.
#define MAX_GPIO 10
// 라즈베리 파이의 GPIO 컨트롤러(BCM2835)의 기본 번호. 커널 내부에서 GPIO를 식별할 때 사용.
// `gpioinfo` 명령어로 실제 base를 확인해야 할 수 있음.
#define GPIOCHIP_BASE 512

// ioctl 명령을 위한 매직 넘버. 다른 디바이스 드라이버의 ioctl 명령과 충돌하지 않도록 고유한 값을 사용.
#define GPIO_IOCTL_MAGIC       'G'
// 인터럽트 활성화를 위한 ioctl 명령 정의. _IOW는 'I/O Write'를 의미하며 유저 공간에서 커널로 데이터를 쓴다는 뜻.
#define GPIO_IOCTL_ENABLE_IRQ  _IOW(GPIO_IOCTL_MAGIC, 1, int)
// 인터럽트 비활성화를 위한 ioctl 명령 정의.
#define GPIO_IOCTL_DISABLE_IRQ _IOW(GPIO_IOCTL_MAGIC, 2, int)

/*
 * =====================================================================================
 * 전역 변수 및 자료구조 (Global Variables & Data Structures)
 * =====================================================================================
 */

// 디바이스 번호(주/부 번호)를 저장할 변수. alloc_chrdev_region()으로 할당받음.
static dev_t dev_num_base;
// 캐릭터 디바이스를 표현하는 구조체. cdev_init()으로 초기화하고 cdev_add()로 커널에 등록.
static struct cdev gpio_cdev;
// 할당받은 주 번호(Major Number)를 저장할 변수.
static int major_num;

// 관리하는 각 GPIO 핀의 정보를 담는 구조체.
struct gpio_entry {
    int bcm_num;                  // BCM 핀 번호 (예: 17, 27)
    struct gpio_desc *desc;       // 해당 GPIO를 가리키는 디스크립터 (최신 gpiod 인터페이스의 핵심)
    struct device *dev;           // 이 GPIO에 할당된 디바이스 구조체 (/dev 및 /sys 파일과 연결됨)
    int irq_num;                  // 이 GPIO에 할당된 인터럽트 번호
    bool irq_enabled;             // 인터럽트가 현재 활성화되었는지 여부 플래그
    struct fasync_struct *async_queue; // 비동기 알림(SIGIO)을 보낼 프로세스 큐
};

// 이 드라이버가 생성할 디바이스 클래스를 가리키는 포인터.
static struct class *gpiod_class;
// export된 GPIO들의 정보를 저장하는 포인터 배열. 부 번호(minor)가 이 배열의 인덱스로 사용됨.
static struct gpio_entry *gpio_table[MAX_GPIO];

/*
 * =====================================================================================
 * 유틸리티 함수 (Utility Functions)
 * =====================================================================================
 */

// BCM 핀 번호로 gpio_table에서 해당 GPIO의 인덱스(부 번호)를 찾음.
static int find_gpio_index(int bcm) {
    for (int i = 0; i < MAX_GPIO; i++) {
        // 테이블의 해당 인덱스가 존재하고 BCM 번호가 일치하는지 확인
        if (gpio_table[i] && gpio_table[i]->bcm_num == bcm)
            return i;
    }
    return -1; // 찾지 못함
}

/*
 * =====================================================================================
 * 인터럽트 핸들러 (Interrupt Handler)
 * =====================================================================================
 */

// GPIO 인터럽트가 발생했을 때 커널에 의해 호출될 함수 (IRQ 핸들러).
// irq: 발생한 인터럽트 번호, dev_id: request_irq() 때 등록한 데이터 (여기서는 struct gpio_entry)
static irqreturn_t gpio_irq_handler(int irq, void *dev_id) {
    struct gpio_entry *entry = dev_id; // dev_id를 원래 타입으로 캐스팅

    // 커널 로그에 인터럽트 발생을 알림. dmesg 명령어로 확인 가능.
    pr_info("[sysprog_gpio] IRQ on GPIO %d\n", entry->bcm_num);

    // fasync 큐에 등록된 프로세스가 있다면
    if (entry->async_queue)
        // SIGIO 시그널을 보내 파일 디스크립터에 데이터가 있음을(POLL_IN) 알림.
        kill_fasync(&entry->async_queue, SIGIO, POLL_IN);

    // 인터럽트가 성공적으로 처리되었음을 커널에 알림.
    return IRQ_HANDLED;
}

/*
 * =====================================================================================
 * 파일 오퍼레이션 (File Operations)
 * =====================================================================================
 * /dev/gpioX 파일을 대상으로 open, read, write 등의 함수가 호출될 때 실행될 실제 코드.
 */

// `open("/dev/gpioX")` 시 호출
static int gpio_fops_open(struct inode *inode, struct file *filp) {
    // inode로부터 부 번호(minor number)를 얻어옴. 이 번호가 gpio_table의 인덱스가 됨.
    int minor = iminor(inode);

    // 유효한 부 번호인지, 해당 GPIO가 export 되었는지 확인
    if (minor >= MAX_GPIO || !gpio_table[minor])
        return -ENODEV; // No such device 오류 반환

    // file 구조체의 private_data 필드에 해당 GPIO의 정보(gpio_entry)를 저장.
    // 이렇게 하면 이후 read, write 등에서 이 정보에 쉽게 접근 가능.
    filp->private_data = gpio_table[minor];
    return 0; // 성공
}

// `close()` 시 호출
static int gpio_fops_release(struct inode *inode, struct file *filp) {
    struct gpio_entry *entry = filp->private_data;

    // 만약 파일이 닫힐 때 인터럽트가 활성화 상태였다면, 자동으로 해제해줌. (리소스 누수 방지)
    if (entry && entry->irq_enabled) {
        free_irq(entry->irq_num, entry);
        entry->irq_enabled = false;
    }

    // 이 파일에 대해 설정된 비동기 I/O 설정을 해제함.
    fasync_helper(-1, filp, 0, &entry->async_queue);
    return 0;
}

// `fcntl(fd, F_SETFL, FASYNC)` 등으로 비동기 모드를 설정/해제할 때 호출됨.
static int gpio_fops_fasync(int fd, struct file *filp, int mode) {
    struct gpio_entry *entry = filp->private_data;
    // fasync_helper가 실제 작업을 처리. 프로세스를 entry->async_queue에 추가하거나 제거.
    return fasync_helper(fd, filp, mode, &entry->async_queue);
}

// `ioctl()` 시 호출
static long gpio_fops_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
    struct gpio_entry *entry = filp->private_data;
    int irq;

    switch (cmd) {
    case GPIO_IOCTL_ENABLE_IRQ: // 인터럽트 활성화 명령
        if (entry->irq_enabled)
            return -EBUSY; // 이미 활성화됨

        // GPIO 디스크립터로부터 IRQ 번호를 얻어옴.
        irq = gpiod_to_irq(entry->desc);
        if (irq < 0) return -EINVAL; // 유효하지 않은 IRQ

        // 커널에 IRQ 핸들러를 등록.
        // IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING: 상승 엣지와 하강 엣지 모두에서 트리거.
        if (request_irq(irq, gpio_irq_handler,
                        IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                        "gpio_irq", entry)) { // 'entry'를 핸들러에 전달
            pr_err("[sysprog_gpio] IRQ request failed\n");
            return -EIO; // I/O 에러
        }
        entry->irq_num = irq;
        entry->irq_enabled = true;
        return 0;

    case GPIO_IOCTL_DISABLE_IRQ: // 인터럽트 비활성화 명령
        if (!entry->irq_enabled)
            return -EINVAL; // 활성화되지 않았음

        // 등록된 IRQ 핸들러를 해제.
        free_irq(entry->irq_num, entry);
        entry->irq_enabled = false;
        return 0;

    default:
        return -ENOTTY; // 지원하지 않는 ioctl 명령
    }
}

// `read()` 시 호출
static ssize_t gpio_fops_read(struct file *filp, char __user *buf, size_t len, loff_t *off) {
    struct gpio_entry *entry = filp->private_data;
    // gpiod_get_value로 현재 GPIO 핀의 값을 읽음 (0 또는 1).
    // 그 값을 문자 '0' 또는 '1'로 변환.
    char val = gpiod_get_value(entry->desc) ? '1' : '0';

    // 커널 공간의 데이터를 유저 공간 버퍼(buf)로 복사.
    if (copy_to_user(buf, &val, 1))
        return -EFAULT; // 복사 실패

    return 1; // 1바이트를 성공적으로 읽었음을 반환.
}

// `write()` 시 호출
static ssize_t gpio_fops_write(struct file *filp, const char __user *buf, size_t len, loff_t *off) {
    struct gpio_entry *entry = filp->private_data;
    char kbuf[8] = {0}; // 유저 공간에서 받은 데이터를 저장할 커널 버퍼

    if (len >= sizeof(kbuf)) return -EINVAL; // 너무 긴 입력 방지
    // 유저 공간 버퍼(buf)에서 커널 버퍼(kbuf)로 데이터 복사.
    if (copy_from_user(kbuf, buf, len)) return -EFAULT;

    kbuf[len] = '\0'; // 문자열로 다루기 위해 널 종료 문자 추가

    if (sysfs_streq(kbuf, "1")) { // "1"을 쓰면
        if (gpiod_get_direction(entry->desc)) return -EPERM; // 입력 모드일 경우 쓰기 금지
        gpiod_set_value(entry->desc, 1); // HIGH로 설정
    } else if (sysfs_streq(kbuf, "0")) { // "0"을 쓰면
        if (gpiod_get_direction(entry->desc)) return -EPERM;
        gpiod_set_value(entry->desc, 0); // LOW로 설정
    } else if (sysfs_streq(kbuf, "in")) { // "in"을 쓰면
        gpiod_direction_input(entry->desc); // 입력 모드로 설정
    } else if (sysfs_streq(kbuf, "out")) { // "out"을 쓰면
        gpiod_direction_output(entry->desc, 0); // 출력 모드로 설정 (기본값 LOW)
    } else {
        return -EINVAL; // 잘못된 값
    }
    return len; // 성공적으로 쓴 바이트 수 반환
}

// 위에서 정의한 파일 오퍼레이션 함수들을 file_operations 구조체에 등록.
static const struct file_operations gpio_fops = {
    .owner = THIS_MODULE, // 이 모듈이 소유자임을 명시. 모듈이 사용 중일 때 rmmod 되는 것을 방지.
    .open = gpio_fops_open,
    .read = gpio_fops_read,
    .write = gpio_fops_write,
    .release = gpio_fops_release,
    .fasync = gpio_fops_fasync,
    .unlocked_ioctl = gpio_fops_ioctl, // 락이 없는 ioctl. 현대 드라이버에서는 이것을 사용.
};

/*
 * =====================================================================================
 * Sysfs 디바이스 속성 (Device Attributes)
 * =====================================================================================
 * /sys/class/sysprog_gpio/gpioX/value, /sys/class/sysprog_gpio/gpioX/direction 파일들.
 */

// `cat /sys/.../gpioX/value` 시 호출
static ssize_t value_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct gpio_entry *entry = dev_get_drvdata(dev); // device 구조체에서 private 데이터(gpio_entry) 가져오기
    int val = gpiod_get_value(entry->desc);
    return scnprintf(buf, PAGE_SIZE, "%d\n", val); // 현재 값을 버퍼에 써서 유저에게 전달
}

// `echo 1 > /sys/.../gpioX/value` 시 호출
static ssize_t value_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    struct gpio_entry *entry = dev_get_drvdata(dev);
    // gpiod_get_direction()이 0이면 출력, 1이면 입력. 즉, 입력모드이면 쓰기 불가.
    if (gpiod_get_direction(entry->desc)) return -EPERM;
    if (buf[0] == '1') gpiod_set_value(entry->desc, 1);
    else if (buf[0] == '0') gpiod_set_value(entry->desc, 0);
    else return -EINVAL;
    return count;
}

// `cat /sys/.../gpioX/direction` 시 호출
static ssize_t direction_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct gpio_entry *entry = dev_get_drvdata(dev);
    int dir = gpiod_get_direction(entry->desc);
    return scnprintf(buf, PAGE_SIZE, "%s\n", dir ? "in" : "out"); // 0이면 "out", 1이면 "in"
}

// `echo "in" > /sys/.../gpioX/direction` 시 호출
static ssize_t direction_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    struct gpio_entry *entry = dev_get_drvdata(dev);
    if (sysfs_streq(buf, "in")) gpiod_direction_input(entry->desc);
    else if (sysfs_streq(buf, "out")) gpiod_direction_output(entry->desc, 0);
    else return -EINVAL;
    return count;
}

// value와 direction 속성을 한번에 정의하는 매크로.
// 이 매크로는 value_show, value_store 함수를 가지는 dev_attr_value 구조체를 생성.
static DEVICE_ATTR_RW(value);
static DEVICE_ATTR_RW(direction);


/*
 * =====================================================================================
 * Sysfs 클래스 속성 (Class Attributes)
 * =====================================================================================
 * /sys/class/sysprog_gpio/export, /sys/class/sysprog_gpio/unexport 파일들.
 */

// `echo 17 > /sys/class/sysprog_gpio/export` 시 호출
static ssize_t export_store(const struct class *class, const struct class_attribute *attr, const char *buf, size_t count) {
    int bcm, i;
    int kernel_gpio;
    struct gpio_entry *entry;
    char name[16];

    // 유저가 입력한 문자열(buf)을 정수(bcm)로 변환
    if (kstrtoint(buf, 10, &bcm) < 0)
        return -EINVAL;

    // 이미 export된 핀인지 확인
    if (find_gpio_index(bcm) >= 0)
        return -EEXIST;

    // gpio_table에서 비어있는 슬롯(인덱스)을 찾음
    for (i = 0; i < MAX_GPIO; i++) {
        if (!gpio_table[i]) break;
    }
    if (i == MAX_GPIO) return -ENOMEM; // 테이블이 꽉 참

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

    // 디바이스 파일 이름을 "gpio17"과 같은 형식으로 만듦
    snprintf(name, sizeof(name), "gpio%d", bcm);
    // /dev/gpio17 디바이스 파일과 /sys/class/sysprog_gpio/gpio17 디렉터리를 생성
    entry->dev = device_create(gpiod_class, NULL, MKDEV(major_num, i), NULL, name);
    if (IS_ERR(entry->dev)) {
        kfree(entry);
        return PTR_ERR(entry->dev);
    }

    // 생성된 디바이스에 private 데이터로 entry를 저장
    dev_set_drvdata(entry->dev, entry);
    // sysfs에 value와 direction 속성 파일을 생성
    device_create_file(entry->dev, &dev_attr_value);
    device_create_file(entry->dev, &dev_attr_direction);

    // gpio_table에 새로운 entry를 등록
    gpio_table[i] = entry;

    pr_info("[sysprog_gpio] Exported GPIO %d\n", bcm);
    return count;
}

// `echo 17 > /sys/class/sysprog_gpio/unexport` 시 호출
static ssize_t unexport_store(const struct class *class, const struct class_attribute *attr, const char *buf, size_t count) {
    int bcm, idx;

    if (kstrtoint(buf, 10, &bcm) < 0)
        return -EINVAL;

    // export된 GPIO인지 확인하고 인덱스를 찾음
    idx = find_gpio_index(bcm);
    if (idx < 0) return -ENOENT; // 해당 항목 없음

    // export_store에서 했던 작업들을 역순으로 수행
    device_remove_file(gpio_table[idx]->dev, &dev_attr_value);
    device_remove_file(gpio_table[idx]->dev, &dev_attr_direction);
    device_destroy(gpiod_class, MKDEV(major_num, idx));
    kfree(gpio_table[idx]);
    gpio_table[idx] = NULL;

    pr_info("[sysprog_gpio] Unexported GPIO %d\n", bcm);
    return count;
}

// export와 unexport 클래스 속성을 쓰기 전용(Write-Only)으로 정의하는 매크로.
static CLASS_ATTR_WO(export);
static CLASS_ATTR_WO(unexport);

/*
 * =====================================================================================
 * 모듈 초기화 및 종료 함수 (Module Init & Exit)
 * =====================================================================================
 */

// `insmod` 시 호출되는 초기화 함수
static int __init gpio_driver_init(void) {
    int ret;
    pr_info("[sysprog_gpio] module loading\n");

    // 1. 디바이스 클래스 생성 (/sys/class/sysprog_gpio/ 생성)
    gpiod_class = class_create(CLASS_NAME);
    if (IS_ERR(gpiod_class)) return PTR_ERR(gpiod_class);

    // 2. 클래스 속성 파일 생성 (/sys/class/sysprog_gpio/export 생성)
    ret = class_create_file(gpiod_class, &class_attr_export);
    if (ret) return ret;

    // 3. 클래스 속성 파일 생성 (/sys/class/sysprog_gpio/unexport 생성)
    ret = class_create_file(gpiod_class, &class_attr_unexport);
    if (ret) return ret;

    // 4. 캐릭터 디바이스를 위한 주/부 번호 할당 요청
    ret = alloc_chrdev_region(&dev_num_base, 0, MAX_GPIO, "gpio");
    if (ret) return ret;

    // 5. 할당받은 주 번호 저장
    major_num = MAJOR(dev_num_base);
    
    // 6. cdev 구조체 초기화 및 파일 오퍼레이션 연결
    cdev_init(&gpio_cdev, &gpio_fops);
    gpio_cdev.owner = THIS_MODULE;

    // 7. cdev를 커널에 등록하여 캐릭터 디바이스 드라이버로 동작 시작
    ret = cdev_add(&gpio_cdev, dev_num_base, MAX_GPIO);
    if (ret) return ret;

    return 0; // 성공
}

// `rmmod` 시 호출되는 종료 함수
static void __exit gpio_driver_exit(void) {
    // 초기화의 역순으로 모든 리소스를 해제해야 함.

    // 1. Export된 모든 GPIO 장치 제거
    for (int i = 0; i < MAX_GPIO; i++) {
        if (gpio_table[i]) {
            device_remove_file(gpio_table[i]->dev, &dev_attr_value);
            device_remove_file(gpio_table[i]->dev, &dev_attr_direction);
            device_destroy(gpiod_class, MKDEV(major_num, i));
            kfree(gpio_table[i]);
            gpio_table[i] = NULL;
        }
    }

    // 2. cdev 등록 해제
    cdev_del(&gpio_cdev);
    // 3. 할당받았던 디바이스 번호 반납
    unregister_chrdev_region(dev_num_base, MAX_GPIO);

    // 4. 클래스 속성 파일 제거
    class_remove_file(gpiod_class, &class_attr_export);
    class_remove_file(gpiod_class, &class_attr_unexport);
    // 5. 디바이스 클래스 제거
    class_destroy(gpiod_class);

    pr_info("[sysprog_gpio] module unloaded\n");
}

// 모듈 진입점과 종료점을 커널에 알림
module_init(gpio_driver_init);
module_exit(gpio_driver_exit);

// 모듈 라이선스. "GPL"이 아니면 일부 커널 심볼(GPL 전용)을 사용할 수 없음.
MODULE_LICENSE("GPL");
// 모듈 작성자 정보
MODULE_AUTHOR("Jiwoong Park");
// 모듈에 대한 간단한 설명
MODULE_DESCRIPTION("GPIO control driver for system programming lectures");
