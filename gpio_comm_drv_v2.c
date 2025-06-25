
#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio/consumer.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/timer.h>

#define DRIVER_NAME "gpiocom"
#define CLASS_NAME  "gpiocom_class"

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
MODULE_DESCRIPTION("GPIO Communication Protocol Driver v2.0");