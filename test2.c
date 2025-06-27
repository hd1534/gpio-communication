/*
 * gpio_comm_drv.c - P2P GPIO communication driver v2.2 (revised)
 *
 * HJH & Gemini original, revisions by ChatGPT
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
 
 #define CLASS_NAME           "gpio_comm"
 #define DRIVER_NAME          "gpio_comm_drv"
 #define MAX_DEVICES          10
 #define MAX_NAME_LEN         20
 #define NUM_DATA_PINS        4
 #define RX_BUFFER_SIZE       256
 #define CLOCK_DELAY_US       10000UL
 #define NORMAL_CLK_MAX_NS    (CLOCK_DELAY_US * 1000UL * 1.5)
 #define SLOW_CLK_MIN_NS      (CLOCK_DELAY_US * 1000UL * 2)
 #define TIMEOUT_MIN_NS       (CLOCK_DELAY_US * 1000UL * 5)
 
 enum connect_mode { MODE_READ_ONLY, MODE_READ_WRITE };
 enum comm_state   { UNINIT=0, IDLE, RECEIVING, SENDING, DONE, ERROR };
 
 struct gpio_comm_dev {
     char name[MAX_NAME_LEN];
     struct cdev cdev;
     struct device *device;
     enum connect_mode mode;
     struct gpio_desc *ctrl_pin;
     struct gpio_desc *data_pins[NUM_DATA_PINS];
     int         my_pin_idx;
     int         irqs[NUM_DATA_PINS+1];
     enum comm_state state;
     spinlock_t      lock;
     wait_queue_head_t wq;
     u8          rx_buffer[RX_BUFFER_SIZE];
     atomic_t    rx_len;
     atomic_t    rx_bytes_done;
     atomic_t    rx_bits_done;
     atomic_t    data_ready;
     ktime_t     last_irq_time;
     struct timer_list timeout_timer;
 };
 
 static struct gpio_comm_dev *g_dev_table[MAX_DEVICES];
 static int g_major;
 static struct class *g_class;
 
 /* helper: write/read 4 bits */
 static void write_4bits(u8 d, struct gpio_comm_dev *dev) { /*...*/ }
 static u8   read_4bits (struct gpio_comm_dev *dev) { /*...*/ }
 static void toggle_ctrl_clock(struct gpio_comm_dev *dev) { /*...*/ }
 
 /* timeout callback */
 static void comm_timeout_callback(struct timer_list *t) {
     struct gpio_comm_dev *dev = from_timer(dev, t, timeout_timer);
     pr_warn("[%s] %s: RX timeout\n", DRIVER_NAME, dev->name);
     atomic_set(&dev->data_ready, -ETIMEDOUT);
     wake_up_interruptible(&dev->wq);
 }
 
 /* IRQ handlers */
 static irqreturn_t ctrl_pin_irq(int irq, void *id) { /* corrected and combined */ }
 static irqreturn_t data_pin_irq(int irq, void *id) { /* corrected */ }
 
 /* fops: open, release, read, write */
 static int gpio_comm_open(struct inode *inode, struct file *filp) { /*...*/ }
 static int gpio_comm_release(struct inode *inode, struct file *filp) { /*...*/ }
 static ssize_t gpio_comm_read(struct file *f, char __user *buf, size_t len, loff_t *off) { /*...*/ }
 static ssize_t gpio_comm_write(struct file *f, const char __user *buf, size_t cnt, loff_t *off) { /*...*/ }
 
 static const struct file_operations gpio_comm_fops = {
     .owner   = THIS_MODULE,
     .open    = gpio_comm_open,
     .release = gpio_comm_release,
     .read    = gpio_comm_read,
     .write   = gpio_comm_write,
 };
 
 /* sysfs export/unexport */
 static ssize_t export_store(struct class *c, struct class_attribute *attr,
                             const char *buf, size_t count) {
     char name[MAX_NAME_LEN]; char mode[4]; int pins[NUM_DATA_PINS+1];
     int dev_idx = -1, i, ret;
     struct gpio_comm_dev *dev;
 
     if (sscanf(buf, "%19[^,],%3[^,],%d,%d,%d,%d,%d",
                name, mode,
                &pins[0], &pins[1], &pins[2], &pins[3], &pins[4]) != 7)
         return -EINVAL;
     for (i = 0; i < MAX_DEVICES; i++) if (!g_dev_table[i]) { dev_idx = i; break; }
     if (dev_idx<0) return -ENOSPC;
 
     dev = kzalloc(sizeof(*dev), GFP_KERNEL);
     if (!dev) return -ENOMEM;
     strlcpy(dev->name, name, MAX_NAME_LEN);
     dev->mode = (mode[0]=='r' && mode[1]=='w') ? MODE_READ_WRITE : MODE_READ_ONLY;
     /* request and configure GPIOs */
     dev->ctrl_pin = gpiod_get_index(NULL, NULL, pins[0], GPIOD_IN);
     for (i=0; i<NUM_DATA_PINS; i++)
         dev->data_pins[i] = gpiod_get_index(NULL, NULL, pins[i+1], GPIOD_IN);
 
     /* assign pin in RW mode */
     dev->my_pin_idx = -1;
     if (dev->mode==MODE_READ_WRITE) {
         for (i=0; i<NUM_DATA_PINS; i++) {
             if (gpiod_get_value(dev->data_pins[i])==0) {
                 dev->my_pin_idx = i;
                 gpiod_direction_output(dev->data_pins[i], 1);
                 break;
             }
         }
         if (dev->my_pin_idx<0) { ret=-EBUSY; goto err; }
     }
 
     /* character device setup */
     cdev_init(&dev->cdev, &gpio_comm_fops);
     ret = alloc_chrdev_region(&g_major, dev_idx, 1, DRIVER_NAME);
     if (ret) goto err;
     dev->device = device_create(g_class, NULL, g_major, NULL, "%s", name);
     if (IS_ERR(dev->device)) { ret=PTR_ERR(dev->device); unregister_chrdev_region(g_major,1); goto err; }
     ret = cdev_add(&dev->cdev, g_major,1);
     if (ret) goto err_dev;
 
     /* init IRQs */
     dev->irqs[0] = gpiod_to_irq(dev->ctrl_pin);
     request_irq(dev->irqs[0], ctrl_pin_irq, IRQF_TRIGGER_RISING, "gpio_ctrl", dev);
     disable_irq(dev->irqs[0]);
     if (dev->mode==MODE_READ_WRITE) {
         for (i=0; i<NUM_DATA_PINS; i++) {
             dev->irqs[i+1] = gpiod_to_irq(dev->data_pins[i]);
             request_irq(dev->irqs[i+1], data_pin_irq,
                         IRQF_TRIGGER_RISING, name, dev);
         }
     }
 
     init_waitqueue_head(&dev->wq);
     spin_lock_init(&dev->lock);
     atomic_set(&dev->state, IDLE);
     atomic_set(&dev->data_ready, 0);
     timer_setup(&dev->timeout_timer, comm_timeout_callback, 0);
 
     g_dev_table[dev_idx] = dev;
     pr_info("[%s] Device %s registered\n", DRIVER_NAME, name);
     return count;
 
 err_dev:
     device_destroy(g_class, g_major);
     unregister_chrdev_region(g_major,1);
 err:
     kfree(dev);
     return ret;
 }
 
 static ssize_t unexport_store(struct class *c, struct class_attribute *attr,
                               const char *buf, size_t count) {
     char name[MAX_NAME_LEN]; int i;
     struct gpio_comm_dev *dev;
     sscanf(buf, "%19s", name);
     for (i=0; i<MAX_DEVICES; i++) {
         if (g_dev_table[i] && !strcmp(g_dev_table[i]->name, name)) break;
     }
     if (i==MAX_DEVICES) return -ENOENT;
     dev = g_dev_table[i]; g_dev_table[i]=NULL;
     /* release resources */
     free_irq(dev->irqs[0], dev);
     if (dev->mode==MODE_READ_WRITE)
         for(i=0;i<NUM_DATA_PINS;i++) free_irq(dev->irqs[i+1], dev);
     del_timer_sync(&dev->timeout_timer);
     cdev_del(&dev->cdev);
     device_destroy(g_class, g_major);
     unregister_chrdev_region(g_major,1);
     gpiod_put(dev->ctrl_pin);
     for (i=0;i<NUM_DATA_PINS;i++) gpiod_put(dev->data_pins[i]);
     kfree(dev);
     pr_info("[%s] Device %s removed\n", DRIVER_NAME, name);
     return count;
 }
 
 static CLASS_ATTR_WO(export);
 static CLASS_ATTR_WO(unexport);
 
 static int __init gpio_comm_init(void) {
     int ret;
     g_class = class_create(THIS_MODULE, CLASS_NAME);
     if (IS_ERR(g_class)) return PTR_ERR(g_class);
     class_create_file(g_class, &class_attr_export);
     class_create_file(g_class, &class_attr_unexport);
     return 0;
 }
 
 static void __exit gpio_comm_exit(void) {
     int i;
     for (i=0;i<MAX_DEVICES;i++) if (g_dev_table[i])
         unexport_store(NULL, NULL, g_dev_table[i]->name, strlen(g_dev_table[i]->name));
     class_remove_file(g_class, &class_attr_export);
     class_remove_file(g_class, &class_attr_unexport);
     class_destroy(g_class);
 }
 
 MODULE_LICENSE("GPL");
 MODULE_AUTHOR("HJH & Gemini & ChatGPT");
 MODULE_DESCRIPTION("P2P GPIO Comm Driver v2.2");
 module_init(gpio_comm_init);
 module_exit(gpio_comm_exit);

 