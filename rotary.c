#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/jiffies.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("KJY");
MODULE_DESCRIPTION("Pure Rotary Encoder Driver with Time-based Debouncing");
MODULE_VERSION("1.2-final-debounced");

#define DEVICE_NAME "rotary_dev"

// �Է� GPIO �� ����
#define ROTARY_S1   (17 + 512)
#define ROTARY_S2   (27 + 512)
#define ROTARY_KEY  (22 + 512)

#define DEBOUNCE_TIME_MS 50

// ���� ����
static int rotary_count = 0;
static int current_mode = 0; // 0: Rotary/Battery ���, 1: Time ���

static dev_t dev_num;
static struct cdev rotary_cdev;
static struct class* rotary_class;
static struct device* rotary_device;
static int rotary_irq_clk, key_switch_irq;
static int last_rotary_clk_state;
static unsigned long last_rotary_irq_time = 0; // �ð� ��� ��ٿ���� ���� ����
static struct timer_list debounce_timer;

// �Լ� ����
static ssize_t rotary_read(struct file*, char __user*, size_t, loff_t*);

// ���� ���۷��̼� ����ü
static const struct file_operations rotary_fops = {
    .owner = THIS_MODULE,
    .read = rotary_read,
};

static irqreturn_t rotary_encoder_isr(int irq, void* dev_id) {
    unsigned long current_time = jiffies;

    // 2ms �̳��� �߻��� �������� ���ͷ�Ʈ�� �����Ͽ� �ٿ���� �����մϴ�.
    if (time_after(current_time, last_rotary_irq_time + msecs_to_jiffies(2))) {
        int current_clk_state = gpio_get_value(ROTARY_S1);
        if (current_clk_state != last_rotary_clk_state) {
            int dt_state = gpio_get_value(ROTARY_S2);
            // CLK ��ȣ�� ��������(Falling) �������� ���� �н��ϴ�.
            if (current_clk_state == 0) {
                if (dt_state == 1) {
                    rotary_count++;
                }
                else {
                    rotary_count--;
                }
            }
        }
        last_rotary_clk_state = current_clk_state;
        last_rotary_irq_time = current_time; // ������ ���ͷ�Ʈ �ð� ���
    }
    return IRQ_HANDLED;
}

// Ű ����ġ ��ٿ �ݹ�
static void debounce_timer_callback(struct timer_list* t) {
    if (gpio_get_value(ROTARY_KEY) == 0) {
        current_mode = !current_mode; // ��� ��ȯ
        printk(KERN_INFO "Rotary Key Pressed: Mode changed to %d\n", current_mode);
    }
    enable_irq(key_switch_irq);
}

// Ű ����ġ ISR
static irqreturn_t key_switch_isr(int irq, void* dev_id) {
    disable_irq_nosync(key_switch_irq);
    mod_timer(&debounce_timer, jiffies + msecs_to_jiffies(DEBOUNCE_TIME_MS));
    return IRQ_HANDLED;
}

// ����� �������� �����͸� �����ϴ� read �Լ�
static ssize_t rotary_read(struct file* file, char __user* buf, size_t count, loff_t* pos) {
    int len;
    char kbuf[32];
    len = scnprintf(kbuf, sizeof(kbuf), "MODE=%d,COUNT=%d", current_mode, rotary_count);
    if (count < len) return -EINVAL;
    if (copy_to_user(buf, kbuf, len)) return -EFAULT;
    return len;
}

static int __init rotary_init(void) {
    int ret;
    static const struct gpio inputs[] = {
        { ROTARY_S1, GPIOF_IN, "r_clk" }, { ROTARY_S2, GPIOF_IN, "r_dt" },
        { ROTARY_KEY, GPIOF_IN, "r_sw" },
    };
    int i;

    ret = alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0) return ret;
    cdev_init(&rotary_cdev, &rotary_fops);
    ret = cdev_add(&rotary_cdev, dev_num, 1);
    if (ret < 0) goto unregister_chrdev;
    rotary_class = class_create(DEVICE_NAME);
    if (IS_ERR(rotary_class)) { ret = PTR_ERR(rotary_class); goto cdev_del; }
    rotary_device = device_create(rotary_class, NULL, dev_num, NULL, DEVICE_NAME);
    if (IS_ERR(rotary_device)) { ret = PTR_ERR(rotary_device); goto class_destroy; }

    for (i = 0; i < ARRAY_SIZE(inputs); i++) {
        ret = gpio_request_one(inputs[i].gpio, inputs[i].flags, inputs[i].label);
        if (ret) { for (i--; i >= 0; i--) gpio_free(inputs[i].gpio); goto device_destroy; }
    }

    last_rotary_clk_state = gpio_get_value(ROTARY_S1);
    timer_setup(&debounce_timer, debounce_timer_callback, 0);

    rotary_irq_clk = gpio_to_irq(ROTARY_S1);
    ret = request_irq(rotary_irq_clk, rotary_encoder_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "rotary_clk_irq", NULL);
    if (ret) goto free_gpios;

    key_switch_irq = gpio_to_irq(ROTARY_KEY);
    ret = request_irq(key_switch_irq, key_switch_isr, IRQF_TRIGGER_FALLING, "key_switch_irq", NULL);
    if (ret) goto free_rotary_irq;

    printk(KERN_INFO "rotary_drv: Driver loaded.\n");
    return 0;

free_rotary_irq:
    free_irq(rotary_irq_clk, NULL);
free_gpios:
    for (i = 0; i < ARRAY_SIZE(inputs); i++) gpio_free(inputs[i].gpio);
device_destroy:
    device_destroy(rotary_class, dev_num);
class_destroy:
    class_destroy(rotary_class);
cdev_del:
    cdev_del(&rotary_cdev);
unregister_chrdev:
    unregister_chrdev_region(dev_num, 1);
    return ret;
}

static void __exit rotary_exit(void) {
    del_timer_sync(&debounce_timer);
    free_irq(key_switch_irq, NULL);
    free_irq(rotary_irq_clk, NULL);
    gpio_free(ROTARY_S1);
    gpio_free(ROTARY_S2);
    gpio_free(ROTARY_KEY);
    device_destroy(rotary_class, dev_num);
    class_destroy(rotary_class);
    cdev_del(&rotary_cdev);
    unregister_chrdev_region(dev_num, 1);
    printk(KERN_INFO "rotary_dev: Driver unloaded.\n");
}

module_init(rotary_init);
module_exit(rotary_exit);