#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/err.h>

/* --- ȣȯ���� ���� �ʼ� ��� ���� --- */
#include <linux/gpio.h>          // gpio_request(), gpio_free()
#include <linux/gpio/consumer.h> // gpiod_* �Լ�

/* --- ��� �Ķ���� ����--- */
static int sda_pin = (23 + 512);
module_param(sda_pin, int, S_IRUGO);
MODULE_PARM_DESC(sda_pin, "GPIO pin for I2C SDA (default: 23)");

static int scl_pin = (24 + 512);
module_param(scl_pin, int, S_IRUGO);
MODULE_PARM_DESC(scl_pin, "GPIO pin for I2C SCL (default: 24)");

/* --- GPIO ��ũ���� --- */
static struct gpio_desc* sda_desc;
static struct gpio_desc* scl_desc;


/* --- GPIO ���� �Լ� (�� ������ ���� �����ϴ� ������� ����) --- */
static void set_scl(int val)
{
    if (val) {
        // HIGH�� ����: ���� �Է����� ����� �ܺ� Ǯ�� ������ ������ HIGH�� ���� ��
        gpiod_direction_input(scl_desc);
    }
    else {
        // LOW�� ����: ���� ������� ����� ���� 0V�� ����
        gpiod_direction_output(scl_desc, 0);
    }
    udelay(5);
}

static void set_sda(int val)
{
    if (val) {
        gpiod_direction_input(sda_desc);
    }
    else {
        gpiod_direction_output(sda_desc, 0);
    }
    udelay(5);
}

static int get_sda(void)
{
    // ���� �б� ���� �׻� �Է����� ����
    gpiod_direction_input(sda_desc);
    return gpiod_get_value(sda_desc);
}


/* --- I2C �������� ���� (������ ����, ���� ���۸� �����) --- */
static void i2c_sw_start(void)
{
    set_sda(1);
    set_scl(1);
    set_sda(0);
    set_scl(0);
}

static void i2c_sw_stop(void)
{
    set_scl(0);
    set_sda(0);
    set_scl(1);
    set_sda(1);
}

static void i2c_sw_write_bit(int bit)
{
    set_scl(0);
    set_sda(bit);
    set_scl(1);
    set_scl(0);
}

static int i2c_sw_read_bit(void)
{
    int bit;
    set_scl(1);
    bit = get_sda();
    set_scl(0);
    return bit;
}

static int i2c_sw_write_byte(u8 data)
{
    int i, ack;
    for (i = 7; i >= 0; i--) {
        i2c_sw_write_bit((data >> i) & 1);
    }
    ack = i2c_sw_read_bit();
    return ack;
}

static u8 i2c_sw_read_byte(int ack)
{
    int i;
    u8 data = 0;
    for (i = 7; i >= 0; i--) {
        data <<= 1;
        data |= i2c_sw_read_bit();
    }
    i2c_sw_write_bit(ack);
    return data;
}


/* --- I2C ����� �ٽ� �Լ� (������ ����) --- */
static int i2c_sw_xfer(struct i2c_adapter* adap, struct i2c_msg* msgs, int num)
{
    int i, j, ret = 0;
    for (i = 0; i < num; i++) {
        struct i2c_msg* pmsg = &msgs[i];
        i2c_sw_start();
        ret = i2c_sw_write_byte(i2c_8bit_addr_from_msg(pmsg));
        if (ret) { ret = -EREMOTEIO; goto out; }
        if (pmsg->flags & I2C_M_RD) {
            for (j = 0; j < pmsg->len; j++) { pmsg->buf[j] = i2c_sw_read_byte(j == (pmsg->len - 1)); }
        }
        else {
            for (j = 0; j < pmsg->len; j++) {
                ret = i2c_sw_write_byte(pmsg->buf[j]);
                if (ret) { ret = -EREMOTEIO; goto out; }
            }
        }
    }
    ret = num;
out:
    i2c_sw_stop();
    return ret;
}

static u32 i2c_sw_func(struct i2c_adapter* adapter)
{
    return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm i2c_sw_algo = { .master_xfer = i2c_sw_xfer, .functionality = i2c_sw_func };
static struct i2c_adapter i2c_sw_adapter = { .owner = THIS_MODULE, .name = "Software I2C Adapter", .algo = &i2c_sw_algo };


/* --- Ŀ�� ��� �ʱ�ȭ �� ���� �Լ� (ȣȯ�� ���� ������� ����) --- */
static int __init i2c_sw_init(void)
{
    int ret;

    printk(KERN_INFO "i2c_sw_driver: Loading module with sda_pin=%d, scl_pin=%d\n", sda_pin, scl_pin);

    // --- SDA �� �ʱ�ȭ ---
    ret = gpio_request(sda_pin, "i2c_sda");
    if (ret) {
        printk(KERN_ERR "i2c_sw_driver: Failed to request SDA GPIO %d\n", sda_pin);
        return ret;
    }
    sda_desc = gpio_to_desc(sda_pin);
    if (!sda_desc) {
        ret = -EINVAL;
        goto err_free_sda;
    }
    // �ʱ� ���´� �Է�(HIGH-Z)���� ����
    gpiod_direction_input(sda_desc);


    // --- SCL �� �ʱ�ȭ ---
    ret = gpio_request(scl_pin, "i2c_scl");
    if (ret) {
        printk(KERN_ERR "i2c_sw_driver: Failed to request SCL GPIO %d\n", scl_pin);
        goto err_free_sda;
    }
    scl_desc = gpio_to_desc(scl_pin);
    if (!scl_desc) {
        ret = -EINVAL;
        goto err_free_scl;
    }
    gpiod_direction_input(scl_desc);


    // --- I2C ����� ��� ---
    ret = i2c_add_adapter(&i2c_sw_adapter);
    if (ret < 0) {
        printk(KERN_ERR "i2c_sw_driver: Failed to add I2C adapter\n");
        goto err_free_scl;
    }

    printk(KERN_INFO "i2c_sw_driver: Adapter registered successfully as i2c-%d\n", i2c_sw_adapter.nr);
    return 0;

err_free_scl:
    gpio_free(scl_pin);
err_free_sda:
    gpio_free(sda_pin);
    return ret;
}

static void __exit i2c_sw_exit(void)
{
    i2c_del_adapter(&i2c_sw_adapter);
    gpio_free(sda_pin);
    gpio_free(scl_pin);
    printk(KERN_INFO "i2c_sw_driver: Module unloaded.\n");
}

module_init(i2c_sw_init);
module_exit(i2c_sw_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("Software I2C (bit-banging) driver with high compatibility");