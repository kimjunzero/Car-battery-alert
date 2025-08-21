#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>

#define DEVICE_NAME "my_rtc"
#define CLASS_NAME  "rtc_class"

// BCD <-> DEC ��ȯ �Լ�
static int bcd2dec(u8 bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0f);
}
static u8 dec2bcd(int dec) {
    return ((dec / 10) << 4) | (dec % 10);
}

// ����̹� ������ ����ü
struct rtc_drv_data {
    struct i2c_client* client;
    struct cdev c_dev;
    dev_t dev_num;
    struct class* cl;
};

// ���� ���۷��̼� �Լ���
static int my_open(struct inode* i, struct file* f) {
    // ����̹� �����͸� file ����ü�� private_data�� ����
    struct rtc_drv_data* drv_data = container_of(i->i_cdev, struct rtc_drv_data, c_dev);
    f->private_data = drv_data;
    return 0;
}

static int my_close(struct inode* i, struct file* f) {
    return 0;
}

// read �Լ�: /dev/my_rtc�� ������ ���� �ð��� ��ȯ
static ssize_t my_read(struct file* f, char __user* buf, size_t len, loff_t* off) {
    struct rtc_drv_data* drv_data = f->private_data;
    struct i2c_client* client = drv_data->client;
    u8 rtc_data[7];
    char time_buf[20];
    int ret;

    // I2C�� ���� DS1307���� �ð� ������ 7����Ʈ�� ����
    ret = i2c_smbus_read_i2c_block_data(client, 0x00, 7, rtc_data);
    if (ret < 0) {
        pr_err("Failed to read from I2C client\n");
        return ret;
    }

    // "YYYY-MM-DD HH:MM:SS" �������� ����
    sprintf(time_buf, "%04d-%02d-%02d %02d:%02d:%02d\n",
        bcd2dec(rtc_data[6]) + 2000,
        bcd2dec(rtc_data[5]),
        bcd2dec(rtc_data[4]),
        bcd2dec(rtc_data[2] & 0x3F),
        bcd2dec(rtc_data[1]),
        bcd2dec(rtc_data[0] & 0x7F));

    // Ŀ�� ������ �����͸� ����� �������� ����
    if (copy_to_user(buf, time_buf, strlen(time_buf))) {
        return -EFAULT;
    }

    return strlen(time_buf);
}

// ���� ���۷��̼� ����ü ����
static const struct file_operations my_fops = {
    .owner = THIS_MODULE,
    .open = my_open,
    .release = my_close,
    .read = my_read,
    // write �Լ��� ���� (�ʿ� �� �����ϰ� ���� ����)
};

// I2C ��ġ�� �߰ߵǾ��� �� ȣ��Ǵ� �Լ� (probe)
static int rtc_probe(struct i2c_client* client, const struct i2c_device_id* id) {
    struct rtc_drv_data* drv_data;
    int ret;

    pr_info("RTC probe function called for device: %s\n", client->name);

    // 1. ����̹� ������ �޸� �Ҵ�
    drv_data = devm_kzalloc(&client->dev, sizeof(struct rtc_drv_data), GFP_KERNEL);
    if (!drv_data) return -ENOMEM;

    drv_data->client = client;
    i2c_set_clientdata(client, drv_data);

    // 2. ���� ����̽� ��ȣ �Ҵ�
    ret = alloc_chrdev_region(&drv_data->dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        pr_err("Failed to allocate major number\n");
        return ret;
    }
    pr_info("Major = %d Minor = %d\n", MAJOR(drv_data->dev_num), MINOR(drv_data->dev_num));

    // 3. ����̽� Ŭ���� ����
    drv_data->cl = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(drv_data->cl)) {
        unregister_chrdev_region(drv_data->dev_num, 1);
        return PTR_ERR(drv_data->cl);
    }

    // 4. cdev ����ü �ʱ�ȭ �� Ŀ�ο� ���
    cdev_init(&drv_data->c_dev, &my_fops);
    ret = cdev_add(&drv_data->c_dev, drv_data->dev_num, 1);
    if (ret < 0) {
        class_destroy(drv_data->cl);
        unregister_chrdev_region(drv_data->dev_num, 1);
        return ret;
    }

    // 5. /dev/my_rtc ��ġ ���� ����
    if (IS_ERR(device_create(drv_data->cl, NULL, drv_data->dev_num, NULL, DEVICE_NAME))) {
        cdev_del(&drv_data->c_dev);
        class_destroy(drv_data->cl);
        unregister_chrdev_region(drv_data->dev_num, 1);
        return -1;
    }

    pr_info("Device Driver Inserted... Done.\n");
    return 0;
}

// I2C ��ġ�� ���ŵ� �� ȣ��Ǵ� �Լ� (remove)
static int rtc_remove(struct i2c_client* client) {
    struct rtc_drv_data* drv_data = i2c_get_clientdata(client);

    device_destroy(drv_data->cl, drv_data->dev_num);
    class_destroy(drv_data->cl);
    cdev_del(&drv_data->c_dev);
    unregister_chrdev_region(drv_data->dev_num, 1);
    pr_info("Device Driver Removed... Done.\n");
    return 0;
}

// ������ I2C ��ġ ���
static const struct i2c_device_id rtc_id[] = {
    { "ds1307", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, rtc_id);

// I2C ����̹� ����ü
static struct i2c_driver rtc_driver = {
    .driver = {
        .name = "my_rtc_driver",
        .owner = THIS_MODULE,
    },
    .probe = rtc_probe,
    .remove = rtc_remove,
    .id_table = rtc_id,
};

// I2C ����̹��� �ý��ۿ� ���/�����ϴ� ���, ��� �ε�/��ε� ������ ���� ���
// �̴� I2C ��ġ�� �������� ����/�������� �ʴ� �Ӻ���� ȯ�濡�� �� ������ �� ����
// ������ ǥ������ ����� i2c_add_driver/i2c_del_driver�� ����ϴ� ��
module_i2c_driver(rtc_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("A simple I2C RTC driver for DS1307");