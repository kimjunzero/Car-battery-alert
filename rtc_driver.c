#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>

#define DEVICE_NAME "my_rtc"
#define CLASS_NAME  "rtc_class"

// BCD <-> DEC 변환 함수
static int bcd2dec(u8 bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0f);
}
static u8 dec2bcd(int dec) {
    return ((dec / 10) << 4) | (dec % 10);
}

// 드라이버 데이터 구조체
struct rtc_drv_data {
    struct i2c_client* client;
    struct cdev c_dev;
    dev_t dev_num;
    struct class* cl;
};

// 파일 오퍼레이션 함수들
static int my_open(struct inode* i, struct file* f) {
    // 드라이버 데이터를 file 구조체의 private_data에 저장
    struct rtc_drv_data* drv_data = container_of(i->i_cdev, struct rtc_drv_data, c_dev);
    f->private_data = drv_data;
    return 0;
}

static int my_close(struct inode* i, struct file* f) {
    return 0;
}

// read 함수: /dev/my_rtc를 읽으면 현재 시간을 반환
static ssize_t my_read(struct file* f, char __user* buf, size_t len, loff_t* off) {
    struct rtc_drv_data* drv_data = f->private_data;
    struct i2c_client* client = drv_data->client;
    u8 rtc_data[7];
    char time_buf[20];
    int ret;

    // I2C를 통해 DS1307에서 시간 데이터 7바이트를 읽음
    ret = i2c_smbus_read_i2c_block_data(client, 0x00, 7, rtc_data);
    if (ret < 0) {
        pr_err("Failed to read from I2C client\n");
        return ret;
    }

    // "YYYY-MM-DD HH:MM:SS" 형식으로 포맷
    sprintf(time_buf, "%04d-%02d-%02d %02d:%02d:%02d\n",
        bcd2dec(rtc_data[6]) + 2000,
        bcd2dec(rtc_data[5]),
        bcd2dec(rtc_data[4]),
        bcd2dec(rtc_data[2] & 0x3F),
        bcd2dec(rtc_data[1]),
        bcd2dec(rtc_data[0] & 0x7F));

    // 커널 공간의 데이터를 사용자 공간으로 복사
    if (copy_to_user(buf, time_buf, strlen(time_buf))) {
        return -EFAULT;
    }

    return strlen(time_buf);
}

// 파일 오퍼레이션 구조체 정의
static const struct file_operations my_fops = {
    .owner = THIS_MODULE,
    .open = my_open,
    .release = my_close,
    .read = my_read,
    // write 함수는 생략 (필요 시 유사하게 구현 가능)
};

// I2C 장치가 발견되었을 때 호출되는 함수 (probe)
static int rtc_probe(struct i2c_client* client, const struct i2c_device_id* id) {
    struct rtc_drv_data* drv_data;
    int ret;

    pr_info("RTC probe function called for device: %s\n", client->name);

    // 1. 드라이버 데이터 메모리 할당
    drv_data = devm_kzalloc(&client->dev, sizeof(struct rtc_drv_data), GFP_KERNEL);
    if (!drv_data) return -ENOMEM;

    drv_data->client = client;
    i2c_set_clientdata(client, drv_data);

    // 2. 문자 디바이스 번호 할당
    ret = alloc_chrdev_region(&drv_data->dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        pr_err("Failed to allocate major number\n");
        return ret;
    }
    pr_info("Major = %d Minor = %d\n", MAJOR(drv_data->dev_num), MINOR(drv_data->dev_num));

    // 3. 디바이스 클래스 생성
    drv_data->cl = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(drv_data->cl)) {
        unregister_chrdev_region(drv_data->dev_num, 1);
        return PTR_ERR(drv_data->cl);
    }

    // 4. cdev 구조체 초기화 및 커널에 등록
    cdev_init(&drv_data->c_dev, &my_fops);
    ret = cdev_add(&drv_data->c_dev, drv_data->dev_num, 1);
    if (ret < 0) {
        class_destroy(drv_data->cl);
        unregister_chrdev_region(drv_data->dev_num, 1);
        return ret;
    }

    // 5. /dev/my_rtc 장치 파일 생성
    if (IS_ERR(device_create(drv_data->cl, NULL, drv_data->dev_num, NULL, DEVICE_NAME))) {
        cdev_del(&drv_data->c_dev);
        class_destroy(drv_data->cl);
        unregister_chrdev_region(drv_data->dev_num, 1);
        return -1;
    }

    pr_info("Device Driver Inserted... Done.\n");
    return 0;
}

// I2C 장치가 제거될 때 호출되는 함수 (remove)
static int rtc_remove(struct i2c_client* client) {
    struct rtc_drv_data* drv_data = i2c_get_clientdata(client);

    device_destroy(drv_data->cl, drv_data->dev_num);
    class_destroy(drv_data->cl);
    cdev_del(&drv_data->c_dev);
    unregister_chrdev_region(drv_data->dev_num, 1);
    pr_info("Device Driver Removed... Done.\n");
    return 0;
}

// 지원할 I2C 장치 목록
static const struct i2c_device_id rtc_id[] = {
    { "ds1307", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, rtc_id);

// I2C 드라이버 구조체
static struct i2c_driver rtc_driver = {
    .driver = {
        .name = "my_rtc_driver",
        .owner = THIS_MODULE,
    },
    .probe = rtc_probe,
    .remove = rtc_remove,
    .id_table = rtc_id,
};

// I2C 드라이버를 시스템에 등록/해제하는 대신, 모듈 로드/언로드 시점에 직접 등록
// 이는 I2C 장치를 동적으로 연결/해제하지 않는 임베디드 환경에서 더 간단할 수 있음
// 하지만 표준적인 방법은 i2c_add_driver/i2c_del_driver를 사용하는 것
module_i2c_driver(rtc_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("A simple I2C RTC driver for DS1307");