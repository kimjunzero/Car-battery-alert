// eeprom_vs1003_bitbang_nodt.c
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/string.h>

/* ===== 드라이버/디바이스 공통 ===== */
#define DRIVER_NAME     "eeprom_vs1003_bb"
#define DEVICE_NAME     "eeprom_vs1003"
#define CLASS_NAME      "eepromvs"

/* ===== VS1003 레지스터 (내장 정의) ===== */
#define VS1003_REG_MODE     0x00
#define VS1003_REG_STATUS   0x01
#define VS1003_REG_BASS     0x02
#define VS1003_REG_CLOCKF   0x03
#define VS1003_REG_DECODE_TIME 0x04
#define VS1003_REG_AUDATA   0x05
#define VS1003_REG_WRAM     0x06
#define VS1003_REG_WRAMADDR 0x07
#define VS1003_REG_HDAT0    0x08
#define VS1003_REG_HDAT1    0x09
#define VS1003_REG_AIADDR   0x0A
#define VS1003_REG_VOL      0x0B

/* ===== 사용자 설정 ===== */
/* 비트뱅 SPI GPIO (너의 환경과 동일하게 500번대 전역 GPIO 번호 사용) */
#define PIN_DREQ    528 /* BCM 16 */
#define PIN_XCS     532 /* BCM 20 */
#define PIN_XDCS    533 /* BCM 21 */
#define PIN_SCLK    538 /* BCM 26 */
#define PIN_MOSI    531 /* BCM 19 */
#define PIN_MISO    525 /* BCM 13  */
#define PIN_XRST    518     //6

/* I2C(K24C256) */
#define I2C_BUS_NUM         22
#define K24C256_ADDR        0x58   /* 필요시 0x50 ~ 0x57로 변경 */
#define EEPROM_DATA_ADDR    0x0000 /* 모니터링 주소 */

/* 동작 파라미터 */
#define ALARM_THRESHOLD     20
#define MONITORING_INTERVAL 1000  /* ms */
#define BB_DELAY_US         1     /* 비트뱅 SPI 딜레이 */

/* ===== 내부 상태 구조체 ===== */
struct alarm_system {
    /* VS1003 비트뱅 SPI GPIO */
    int gpio_dreq, gpio_xcs, gpio_xdcs;
    int gpio_sclk, gpio_mosi, gpio_miso;
    int irq_dreq;

    /* I2C */
    struct i2c_client* i2c_client;

    /* 동기화 */
    struct mutex      lock;
    wait_queue_head_t dreq_wait;
    bool              dreq_ready;

    /* 상태 */
    bool              alarm_active;
    bool              system_enabled;
    bool              test_mode;
    int               test_value;
    int               custom_threshold;

    /* 작업/타이머 */
    struct delayed_work monitor_work;
    struct work_struct alarm_work;
    struct workqueue_struct* alarm_wq;

    /* 문자 디바이스 */
    dev_t             devt;
    struct cdev       cdev;
    struct class* class;
    struct device* device;

    /* 통계 */
    int last_eeprom_value;
    int alarm_count;
    int siren_repeat_count;
};

static struct alarm_system* gdev;

/* ===== 비트뱅 SPI 기본 ===== */
static inline void clk_hi(struct alarm_system* v) { gpio_set_value(v->gpio_sclk, 1); }
static inline void clk_lo(struct alarm_system* v) { gpio_set_value(v->gpio_sclk, 0); }

static u8 bb_txrx_byte(struct alarm_system* v, u8 out)
{
    u8 in = 0;
    int i;
    for (i = 7; i >= 0; i--) {
        gpio_set_value(v->gpio_mosi, (out >> i) & 1);
        udelay(BB_DELAY_US);
        clk_hi(v);
        udelay(BB_DELAY_US);
        in = (in << 1) | gpio_get_value(v->gpio_miso);
        clk_lo(v);
        udelay(BB_DELAY_US);
    }
    return in;
}

static int bb_transfer(struct alarm_system* v, const u8* tx, u8* rx, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        u8 r = bb_txrx_byte(v, tx ? tx[i] : 0xFF);
        if (rx) rx[i] = r;
    }
    return 0;
}

/* ===== DREQ 대기/IRQ ===== */
static irqreturn_t dreq_irq_handler(int irq, void* data)
{
    struct alarm_system* v = data;
    v->dreq_ready = true;
    wake_up_interruptible(&v->dreq_wait);
    return IRQ_HANDLED;
}

static int wait_dreq(struct alarm_system* v)
{
    if (gpio_get_value(v->gpio_dreq))
        return 0;

    v->dreq_ready = false;
    if (!wait_event_interruptible_timeout(v->dreq_wait, v->dreq_ready,
        msecs_to_jiffies(1000)))
        return -ETIMEDOUT;
    return 0;
}

/* ===== VS1003 SCI/SDI ===== */
static u16 sci_read(struct alarm_system* v, u8 reg)
{
    u8 tx[4] = { 0x03, reg, 0, 0 }, rx[4] = { 0 };
    gpio_set_value(v->gpio_xcs, 0);
    if (!wait_dreq(v))
        bb_transfer(v, tx, rx, 4);
    gpio_set_value(v->gpio_xcs, 1);
    return ((u16)rx[2] << 8) | rx[3];
}

static int sci_write(struct alarm_system* v, u8 reg, u16 val)
{
    u8 tx[4] = { 0x02, reg, (u8)(val >> 8), (u8)val };
    gpio_set_value(v->gpio_xcs, 0);
    if (!wait_dreq(v))
        bb_transfer(v, tx, NULL, 4);
    gpio_set_value(v->gpio_xcs, 1);
    return 0;
}

static int sdi_write(struct alarm_system* v, const u8* buf, size_t len)
{
    int ret = wait_dreq(v);
    if (ret) return ret;
    gpio_set_value(v->gpio_xdcs, 0);
    bb_transfer(v, buf, NULL, len);
    gpio_set_value(v->gpio_xdcs, 1);
    return 0;
}

/* ===== VS1003 유틸 ===== */
static int vs1003_hw_reset(struct alarm_system* v)
{
    /* XRESET 핀이 없다면 SCI로만 초기화: 여기선 DREQ ready만 기다림 */
    int ret = wait_dreq(v);
    if (ret) return ret;

    /* 기본 세팅: 클럭/볼륨 */
    sci_write(v, VS1003_REG_MODE, 0x0800);
    sci_write(v, VS1003_REG_CLOCKF, 0x6000); /* 클럭업 */
    sci_write(v, VS1003_REG_VOL, 0x0000);    /* 볼륨 적당히 */

    return 0;
}

static int vs1003_play_sine(struct alarm_system* v, u8 freq_byte, int duration_ms)
{
    u8 sine_on[8] = { 0x53,0xEF,0x6E,freq_byte,0x00,0x00,0x00,0x00 };
    u8 sine_off[8] = { 0x45,0x78,0x69,0x74,      0x00,0x00,0x00,0x00 };
    int ret;

    sci_write(v, VS1003_REG_MODE, 0x0820); /* SM_TESTS */
    ret = sdi_write(v, sine_on, sizeof(sine_on));
    if (ret) return ret;

    msleep(duration_ms);

    ret = sdi_write(v, sine_off, sizeof(sine_off));
    sci_write(v, VS1003_REG_MODE, 0x0800);
    return ret;
}

/* 간단한 올라갔다 내려오는 사이렌 */
static int siren_test(struct alarm_system* v, int duration_ms)
{
    u8 stop[8] = { 0x45,0x78,0x69,0x74,0,0,0,0 };
    int freq_low = 0x30, freq_high = 0x80, step = 2, delay_ms = 50;
    int cur = freq_low, dir = 1, t = 0;

    sci_write(v, VS1003_REG_MODE, 0x0820);

    while (t < duration_ms && v->alarm_active && v->system_enabled) {
        u8 start[8] = { 0x53,0xEF,0x6E,(u8)cur,0,0,0,0 };
        gpio_set_value(v->gpio_xdcs, 0);
        if (!wait_dreq(v)) bb_transfer(v, start, NULL, 8);
        gpio_set_value(v->gpio_xdcs, 1);

        msleep(delay_ms);

        gpio_set_value(v->gpio_xdcs, 0);
        if (!wait_dreq(v)) bb_transfer(v, stop, NULL, 8);
        gpio_set_value(v->gpio_xdcs, 1);

        cur += dir * step;
        if (cur >= freq_high) { cur = freq_high; dir = -1; }
        else if (cur <= freq_low) { cur = freq_low; dir = 1; }

        t += delay_ms;
    }

    sci_write(v, VS1003_REG_MODE, 0x0800);
    return 0;
}

/* ===== I2C EEPROM (K24C256) ===== */
static int eeprom_read_byte(struct alarm_system* dev, u16 addr, u8* data)
{
    struct i2c_msg msgs[2];
    u8 addr_buf[2];
    int ret;

    if (!dev->i2c_client) return -ENODEV;

    /* 15-bit address: 상위 7비트만 사용 */
    addr_buf[0] = (addr >> 8) & 0x7F;
    addr_buf[1] = addr & 0xFF;

    msgs[0].addr = dev->i2c_client->addr;
    msgs[0].flags = 0;
    msgs[0].len = 2;
    msgs[0].buf = addr_buf;

    msgs[1].addr = dev->i2c_client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = 1;
    msgs[1].buf = data;

    ret = i2c_transfer(dev->i2c_client->adapter, msgs, 2);
    return (ret == 2) ? 0 : -EIO;
}

static int eeprom_write_byte(struct alarm_system* dev, u16 addr, u8 val)
{
    u8 buf[3];

    if (!dev->i2c_client) return -ENODEV;

    buf[0] = (addr >> 8) & 0x7F;
    buf[1] = addr & 0xFF;
    buf[2] = val;

    if (i2c_master_send(dev->i2c_client, buf, sizeof(buf)) != sizeof(buf))
        return -EIO;

    msleep(5); /* write cycle */
    return 0;
}

/* ===== 알람/모니터링 ===== */
static void play_siren_sequence(struct alarm_system* dev)
{
    int i;
    for (i = 0; i < dev->siren_repeat_count; i++) {
        if (!dev->alarm_active || !dev->system_enabled) break;
        siren_test(dev, 3000);
        if (i < dev->siren_repeat_count - 1) msleep(1000);
    }
}

static void alarm_work_handler(struct work_struct* work)
{
    struct alarm_system* dev = container_of(work, struct alarm_system, alarm_work);
    mutex_lock(&dev->lock);
    if (dev->alarm_active && dev->system_enabled) {
        play_siren_sequence(dev);
        dev->alarm_count++;
    }
    mutex_unlock(&dev->lock);
}

// 새 콜백
static void monitor_work_fn(struct work_struct* w)
{
    struct alarm_system* dev = container_of(to_delayed_work(w),
        struct alarm_system, monitor_work);
    u8 val = 0xFF;
    int cur, th, ret;

    if (!dev->system_enabled) {
        queue_delayed_work(dev->alarm_wq, &dev->monitor_work,
            msecs_to_jiffies(MONITORING_INTERVAL));
        return;
    }

    if (dev->test_mode) {
        cur = dev->test_value;
    }
    else {
        ret = eeprom_read_byte(dev, EEPROM_DATA_ADDR, &val);
        cur = (ret ? 255 : val); //성공시 0을 반환
        pr_info("EEPROM value: %d%s\n", cur, ret ? " (read fail)" : "");
    }

    dev->last_eeprom_value = cur;
    th = (dev->custom_threshold > 0) ? dev->custom_threshold : ALARM_THRESHOLD;

    if (cur <= th) {
        if (!dev->alarm_active) {
            dev->alarm_active = true;
            queue_work(dev->alarm_wq, &dev->alarm_work);
        }
    }
    else {
        dev->alarm_active = false;
    }

    // 다음 주기 예약
    queue_delayed_work(dev->alarm_wq, &dev->monitor_work,
        msecs_to_jiffies(MONITORING_INTERVAL));
}


/* ===== 문자 디바이스 ===== */
static int alarm_chr_open(struct inode* ino, struct file* filp)
{
    filp->private_data = gdev;
    return 0;
}

static ssize_t alarm_chr_read(struct file* filp, char __user* buf, size_t count, loff_t* off)
{
    struct alarm_system* dev = filp->private_data;
    char out[512];
    int th = (dev->custom_threshold > 0) ? dev->custom_threshold : ALARM_THRESHOLD;
    int len = scnprintf(out, sizeof(out),
        "=== VS1003 BitBang + K24C256 ===\n"
        "system: %s, alarm: %s, mode: %s\n"
        "value: %d, threshold: %d, repeats: %d, alarms: %d\n",
        dev->system_enabled ? "on" : "off",
        dev->alarm_active ? "on" : "off",
        dev->test_mode ? "test" : "eeprom",
        dev->last_eeprom_value, th, dev->siren_repeat_count, dev->alarm_count);

    return simple_read_from_buffer(buf, count, off, out, len);
}

static ssize_t alarm_chr_write(struct file* filp, const char __user* buf, size_t count, loff_t* off)
{
    struct alarm_system* dev = filp->private_data;
    char cmd[64];
    int v, a;

    if (count >= sizeof(cmd)) return -EINVAL;
    if (copy_from_user(cmd, buf, count)) return -EFAULT;
    cmd[count] = '\0';
    if (count && cmd[count - 1] == '\n') cmd[count - 1] = '\0';

    mutex_lock(&dev->lock);

    if (!strcmp(cmd, "enable")) {
        dev->system_enabled = true;
    }
    else if (!strcmp(cmd, "disable")) {
        dev->system_enabled = false;
        dev->alarm_active = false;
    }
    else if (!strcmp(cmd, "test")) {
        dev->alarm_active = true;
        queue_work(dev->alarm_wq, &dev->alarm_work);
    }
    else if (!strcmp(cmd, "testmode on")) {
        dev->test_mode = true;
        if (dev->test_value == 0) dev->test_value = 25;
    }
    else if (!strcmp(cmd, "testmode off")) {
        dev->test_mode = false;
    }
    else if (sscanf(cmd, "setvalue %d", &v) == 1) {
        if (v >= 0 && v <= 255) dev->test_value = v;
    }
    else if (sscanf(cmd, "threshold %d", &v) == 1) {
        if (v >= 0 && v <= 255) dev->custom_threshold = v;
    }
    else if (sscanf(cmd, "repeat %d", &v) == 1) {
        if (v >= 1 && v <= 10) dev->siren_repeat_count = v;
    }
    else if (sscanf(cmd, "writeeprom %d %d", &a, &v) == 2) {
        if (a >= 0 && a <= 32767 && v >= 0 && v <= 255)
            eeprom_write_byte(dev, (u16)a, (u8)v);
    }

    mutex_unlock(&dev->lock);
    return count;
}

static const struct file_operations alarm_fops = {
    .owner = THIS_MODULE,
    .open = alarm_chr_open,
    .read = alarm_chr_read,
    .write = alarm_chr_write,
};

/* ===== I2C 드라이버(선택적) — 있으면 probe에서 상태 찍음 ===== */
static int eeprom_i2c_probe(struct i2c_client* client)
{
    pr_info("k24c256 bound (addr=0x%02x)\n", client->addr);
    return 0;
}
static void eeprom_i2c_remove(struct i2c_client* client)
{
    pr_info("k24c256 unbound\n");
}
static const struct i2c_device_id eeprom_i2c_id[] = {
    { "k24c256", 0 }, { }
};
MODULE_DEVICE_TABLE(i2c, eeprom_i2c_id);
static struct i2c_driver eeprom_i2c_driver = {
    .driver = {.name = "k24c256" },
    .probe = eeprom_i2c_probe,
    .remove = eeprom_i2c_remove,
    .id_table = eeprom_i2c_id,
};

/* ===== 모듈 init/exit ===== */
static int __init eeprom_vs1003_init(void)
{
    int ret;
    struct i2c_adapter* adap;
    struct i2c_board_info bi = { };
    struct alarm_system* dev;

    pr_info(DRIVER_NAME ": init (bitbang SPI + K24C256)\n");

    dev = kzalloc(sizeof(*dev), GFP_KERNEL);
    if (!dev) return -ENOMEM;
    gdev = dev;

    mutex_init(&dev->lock);
    init_waitqueue_head(&dev->dreq_wait);

    dev->system_enabled = true;
    dev->alarm_active = false;
    dev->test_mode = false;
    dev->test_value = 25;
    dev->custom_threshold = 0;
    dev->last_eeprom_value = 255;
    dev->alarm_count = 0;
    dev->siren_repeat_count = 3;

    /* GPIO 요청 */
    if ((ret = gpio_request_one(PIN_DREQ, GPIOF_IN, "vs_dreq"))) goto err_gpio;
    if ((ret = gpio_request_one(PIN_XCS, GPIOF_OUT_INIT_HIGH, "vs_xcs")))  goto err_gpio;
    if ((ret = gpio_request_one(PIN_XDCS, GPIOF_OUT_INIT_HIGH, "vs_xdcs"))) goto err_gpio;
    if ((ret = gpio_request_one(PIN_SCLK, GPIOF_OUT_INIT_LOW, "vs_sclk"))) goto err_gpio;
    if ((ret = gpio_request_one(PIN_MOSI, GPIOF_OUT_INIT_LOW, "vs_mosi"))) goto err_gpio;
    if ((ret = gpio_request_one(PIN_MISO, GPIOF_IN, "vs_miso"))) goto err_gpio;

    dev->gpio_dreq = PIN_DREQ; dev->gpio_xcs = PIN_XCS; dev->gpio_xdcs = PIN_XDCS;
    dev->gpio_sclk = PIN_SCLK; dev->gpio_mosi = PIN_MOSI; dev->gpio_miso = PIN_MISO;

    /* DREQ IRQ */
    dev->irq_dreq = gpio_to_irq(dev->gpio_dreq);
    if (dev->irq_dreq < 0) { ret = dev->irq_dreq; goto err_gpio; }
    ret = request_irq(dev->irq_dreq, dreq_irq_handler, IRQF_TRIGGER_RISING,
        "vs1003-dreq", dev);
    if (ret) goto err_gpio;

    /* 워크큐/타이머 */
    dev->alarm_wq = create_singlethread_workqueue("vs_alarm_wq");
    if (!dev->alarm_wq) { ret = -ENOMEM; goto err_irq; }
    INIT_WORK(&dev->alarm_work, alarm_work_handler);
    INIT_DELAYED_WORK(&dev->monitor_work, monitor_work_fn);

    /* 문자 디바이스 */
    if ((ret = alloc_chrdev_region(&dev->devt, 0, 1, DEVICE_NAME))) goto err_wq;
    cdev_init(&dev->cdev, &alarm_fops);
    dev->cdev.owner = THIS_MODULE;
    if ((ret = cdev_add(&dev->cdev, dev->devt, 1))) goto err_chr;
    dev->class = class_create(CLASS_NAME);
    if (IS_ERR(dev->class)) { ret = PTR_ERR(dev->class); goto err_chr; }
    dev->device = device_create(dev->class, NULL, dev->devt, NULL, DEVICE_NAME);
    if (IS_ERR(dev->device)) { ret = PTR_ERR(dev->device); goto err_class; }

    /* I2C 클라이언트 생성 */
    adap = i2c_get_adapter(I2C_BUS_NUM);
    if (!adap) { pr_err("i2c-%d not found\n", I2C_BUS_NUM); ret = -ENODEV; goto err_class; }
    memset(&bi, 0, sizeof(bi));
    strscpy(bi.type, "k24c256", sizeof(bi.type));
    bi.addr = K24C256_ADDR;
    dev->i2c_client = i2c_new_client_device(adap, &bi);
    i2c_put_adapter(adap);
    if (IS_ERR(dev->i2c_client)) {
        pr_warn("i2c_new_client_device failed: %ld (EEPROM 없이 테스트 모드 사용 가능)\n",
            PTR_ERR(dev->i2c_client));
        dev->i2c_client = NULL;
    }

    /* (선택) I2C 드라이버 등록: 단순 상태 메시지용 */
    i2c_add_driver(&eeprom_i2c_driver);

    /* VS1003 초기화(하드리셋 핀이 없다면 DREQ 대기 + 기본세팅) */
    msleep(100);
    vs1003_hw_reset(dev);

    /* 모니터링 시작 */
    queue_delayed_work(dev->alarm_wq, &dev->monitor_work,
        msecs_to_jiffies(MONITORING_INTERVAL));

    pr_info("ready: /dev/%s (I2C=0x%02x, bitbang SPI)\n", DEVICE_NAME, K24C256_ADDR);
    return 0;

err_class:
    class_destroy(dev->class);
err_chr:
    cdev_del(&dev->cdev);
    unregister_chrdev_region(dev->devt, 1);
err_wq:
    destroy_workqueue(dev->alarm_wq);
err_irq:
    free_irq(dev->irq_dreq, dev);
err_gpio:
    gpio_free(PIN_MISO); gpio_free(PIN_MOSI); gpio_free(PIN_SCLK);
    gpio_free(PIN_XDCS); gpio_free(PIN_XCS);  gpio_free(PIN_DREQ);
    kfree(dev);
    gdev = NULL;
    return ret;
}

static void __exit eeprom_vs1003_exit(void)
{
    struct alarm_system* dev = gdev;
    if (!dev) return;

    cancel_delayed_work_sync(&dev->monitor_work);
    cancel_work_sync(&dev->alarm_work);
    if (dev->alarm_wq) destroy_workqueue(dev->alarm_wq);

    if (dev->i2c_client) i2c_unregister_device(dev->i2c_client);
    i2c_del_driver(&eeprom_i2c_driver);

    device_destroy(dev->class, dev->devt);
    class_destroy(dev->class);
    cdev_del(&dev->cdev);
    unregister_chrdev_region(dev->devt, 1);

    free_irq(dev->irq_dreq, dev);
    gpio_free(PIN_MISO); gpio_free(PIN_MOSI); gpio_free(PIN_SCLK);
    gpio_free(PIN_XDCS); gpio_free(PIN_XCS);  gpio_free(PIN_DREQ);

    kfree(dev);
    gdev = NULL;
    pr_info(DRIVER_NAME ": exit\n");
}

module_init(eeprom_vs1003_init);
module_exit(eeprom_vs1003_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("my");
MODULE_DESCRIPTION("K24C256 monitor + VS1003 siren (bit-banged SPI, no DT)");
MODULE_VERSION("1.0");
