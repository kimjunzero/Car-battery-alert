#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

// --- Display Constants ---
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define SCREEN_PAGES    (SCREEN_HEIGHT / 8)
#define FRAMEBUFFER_SIZE (SCREEN_WIDTH * SCREEN_PAGES) // 128 * 8 = 1024 bytes

// --- IOCTL Command Definition ---
#define SSD1306_IOCTL_MAGIC 'o'
// Command to write the full framebuffer from user space to the kernel
#define SSD1306_DISPLAY_FULL_BUFFER _IOW(SSD1306_IOCTL_MAGIC, 1, unsigned char *)

// --- SSD1306 I2C and Command Set ---
#define SSD1306_CONTROL_BYTE_CMD_STREAM  0x00
#define SSD1306_CONTROL_BYTE_DATA_STREAM 0x40
#define SSD1306_DISPLAY_OFF             0xAE
#define SSD1306_DISPLAY_ON              0xAF
#define SSD1306_SET_DISPLAY_CLOCK_DIV   0xD5
#define SSD1306_SET_MULTIPLEX_RATIO     0xA8
#define SSD1306_SET_DISPLAY_OFFSET      0xD3
#define SSD1306_SET_DISPLAY_START_LINE  0x40
#define SSD1306_SET_CHARGE_PUMP         0x8D
#define SSD1306_MEMORY_ADDR_MODE        0x20
#define SSD1306_SET_SEGMENT_REMAP       0xA0
#define SSD1306_SET_COM_SCAN_DIR_INC    0xC0
#define SSD1306_SET_COM_PINS_CONFIG     0xDA
#define SSD1306_SET_CONTRAST_CONTROL    0x81
#define SSD1306_SET_PRECHARGE_PERIOD    0xD9
#define SSD1306_SET_VCOMH_DESELECT      0xDB
#define SSD1306_DISPLAY_ALL_ON_RESUME   0xA4
#define SSD1306_SET_NORMAL_DISPLAY      0xA6
#define SSD1306_SET_COLUMN_ADDR         0x21
#define SSD1306_SET_PAGE_ADDR           0x22

// --- Global Variables ---
static struct i2c_client* ssd1306_client;
static u8 framebuffer[FRAMEBUFFER_SIZE]; // 1KB screen buffer in kernel space

// --- Low-level I2C Functions ---
static int ssd1306_write_cmds(struct i2c_client* client, const u8* cmds, size_t len) {
    u8* buf = kmalloc(len + 1, GFP_KERNEL);
    if (!buf) return -ENOMEM;
    buf[0] = SSD1306_CONTROL_BYTE_CMD_STREAM;
    memcpy(&buf[1], cmds, len);
    int ret = i2c_master_send(client, buf, len + 1);
    kfree(buf);
    return (ret == len + 1) ? 0 : -EIO;
}

static int ssd1306_write_data(struct i2c_client* client, const u8* data, size_t len) {
    u8* buf = kmalloc(len + 1, GFP_KERNEL);
    if (!buf) return -ENOMEM;
    buf[0] = SSD1306_CONTROL_BYTE_DATA_STREAM;
    memcpy(&buf[1], data, len);
    int ret = i2c_master_send(client, buf, len + 1);
    kfree(buf);
    return (ret == len + 1) ? 0 : -EIO;
}

// --- Display Control Functions ---
static void ssd1306_set_window(struct i2c_client* client) {
    const u8 cmds[] = {
        SSD1306_SET_COLUMN_ADDR, 0, SCREEN_WIDTH - 1,
        SSD1306_SET_PAGE_ADDR, 0, SCREEN_PAGES - 1
    };
    ssd1306_write_cmds(client, cmds, sizeof(cmds));
}

static void ssd1306_update_display(struct i2c_client* client) {
    ssd1306_set_window(client);
    ssd1306_write_data(client, framebuffer, FRAMEBUFFER_SIZE);
}

// --- File Operations & IOCTL Handler ---
static int ssd1306_open(struct inode* inode, struct file* file) {
    pr_info("SSD1306: Device opened.\n");
    return 0;
}

static int ssd1306_release(struct inode* inode, struct file* file) {
    pr_info("SSD1306: Device closed.\n");
    return 0;
}

static long ssd1306_ioctl(struct file* file, unsigned int cmd, unsigned long arg) {
    switch (cmd) {
    case SSD1306_DISPLAY_FULL_BUFFER:
        if (copy_from_user(framebuffer, (unsigned char __user*)arg, FRAMEBUFFER_SIZE)) {
            pr_err("SSD1306: Failed to copy framebuffer from user space\n");
            return -EFAULT;
        }
        ssd1306_update_display(ssd1306_client);
        break;
    default:
        pr_warn("SSD1306: Unsupported ioctl command\n");
        return -EINVAL;
    }
    return 0;
}

static const struct file_operations ssd1306_fops = {
    .owner = THIS_MODULE,
    .open = ssd1306_open,
    .release = ssd1306_release,
    .unlocked_ioctl = ssd1306_ioctl,
};

// --- Misc Device & I2C Driver Structures ---
static struct miscdevice ssd1306_misc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "ssd1306_oled",
    .fops = &ssd1306_fops,
};

static void ssd1306_init_display(struct i2c_client* client) {
    const u8 init_cmds[] = {
        SSD1306_DISPLAY_OFF, SSD1306_SET_DISPLAY_CLOCK_DIV, 0x80,
        SSD1306_SET_MULTIPLEX_RATIO, 0x3F, SSD1306_SET_DISPLAY_OFFSET, 0x00,
        SSD1306_SET_DISPLAY_START_LINE | 0x0, SSD1306_SET_CHARGE_PUMP, 0x14,
        SSD1306_MEMORY_ADDR_MODE, 0x00, SSD1306_SET_SEGMENT_REMAP | 0x1,
        SSD1306_SET_COM_SCAN_DIR_INC | 0x8, SSD1306_SET_COM_PINS_CONFIG, 0x12,
        SSD1306_SET_CONTRAST_CONTROL, 0xCF, SSD1306_SET_PRECHARGE_PERIOD, 0xF1,
        SSD1306_SET_VCOMH_DESELECT, 0x40, SSD1306_DISPLAY_ALL_ON_RESUME,
        SSD1306_SET_NORMAL_DISPLAY, SSD1306_DISPLAY_ON
    };
    ssd1306_write_cmds(client, init_cmds, sizeof(init_cmds));
    memset(framebuffer, 0x00, FRAMEBUFFER_SIZE); // Clear buffer
    ssd1306_update_display(client); // Push clear buffer to screen
    pr_info("SSD1306: Display initialized and ON.\n");
}

static int ssd1306_probe(struct i2c_client* client) {
    int ret;
    pr_info("SSD1306: Probing for device.\n");

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        pr_err("SSD1306: I2C adapter does not support I2C transactions.\n");
        return -EIO;
    }
    ssd1306_client = client;
    ssd1306_init_display(client);

    ret = misc_register(&ssd1306_misc_device);
    if (ret) {
        pr_err("SSD1306: Failed to register misc device\n");
        return ret;
    }

    pr_info("SSD1306: Driver loaded. Device /dev/%s is ready.\n", ssd1306_misc_device.name);
    return 0;
}

static void ssd1306_remove(struct i2c_client* client) {
    const u8 cmd[] = { SSD1306_DISPLAY_OFF };
    ssd1306_write_cmds(client, cmd, sizeof(cmd));
    misc_deregister(&ssd1306_misc_device);
    pr_info("SSD1306: Driver removed.\n");
}

static const struct i2c_device_id ssd1306_id[] = {
    { "ssd1306", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, ssd1306_id);

static struct i2c_driver ssd1306_driver = {
    .driver = {.name = "ssd1306_oled", .owner = THIS_MODULE },
    .probe = ssd1306_probe,
    .remove = ssd1306_remove,
    .id_table = ssd1306_id,
};

module_i2c_driver(ssd1306_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("SSD1306 OLED I2C framebuffer driver");
~