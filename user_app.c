#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <time.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "oled_app.h"

/* =================================================================
   장치 경로 및 I2C 설정
================================================================= */
#define ROTARY_DEVICE_PATH "/dev/rotary_dev"
#define OLED_DEVICE_PATH   "/dev/ssd1306_oled"
#define I2C_DEVICE_PATH   "/dev/i2c-22"
#define EEPROM_ADDR       0x58
#define TARGET_MEM_ADDR   0x0000

/* =================================================================
   LED GPIO 핀 번호 정의
================================================================= */
static const int led_gpios[] = {
    16 + 512, 20 + 512, 21 + 512, 5 + 512,
    6 + 512, 13 + 512, 19 + 512, 26 + 512
};
#define NUM_LEDS (sizeof(led_gpios)/sizeof(int))

/* =================================================================
   전역 변수
================================================================= */
unsigned char framebuffer[FRAMEBUFFER_SIZE];
static int led_fds[NUM_LEDS];

// 함수 선언
int read_eeprom_byte(unsigned short mem_addr);
void draw_bitmap(const unsigned char* bitmap, int x, int y, int width, int height);
void draw_string(const char* str, int x, int y_page);
int setup_gpios(void);
void cleanup_gpios(void);
void set_leds(unsigned char value);

/* =================================================================
   메인 함수
================================================================= */
int main(void) {
    int rotary_fd, oled_fd;
    char kernel_buf[32];

    if (setup_gpios() != 0) return -1;
    rotary_fd = open(ROTARY_DEVICE_PATH, O_RDONLY);
    if (rotary_fd < 0) { perror("rotary_dev open"); cleanup_gpios(); return -1; }
    oled_fd = open(OLED_DEVICE_PATH, O_RDWR);
    if (oled_fd < 0) { perror("ssd1306_oled open"); close(rotary_fd); cleanup_gpios(); return -1; }

    printf("모든 장치 초기화 성공. 프로그램을 시작합니다.\n");

    unsigned char led_clock_counter = 0;
    time_t last_time_check = 0;
    int previous_count = 0;
    unsigned char last_direction_led_state = 0;

    while (1) {
        int mode = -1, count = -1;

        lseek(rotary_fd, 0, SEEK_SET);
        int bytes_read = read(rotary_fd, kernel_buf, sizeof(kernel_buf) - 1);
        if (bytes_read > 0) {
            kernel_buf[bytes_read] = '\0';
            sscanf(kernel_buf, "MODE=%d,COUNT=%d", &mode, &count);
        }
        else {
            usleep(100000); continue;
        }

        memset(framebuffer, 0x00, FRAMEBUFFER_SIZE);

        if (mode == 1) { // 시간 모드
            time_t raw_time;
            struct tm* time_info;
            time(&raw_time);
            time_info = localtime(&raw_time);

            if (raw_time > last_time_check) {
                last_time_check = raw_time;
                led_clock_counter++;
            }
            set_leds(led_clock_counter);
            last_direction_led_state = 0;

            char date_str[20], time_str[20];
            strftime(date_str, sizeof(date_str), "%Y-%m-%d", time_info);
            strftime(time_str, sizeof(time_str), "%H:%M:%S", time_info);
            draw_string("Current Time", 15, 1);
            draw_string(date_str, 10, 3);
            draw_string(time_str, 20, 5);

        }
        else { // 배터리 모드
            int percentage = read_eeprom_byte(TARGET_MEM_ADDR);

            if (count > previous_count) {
                last_direction_led_state = (1 << 7);
            }
            else if (count < previous_count) {
                last_direction_led_state = (1 << 6);
            }

            int rotary_val = count % 64;
            if (rotary_val < 0) rotary_val += 64;
            unsigned char count_leds = (unsigned char)rotary_val;

            set_leds(count_leds | last_direction_led_state);

            if (percentage < 0) {
                draw_string("EEPROM READ", 10, 2);
                draw_string("FAILED!", 20, 4);
            }
            else {
                if (percentage > 100) percentage = 100;

                // --- ★★★★★ 배터리 그래픽 그리기 로직 (복원됨) ★★★★★ ---
                draw_string("Battery State", 2, 0);

                const unsigned char* center_icon = NULL;
                if (percentage <= 30) center_icon = danger_icon_24x24;
                else if (percentage <= 60) center_icon = normal_status_icon_24x24;
                else center_icon = safe_icon_24x24;
                draw_bitmap(center_icon, (SCREEN_WIDTH / 2) - 12, 24, 24, 24);

                const unsigned char* batt_icon = NULL;
                if (percentage <= 25) batt_icon = batt_25_icon_16x8;
                else if (percentage <= 50) batt_icon = batt_50_icon_16x8;
                else if (percentage <= 75) batt_icon = batt_75_icon_16x8;
                else batt_icon = batt_100_icon_16x8;

                int batt_icon_x = SCREEN_WIDTH - 18;
                draw_bitmap(batt_icon, batt_icon_x, 0, 16, 8);

                char percent_str[5];
                snprintf(percent_str, sizeof(percent_str), "%d%%", percentage);
                int text_width = strlen(percent_str) * (FONT_WIDTH + 1);
                int text_x = batt_icon_x - text_width - 2;
                draw_string(percent_str, text_x, 0);
                // --- ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★ ---
            }
        }

        previous_count = count;

        ioctl(oled_fd, SSD1306_DISPLAY_FULL_BUFFER, framebuffer);
        usleep(100000);
    }

    close(rotary_fd);
    close(oled_fd);
    cleanup_gpios();
    return 0;
}

/* =================================================================
   Sysfs GPIO 제어 함수 (최적화 버전)
================================================================= */
int setup_gpios(void) {
    char buffer[64];
    int i, fd;
    for (i = 0; i < NUM_LEDS; i++) {
        led_fds[i] = -1;
        fd = open("/sys/class/gpio/export", O_WRONLY);
        if (fd >= 0) {
            sprintf(buffer, "%d", led_gpios[i]);
            write(fd, buffer, strlen(buffer));
            close(fd);
        }
        usleep(100000);
        sprintf(buffer, "/sys/class/gpio/gpio%d/direction", led_gpios[i]);
        fd = open(buffer, O_WRONLY);
        if (fd < 0) { usleep(100000); fd = open(buffer, O_WRONLY); }
        if (fd < 0) { perror("gpio direction"); return -1; }
        write(fd, "out", 3);
        close(fd);
        sprintf(buffer, "/sys/class/gpio/gpio%d/value", led_gpios[i]);
        led_fds[i] = open(buffer, O_WRONLY);
        if (led_fds[i] < 0) { perror("gpio value open"); return -1; }
    }
    printf("All LED GPIOs are set up and opened.\n");
    return 0;
}

void cleanup_gpios(void) {
    char buffer[64];
    int i, fd;
    set_leds(0);
    for (i = 0; i < NUM_LEDS; i++) {
        if (led_fds[i] != -1) close(led_fds[i]);
        fd = open("/sys/class/gpio/unexport", O_WRONLY);
        if (fd >= 0) {
            sprintf(buffer, "%d", led_gpios[i]);
            write(fd, buffer, strlen(buffer));
            close(fd);
        }
    }
    printf("All LED GPIOs are cleaned up.\n");
}

void set_leds(unsigned char value) {
    int i;
    for (i = 0; i < NUM_LEDS; i++) {
        if (led_fds[i] != -1) {
            if (value & (1 << i)) write(led_fds[i], "1", 1);
            else write(led_fds[i], "0", 1);
        }
    }
}

/* =================================================================
   I2C 및 OLED 그리기 함수 구현
================================================================= */
int read_eeprom_byte(unsigned short mem_addr) {
    int fd;
    unsigned char out_buf[2], in_buf[1];
    struct i2c_msg messages[2];
    struct i2c_rdwr_ioctl_data ioctl_data;
    if ((fd = open(I2C_DEVICE_PATH, O_RDWR)) < 0) return -1;
    out_buf[0] = (mem_addr >> 8) & 0xFF;
    out_buf[1] = mem_addr & 0xFF;
    messages[0].addr = EEPROM_ADDR; messages[0].flags = 0; messages[0].len = 2; messages[0].buf = out_buf;
    messages[1].addr = EEPROM_ADDR; messages[1].flags = I2C_M_RD; messages[1].len = 1; messages[1].buf = in_buf;
    ioctl_data.msgs = messages; ioctl_data.nmsgs = 2;
    if (ioctl(fd, I2C_RDWR, &ioctl_data) < 0) { close(fd); return -1; }
    close(fd);
    return in_buf[0];
}

void draw_bitmap(const unsigned char* bitmap, int x, int y, int width, int height) {
    int i, j;
    int page_height = height / 8;
    for (i = 0; i < width; i++) {
        for (j = 0; j < page_height; j++) {
            int current_x = x + i;
            int current_page = (y / 8) + j;
            if (current_x < 0 || current_x >= SCREEN_WIDTH || current_page < 0 || current_page >= SCREEN_HEIGHT / 8) continue;
            int offset = current_x + (current_page * SCREEN_WIDTH);
            if (offset >= FRAMEBUFFER_SIZE) continue;
            framebuffer[offset] = *(bitmap + (i * page_height) + j);
        }
    }
}

void draw_string(const char* str, int x, int y_page) {
    int cursor_x = x;
    while (*str) {
        if (cursor_x + FONT_WIDTH > SCREEN_WIDTH) {
            cursor_x = x;
            y_page++;
        }
        if (y_page >= SCREEN_HEIGHT / 8) break;
        char c = *str;
        if (c < ' ' || c > '~') c = ' ';
        const unsigned char* glyph = &font_5x8[(c - ' ') * FONT_WIDTH];
        for (int i = 0; i < FONT_WIDTH; i++) {
            if (cursor_x + i < SCREEN_WIDTH) {
                int offset = (cursor_x + i) + (y_page * SCREEN_WIDTH);
                if (offset < FRAMEBUFFER_SIZE) framebuffer[offset] = glyph[i];
            }
        }
        cursor_x += FONT_WIDTH + 1;
        str++;
    }
}