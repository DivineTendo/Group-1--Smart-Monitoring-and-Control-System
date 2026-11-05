#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"

/* Pins */
#define PIN_SPI_SCK         18
#define PIN_SPI_MISO        19
#define PIN_SPI_MOSI        23

#define PIN_RC522_SS_IN     5
#define PIN_RC522_SS_OUT    17
#define PIN_RC522_RST       16

#define SERVO_ENTRY_PIN 25
#define SERVO_EXIT_PIN  26

#define PIN_BUZZER          13

#define DEFAULT_SCHED_START (6 * 60)
#define DEFAULT_SCHED_END   (22 * 60)

#define UID_LEN             12
#define NAME_LEN            24
#define MAX_USERS           32

static const char *TAG = "COCIS_ENTRANCE";

/* Types */
typedef enum {
    DIRECTION_ENTRY = 0,
    DIRECTION_EXIT = 1
} direction_t;

typedef enum {
    ACCESS_GRANTED,
    ACCESS_DENIED_UNKNOWN,
    ACCESS_DENIED_SCHEDULE,
    ACCESS_DENIED_PASSBACK
} access_result_t;

typedef struct {
    char uid[UID_LEN];
    char name[NAME_LEN];
    bool inside;
    bool used;
} user_t;

enum {
    BUZZER_OK = 1,
    BUZZER_DENIED
};

/* Shared state */
static QueueHandle_t buzzer_queue;

static user_t registered_users[MAX_USERS] = {
    { "01:02:03:04", "Alice Namuli", false, true },
    { "11:22:33:44", "Brian Okello", false, true },
    { "55:66:77:88", "Carol Nakato", false, true },
    { "DE:AD:BE:EF", "Dr Mukasa",    false, true }
};

/* Clock */
static void clock_set_manual(int year, int month, int day,
                             int hour, int minute, int second)
{
    struct tm tm = {
        .tm_year  = year - 1900,
        .tm_mon   = month - 1,
        .tm_mday  = day,
        .tm_hour  = hour,
        .tm_min   = minute,
        .tm_sec   = second,
        .tm_isdst = 0
    };

    time_t t = mktime(&tm);

    struct timeval tv = {
        .tv_sec  = t,
        .tv_usec = 0
    };

    settimeofday(&tv, NULL);
}

/* Buzzer */
static void buzz(int pattern)
{
    if (buzzer_queue != NULL) {
        xQueueSend(buzzer_queue, &pattern, 0);
    }
}

static void tone(int hz, int ms)
{
    if (hz > 0) {
        ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_1, hz);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 4096);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
    }

    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);

    vTaskDelay(pdMS_TO_TICKS(ms));

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
}

static void buzzer_task(void *arg)
{
    while (1) {
        int pattern;

        if (xQueueReceive(buzzer_queue, &pattern, pdMS_TO_TICKS(200)) != pdTRUE) {
            continue;
        }

        switch (pattern) {
            case BUZZER_OK:
                tone(2000, 80);
                break;

            case BUZZER_DENIED:
                tone(400, 150);
                tone(0, 100);
                tone(400, 150);
                break;

            default:
                break;
        }
    }
}

/* Servo and door */
static void set_servo_angle(int angle, int servo)
{
    if (angle < 0) {
        angle = 0;
    } else if (angle > 180) {
        angle = 180;
    }

    uint32_t pulse = 500 + ((uint32_t)angle * 1900U) / 180U;
    uint32_t duty = (pulse * 8192U) / 20000U;

    if (servo == 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    }
}

static void init_access_servos(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ledc_timer_config(&timer);

    ledc_channel_config_t channel1 = {
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .gpio_num = SERVO_ENTRY_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .intr_type = LEDC_INTR_DISABLE,
        .duty = 0,
        .hpoint = 0
    };

    ledc_channel_config_t channel2 = {
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_0,
        .gpio_num = SERVO_EXIT_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .intr_type = LEDC_INTR_DISABLE,
        .duty = 0,
        .hpoint = 0
    };

    ledc_channel_config(&channel1);
    ledc_channel_config(&channel2);

    /* Both doors start in their locked position. */
    set_servo_angle(0, 0);
    set_servo_angle(0, 1);
}

static void open_door(direction_t direction)
{
    int servo = direction == DIRECTION_ENTRY ? 0 : 1;
    int closed_angle = 0;

    set_servo_angle(90, servo);
    vTaskDelay(pdMS_TO_TICKS(2000));

    set_servo_angle(closed_angle, servo);
}

/* Schedule */
static bool schedule_is_open(time_t now)
{
    uint16_t start = DEFAULT_SCHED_START;
    uint16_t end = DEFAULT_SCHED_END;

    struct tm tm;
    localtime_r(&now, &tm);

    int current_minutes = tm.tm_hour * 60 + tm.tm_min;

    if (start == end) {
        return true;
    }

    if (start < end) {
        return current_minutes >= start && current_minutes < end;
    }

    /* Crosses midnight. */
    return current_minutes >= start || current_minutes < end;
}

/* RC522 driver */
#define RC_COMMAND_REG      0x01
#define RC_COM_IRQ_REG      0x04
#define RC_ERROR_REG        0x06
#define RC_STATUS2_REG      0x08
#define RC_FIFO_DATA_REG    0x09
#define RC_FIFO_LEVEL_REG   0x0A
#define RC_BIT_FRAMING_REG  0x0D
#define RC_COLL_REG         0x0E
#define RC_MODE_REG         0x11
#define RC_TX_CONTROL_REG   0x14
#define RC_TX_ASK_REG       0x15
#define RC_T_MODE_REG       0x2A
#define RC_T_PRESCALER_REG  0x2B
#define RC_T_RELOAD_REG_H   0x2C
#define RC_T_RELOAD_REG_L   0x2D
#define RC_VERSION_REG      0x37

static spi_device_handle_t rfid_readers[2];
static bool rfid_reader_present[2];

static void spi_bus_setup(void)
{
    spi_bus_config_t bus = {
        .mosi_io_num = PIN_SPI_MOSI,
        .miso_io_num = PIN_SPI_MISO,
        .sclk_io_num = PIN_SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));
}

static uint8_t rc_read(int reader, uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(((reg << 1) & 0x7E) | 0x80), 0 };
    uint8_t rx[2] = { 0, 0 };

    spi_transaction_t transaction = {
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = rx
    };

    spi_device_transmit(rfid_readers[reader], &transaction);

    return rx[1];
}

static void rc_read_fifo(int reader, uint8_t *data, int count)
{
    if (count <= 0 || count > 64) {
        return;
    }

    uint8_t tx[65] = { 0 };
    uint8_t rx[65] = { 0 };

    const uint8_t address = (uint8_t)(((RC_FIFO_DATA_REG << 1) & 0x7E) | 0x80);

    tx[0] = address;

    for (int i = 1; i < count; i++) {
        tx[i] = address;
    }

    spi_transaction_t transaction = {
        .length = (count + 1) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx
    };

    spi_device_transmit(rfid_readers[reader], &transaction);

    memcpy(data, &rx[1], count);
}

static void rc_write(int reader, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)((reg << 1) & 0x7E), value };

    spi_transaction_t transaction = {
        .length = 16,
        .tx_buffer = tx
    };

    spi_device_transmit(rfid_readers[reader], &transaction);
}

static void rc_set_bits(int reader, uint8_t reg, uint8_t mask)
{
    uint8_t value = rc_read(reader, reg);
    rc_write(reader, reg, value | mask);
}

static void rc_clear_bits(int reader, uint8_t reg, uint8_t mask)
{
    uint8_t value = rc_read(reader, reg);
    rc_write(reader, reg, value & (uint8_t)~mask);
}

static void rc_init(int reader)
{
    rc_write(reader, RC_COMMAND_REG, 0x0F);
    vTaskDelay(pdMS_TO_TICKS(50));

    rc_write(reader, RC_T_MODE_REG, 0x80);
    rc_write(reader, RC_T_PRESCALER_REG, 0xA9);
    rc_write(reader, RC_T_RELOAD_REG_H, 0x03);
    rc_write(reader, RC_T_RELOAD_REG_L, 0xE8);
    rc_write(reader, RC_TX_ASK_REG, 0x40);
    rc_write(reader, RC_MODE_REG, 0x3D);

    if (!(rc_read(reader, RC_TX_CONTROL_REG) & 0x03)) {
        rc_set_bits(reader, RC_TX_CONTROL_REG, 0x03);
    }
}

static int rc_transceive(int reader, const uint8_t *send, int send_len,
                         uint8_t *receive, int *receive_len, uint8_t tx_bits)
{
    rc_write(reader, RC_COMMAND_REG, 0x00);
    rc_write(reader, RC_COM_IRQ_REG, 0x7F);
    rc_write(reader, RC_FIFO_LEVEL_REG, 0x80);

    for (int i = 0; i < send_len; i++) {
        rc_write(reader, RC_FIFO_DATA_REG, send[i]);
    }

    rc_write(reader, RC_BIT_FRAMING_REG, tx_bits);
    rc_write(reader, RC_COMMAND_REG, 0x0C);

    rc_set_bits(reader, RC_BIT_FRAMING_REG, 0x80);

    uint8_t irq = 0;
    int64_t deadline = esp_timer_get_time() + 40000;

    while (1) {
        irq = rc_read(reader, RC_COM_IRQ_REG);

        if (irq & 0x31) {
            break;
        }

        if (esp_timer_get_time() >= deadline) {
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    rc_clear_bits(reader, RC_BIT_FRAMING_REG, 0x80);

    if (!(irq & 0x31)) {
        return -1;
    }

    if (rc_read(reader, RC_ERROR_REG) & 0x13) {
        return -2;
    }

    if (irq & 0x01) {
        return -3;
    }

    int count = rc_read(reader, RC_FIFO_LEVEL_REG);

    if (count > *receive_len) {
        count = *receive_len;
    }

    rc_read_fifo(reader, receive, count);

    *receive_len = count;

    return 0;
}

static bool rc_read_uid(int reader, char *uid)
{
    uint8_t buffer[8];
    int buffer_len = sizeof(buffer);

    rc_clear_bits(reader, RC_STATUS2_REG, 0x08);
    rc_clear_bits(reader, RC_COLL_REG, 0x80);

    const uint8_t request = 0x26;

    if (rc_transceive(reader, &request, 1, buffer, &buffer_len, 0x07) != 0 || buffer_len != 2) {
        return false;
    }

    const uint8_t anti_collision[2] = { 0x93, 0x20 };

    buffer_len = sizeof(buffer);

    if (rc_transceive(reader, anti_collision, 2, buffer, &buffer_len, 0x00) != 0 || buffer_len != 5) {
        return false;
    }

    if ((buffer[0] ^ buffer[1] ^ buffer[2] ^ buffer[3]) != buffer[4]) {
        return false;
    }

    snprintf(uid, UID_LEN, "%02X:%02X:%02X:%02X",
             buffer[0], buffer[1], buffer[2], buffer[3]);

    return true;
}

static void rc522_setup(void)
{
    gpio_config_t reset_pin = {
        .pin_bit_mask = 1ULL << PIN_RC522_RST,
        .mode = GPIO_MODE_OUTPUT
    };

    gpio_config(&reset_pin);

    gpio_set_level(PIN_RC522_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    gpio_set_level(PIN_RC522_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    const int chip_select_pins[2] = {
        PIN_RC522_SS_IN,
        PIN_RC522_SS_OUT
    };

    for (int reader = 0; reader < 2; reader++) {
        spi_device_interface_config_t device_config = {
            .clock_speed_hz = 4 * 1000 * 1000,
            .mode = 0,
            .spics_io_num = chip_select_pins[reader],
            .queue_size = 2
        };

        ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &device_config, &rfid_readers[reader]));

        rc_init(reader);

        uint8_t version = rc_read(reader, RC_VERSION_REG);

        rfid_reader_present[reader] = (version != 0x00 && version != 0xFF);

        ESP_LOGI(TAG, "RC522 %s: version 0x%02X (%s)",
                 reader == 0 ? "ENTRY" : "EXIT",
                 version,
                 rfid_reader_present[reader] ? "present" : "not detected");
    }
}

/* Access logic */
static user_t *find_user(const char *uid)
{
    for (int i = 0; i < MAX_USERS; i++) {
        if (registered_users[i].used && strcmp(registered_users[i].uid, uid) == 0) {
            return &registered_users[i];
        }
    }

    return NULL;
}

static access_result_t access_handle_scan(direction_t direction, const char *uid_input)
{
    char uid[UID_LEN];

    snprintf(uid, sizeof(uid), "%s", uid_input);

    for (char *p = uid; *p; p++) {
        *p = toupper((unsigned char)*p);
    }

    bool schedule_ok = schedule_is_open(time(NULL));

    access_result_t result = ACCESS_DENIED_UNKNOWN;

    char name[NAME_LEN] = "-";

    user_t *user = find_user(uid);

    if (user == NULL) {
        result = ACCESS_DENIED_UNKNOWN;
    } else {
        snprintf(name, sizeof(name), "%s", user->name);

        if (!schedule_ok) {
            result = ACCESS_DENIED_SCHEDULE;
        } else if (direction == DIRECTION_ENTRY && user->inside) {
            result = ACCESS_DENIED_PASSBACK;
        } else if (direction == DIRECTION_EXIT && !user->inside) {
            result = ACCESS_DENIED_PASSBACK;
        } else {
            user->inside = (direction == DIRECTION_ENTRY);
            result = ACCESS_GRANTED;
        }
    }

    static const char *result_names[] = {
        "GRANTED",
        "DENIED_UNKNOWN",
        "DENIED_SCHEDULE",
        "DENIED_PASSBACK"
    };

    printf("[RFID] %s UID %s -> %s (%s)\n",
           direction == DIRECTION_ENTRY ? "ENTRY" : "EXIT",
           uid,
           result_names[result],
           name);

    if (result == ACCESS_GRANTED) {
        buzz(BUZZER_OK);
        open_door(direction);
    } else {
        buzz(BUZZER_DENIED);
    }

    return result;
}

/* Reader task */
static void reader_task(void *arg)
{
    char last_uid[2][UID_LEN] = { "", "" };
    int64_t last_time_ms[2] = { 0, 0 };

    while (1) {
        for (int reader = 0; reader < 2; reader++) {
            if (!rfid_reader_present[reader]) {
                continue;
            }

            char uid[UID_LEN];

            if (!rc_read_uid(reader, uid)) {
                continue;
            }

            int64_t now_ms = esp_timer_get_time() / 1000;

            /* Ignore repeated scans from the same reader for 2 s. */
            if (strcmp(uid, last_uid[reader]) == 0 && now_ms - last_time_ms[reader] < 2000) {
                last_time_ms[reader] = now_ms;
                continue;
            }

            snprintf(last_uid[reader], UID_LEN, "%s", uid);
            last_time_ms[reader] = now_ms;

            access_handle_scan(reader == 0 ? DIRECTION_ENTRY : DIRECTION_EXIT, uid);
        }

        vTaskDelay(pdMS_TO_TICKS(120));
    }
}

/* Hardware setup */
static void buzzer_ledc_setup(void)
{
    ledc_timer_config_t buzzer_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 2000,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ledc_timer_config(&buzzer_timer);

    ledc_channel_config_t buzzer_channel = {
        .gpio_num = PIN_BUZZER,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_2,
        .timer_sel = LEDC_TIMER_1
    };

    ledc_channel_config(&buzzer_channel);
}

/* Main */
void app_main(void)
{
    clock_set_manual(2026, 11, 3, 10, 30, 0);

    buzzer_queue = xQueueCreate(8, sizeof(int));

    init_access_servos();
    buzzer_ledc_setup();

    spi_bus_setup();
    rc522_setup();

    xTaskCreate(buzzer_task, "buzzer", 2048, NULL, 3, NULL);
    xTaskCreate(reader_task, "reader", 6144, NULL, 5, NULL);
}