#ifndef COCIS_APP_H
#define COCIS_APP_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <time.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_random.h"
#include "esp_rom_sys.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_adc/adc_oneshot.h"

/* ==================================================================
 * PINS
 * ================================================================== */
#define PIN_SPI_SCK 18
#define PIN_SPI_MISO 19
#define PIN_SPI_MOSI 23
#define PIN_RC522_SS_IN 5
#define PIN_RC522_SS_OUT 17
#define PIN_RC522_RST 16
#define PIN_SD_CS 4
#define PIN_I2C_SDA         21
#define PIN_I2C_SCL         22
#define LCD_ADDR           0x27
#define SERVO_ENTRY_PIN     25
#define SERVO_EXIT_PIN      26
#define PIN_BUZZER 13
#define PIN_EMERG_BTN 15
#define PIN_LED_R1 14
#define PIN_LED_R2 27
#define PIN_SONAR1_TRIG 32
#define PIN_SONAR1_ECHO 34
#define PIN_SONAR2_TRIG 33
#define PIN_SONAR2_ECHO 35
#define LDR_ADC_CHANNEL ADC_CHANNEL_0

/* BEHAVIOUR */
#define ROOM_OFF_DELAY_MS 30000
#define ROOM_PRESENCE_CM 150
#define ROOM_LIGHT_WATTS 36
#define LDR_BRIGHT_RAW 2500
#define LDR_INVERT 1
#define GHOST_CONFIRM_MS 5000
#define TASK_TIMEOUT_MS 8000
#define SESSION_MS 120000
#define LOGIN_LOCK_MS 60000
#define MAX_LOGIN_FAILS 3
#define DEFAULT_ADMIN_PW "admin123"
#define LOG_ROTATE_BYTES (64 * 1024)
#define DEFAULT_SCHED_START (6 * 60)
#define DEFAULT_SCHED_END (22 * 60)
#define MAX_USERS 32
#define UID_LEN 12
#define NAME_LEN 24
#define LOG_LINE 160
#define RING_N 40
#define MOUNT_POINT "/sdcard"
#define MAX_READER_FAILED_SCANS 3
#define READER_LOCKOUT_MS 30000

#define LOG_PATH MOUNT_POINT "/access.csv"
#define LOG_PATH_1 MOUNT_POINT "/access1.csv"
#define USERS_PATH MOUNT_POINT "/users.csv"
#define USERS_TMP MOUNT_POINT "/users.tmp"

/* ==================================================================
 * WIFI DEFAULT SETTINGS
 * ================================================================== */
#define WIFI_DEFAULT_SSID   "Wokwi-GUEST"
#define WIFI_DEFAULT_PASS   ""

/* ==================================================================
 * TYPES
 * ================================================================== */
typedef enum
{
    DIRECTION_ENTRY = 0,
    DIRECTION_EXIT = 1
} direction_t;

typedef enum
{
    ACCESS_GRANTED,
    ACCESS_DENIED_UNKNOWN,
    ACCESS_DENIED_SCHEDULE,
    ACCESS_DENIED_PASSBACK,
    ACCESS_DENIED_LOCKOUT
} access_result_t;

typedef enum
{
    TASK_READER,
    TASK_ROOMS,
    TASK_LOGGER,
    TASK_CLI,
    TASK_COORD,
    TASK_BUZZER,
    TASK_DISPLAY,
    TASK_IO,
    TASK_COUNT
} task_id_t;

typedef struct
{
    char uid[UID_LEN];
    char name[NAME_LEN];
    bool inside;
    bool used;
} user_t;

typedef struct
{
    bool occupied;
    bool light_on;
    int distance_cm;
    int off_in_seconds;
    uint32_t saved_seconds;
    uint32_t auto_off_count;
} room_t;

typedef struct
{
    int date;
    uint16_t entries;
    uint16_t exits;
    uint16_t denied;
} day_stat_t;

enum
{
    MESSAGE_ROOM_STATE = 1
};

typedef struct
{
    uint8_t type;
    uint8_t room;
    uint8_t occupied;
    uint8_t light_on;
} room_message_t;

typedef struct
{
    char line[LOG_LINE];
} log_item_t;

typedef enum
{
    RESOURCE_DATA,
    RESOURCE_CONFIG,
    RESOURCE_LOG,
    RESOURCE_SD,
    RESOURCE_SPI,
    RESOURCE_I2C,
    RESOURCE_COUNT
} resource_t;

#define EVENT_EMERGENCY (1u << 0)

enum
{
    BUZZER_OK = 1,
    BUZZER_DENIED,
    BUZZER_LOCKOUT
};

/* ==================================================================
 * SHARED STATE
 * ================================================================== */
extern SemaphoreHandle_t resource_mutexes[RESOURCE_COUNT];
extern volatile uint32_t resource_lock_timeouts[RESOURCE_COUNT];
extern QueueHandle_t log_queue;
extern QueueHandle_t room_message_queue;
extern QueueHandle_t buzzer_queue;
extern EventGroupHandle_t system_event_group;
extern volatile uint32_t task_last_heartbeat[TASK_COUNT];

extern user_t registered_users[MAX_USERS];
extern int people_inside_count;
extern bool users_changed;
extern room_t room_states[2];
extern day_stat_t daily_statistics[7];
extern bool statistics_changed;
extern char recent_alerts[8][96];
extern int alert_total;
extern volatile int ldr_raw_value;

extern uint16_t schedule_start_minutes;
extern uint16_t schedule_end_minutes;
extern nvs_handle_t nvshandle;

extern char log_ring[RING_N][LOG_LINE];
extern int log_ring_total;
extern volatile bool sd_card_ok;
extern volatile uint32_t log_dropped_count;
extern volatile bool echo_log;

extern bool rfid_reader_present[2];

/* sync.c */
bool lock_resource(resource_t resource, uint32_t timeout_ms);
void unlock_resource(resource_t resource);
void update_task_heartbeat(task_id_t task);
bool emergency_active(void);
void format_time(time_t t, char *out, size_t size);
void log_event(const char *type, const char *fmt, ...);
void raise_alert(const char *kind, const char *fmt, ...);
void log_print_recent(int max_lines);

/* config.c */
bool schedule_is_open(time_t now);
void schedule_save(void);
void config_load(void);
int today_key(void);
void statistics_add_locked(int kind);
void password_store(const char *password);
bool password_check(const char *password);
void auth_init(void);
void clock_init(void);

/* rfid.c */
void rc522_setup(void);
bool rfid_read_uid(int reader, char *uid);

/* access.c */
void buzz(int pattern);
void tone(int hz, int ms);
void buzzer_task(void *arg);
void set_servo_angle(int angle, int servo);
void init_access_servos(void);
void buzzer_ledc_setup(void);
void open_door(direction_t direction);
user_t *find_user(const char *uid);
access_result_t access_handle_scan(direction_t direction, const char *uid_input);
void reader_task(void *arg);

/* storage_logger.c */
void spi_bus_setup(void);
void sd_card_setup(void);
void users_load(void);
void logger_task(void *arg);

/* rooms.c */
void rooms_gpio_init(void);
void rooms_adc_init(void);
int rooms_read_ldr(void);
bool ldr_is_bright(void);
void rooms_task(void *arg);

/* cocis_system.c */
void system_gpio_init(void);
void emergency_set(bool on, const char *who);
void coordinator_task(void *arg);
void io_task(void *arg);
void supervisor_task(void *arg);
void display_task(void *arg);

/* cli.c */
void cli_task(void *arg);
extern volatile bool stress_test_active;

/* network.c */
bool wifi_start(void);
bool wifi_is_connected(void);
bool time_is_synced(void);
const char *wifi_get_ip(void);
void wifi_print_status(void);
bool wifi_save_credentials(const char *ssid, const char *password);
void wifi_reconnect(void);
void start_webserver(void);

extern volatile bool stress_test_active;
#endif