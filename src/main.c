/*
* CoCIS Smart Control and Monitoring System  -  single ESP32, single main.c
 * ESP-IDF (>= 5.1) / PlatformIO / Wokwi
 *
 * Sections (match the five-module plan):
 *   0. Config, pins, shared types
 *   3. Bus & sync (bus_lock), logging front-end, supervisor heartbeat
 *   1. Entrance: RC522 driver, access_handle_scan, doors, buzzer
 *   2. Data & admin: SD/FATFS, logger, users, NVS schedule, auth, CLI
 *   4. Rooms: sonar presence, lights, daylight, energy counter
 *   5. Emergency, ghost occupancy, OLED dashboard, app_main
 *
 * LOCK ORDER (always ascending, never take a lower one while holding a higher one):
 *   RES_DATA < RES_CFG < RES_LOG < RES_SD < RES_SPI < RES_I2C
 * log_event() takes no lock, so it is safe to call from anywhere.
 */

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
#include "mbedtls/sha256.h"

/* ====================================================================== */
/* 0. CONFIG + PINS                                                       */
/* ====================================================================== */

/* ---- PINS (single source of truth; keep diagram.json in sync) ---- */
#define PIN_SPI_SCK        18
#define PIN_SPI_MISO       19
#define PIN_SPI_MOSI       23
#define PIN_RC522_SS_IN     5
#define PIN_RC522_SS_OUT   17
#define PIN_RC522_RST      16
#define PIN_SD_CS           4
#define PIN_I2C_SDA        21
#define PIN_I2C_SCL        22
#define PIN_SERVO_IN       25
#define PIN_SERVO_OUT      26
#define PIN_BUZZER         13
#define PIN_EMERG_BTN      15
#define PIN_LED_R1         14
#define PIN_LED_R2         27
#define PIN_SONAR1_TRIG    32
#define PIN_SONAR1_ECHO    34
#define PIN_SONAR2_TRIG    33
#define PIN_SONAR2_ECHO    35
#define LDR_ADC_CHANNEL    ADC_CHANNEL_0      /* GPIO36 / VP */

/* ---- Behaviour ---- */
#define DOOR_OPEN_MS        5000
#define MAX_FAILS           3
#define LOCKOUT_MS          30000
#define ROOM_OFF_DELAY_MS   30000
#define ROOM_WARN_MS        5000      /* blink during the last N ms before lights-off */
#define ROOM_PRESENCE_CM    150       /* sonar closer than this = someone is there   */
#define ROOM_LIGHT_WATTS    36
#define LDR_BRIGHT_RAW      2500      /* above this = daylight (flip LDR_INVERT if sensor is opposite) */
#define LDR_INVERT          0
#define GHOST_CONFIRM_MS    5000
#define TASK_TIMEOUT_MS     8000
#define SESSION_MS          120000
#define LOGIN_LOCK_MS       60000
#define MAX_LOGIN_FAILS     3
#define DEFAULT_ADMIN_PW    "admin123"
#define LOG_ROTATE_BYTES    (64 * 1024)
#define DEFAULT_SCHED_START (6 * 60)
#define DEFAULT_SCHED_END   (22 * 60)

#define MAX_USERS   32
#define UID_LEN     12
#define NAME_LEN    24
#define LOG_LINE    160
#define RING_N      40
#define OLED_ADDR   0x3C

#define MOUNT_POINT "/sdcard"
/* 8.3-safe names: work even if FATFS long file names are not enabled */
#define LOG_PATH    MOUNT_POINT "/access.csv"
#define LOG_PATH_1  MOUNT_POINT "/access1.csv"
#define USERS_PATH  MOUNT_POINT "/users.csv"
#define USERS_TMP   MOUNT_POINT "/users.tmp"

static const char *TAG = "cocis";

typedef enum { DIR_ENTRY = 0, DIR_EXIT = 1 } dir_t;
typedef enum {
    ACC_GRANTED, ACC_DENIED_UNKNOWN, ACC_DENIED_SCHEDULE,
    ACC_DENIED_PASSBACK, ACC_DENIED_LOCKOUT
} acc_result_t;

typedef enum {
    T_READER, T_DOOR, T_ROOMS, T_LOGGER, T_CLI, T_COORD, T_BUZZ, T_DISPLAY, T_IO, T_COUNT
} task_id_t;
static const char *TASK_NAME[T_COUNT] = {
    "reader", "door", "rooms", "logger", "cli", "coord", "buzzer", "display", "io"
};

typedef struct {
    char   uid[UID_LEN];
    char   name[NAME_LEN];
    bool   inside;
    time_t last_in;
    bool   used;
} user_t;

typedef struct {
    bool presence, light;
    int  dist_cm;
    int  off_in_s;
    uint32_t saved_s;      /* seconds the light was off during open hours */
    uint32_t auto_offs;
} room_t;

typedef struct { int date; uint16_t entries, exits, denied; } day_stat_t;

enum { MSG_ROOM_STATE = 1 };
typedef struct { uint8_t type, room, presence, light; } msg_t;
typedef struct { char line[LOG_LINE]; } log_item_t;

#define EV_EMERGENCY (1u << 0)

/* ====================================================================== */
/* 3. BUS & SYNC                                                          */
/* ====================================================================== */

typedef enum { RES_DATA, RES_CFG, RES_LOG, RES_SD, RES_SPI, RES_I2C, RES_COUNT } res_t;
static const char *RES_NAME[RES_COUNT] = { "DATA", "CFG", "LOG", "SD", "SPI", "I2C" };
static SemaphoreHandle_t g_locks[RES_COUNT];
static volatile uint32_t g_lock_timeouts[RES_COUNT];

static bool bus_lock(res_t r, uint32_t ms)
{
    if (xSemaphoreTake(g_locks[r], pdMS_TO_TICKS(ms)) == pdTRUE) return true;
    g_lock_timeouts[r]++;
    printf("!! lock timeout on %s\n", RES_NAME[r]);
    return false;
}
static void bus_unlock(res_t r) { xSemaphoreGive(g_locks[r]); }

static QueueHandle_t g_q_log, g_q_msg, g_q_buzz;
static EventGroupHandle_t g_ev;

static volatile uint32_t g_hb[T_COUNT];
static volatile uint32_t g_hang_mask;
static inline void hb_beat(task_id_t id)
{
    if (!(g_hang_mask & (1u << id))) g_hb[id] = xTaskGetTickCount();
}

static inline bool emergency_active(void) { return (xEventGroupGetBits(g_ev) & EV_EMERGENCY) != 0; }

/* ---- shared state (guarded by RES_DATA unless noted) ---- */
static user_t     g_users[MAX_USERS];
static int        g_inside;
static bool       g_users_dirty;
static room_t     g_rooms[2];
static day_stat_t g_stats[7];
static bool       g_stats_dirty;
static int64_t    g_lockout_until[2];
static int        g_fail_count[2];
static char       g_alerts[8][96];
static int        g_alert_total;
static volatile bool g_force_open;           /* stress test only */
static volatile int  g_ldr_raw;

/* ---- config (RES_CFG) ---- */
static uint16_t   g_sched_start = DEFAULT_SCHED_START;
static uint16_t   g_sched_end   = DEFAULT_SCHED_END;
static nvs_handle_t g_nvs;

/* ---- log ring (RES_LOG) + SD state ---- */
static char       g_ring[RING_N][LOG_LINE];
static int        g_ring_n;
static volatile bool g_sd_ok;
static volatile uint32_t g_log_dropped;
static volatile bool g_echo_log = true;

static void fmt_time(time_t t, char *out, size_t n)
{
    struct tm tm; localtime_r(&t, &tm);
    strftime(out, n, "%Y-%m-%d %H:%M:%S", &tm);
}

/* Lock-free front end: formats a CSV line and queues it for the logger task. */
static void log_event(const char *type, const char *fmt, ...)
{
    if (!g_q_log) return;
    char detail[100];
    va_list ap; va_start(ap, fmt); vsnprintf(detail, sizeof detail, fmt, ap); va_end(ap);
    char ts[24]; fmt_time(time(NULL), ts, sizeof ts);
    log_item_t it;
    snprintf(it.line, sizeof it.line, "%s,%s,%s", ts, type, detail);
    if (xQueueSend(g_q_log, &it, pdMS_TO_TICKS(20)) != pdTRUE) g_log_dropped++;
}

static void buzz(int pattern);   /* fwd */

static void raise_alert(const char *kind, const char *fmt, ...)
{
    char detail[80];
    va_list ap; va_start(ap, fmt); vsnprintf(detail, sizeof detail, fmt, ap); va_end(ap);
    char ts[24]; fmt_time(time(NULL), ts, sizeof ts);
    if (bus_lock(RES_DATA, 200)) {
        int slot = g_alert_total % 8;
        snprintf(g_alerts[slot], sizeof g_alerts[slot], "%s %s: %s", ts + 11, kind, detail);
        g_alert_total++;
        bus_unlock(RES_DATA);
    }
    log_event("ALERT", "%s;%s", kind, detail);
    buzz(3);
}

/* ====================================================================== */
/* CONFIG / NVS / SCHEDULE                                                */
/* ====================================================================== */

static bool sched_is_open(time_t t)
{
    if (g_force_open) return true;
    uint16_t s, e;
    if (bus_lock(RES_CFG, 200)) { s = g_sched_start; e = g_sched_end; bus_unlock(RES_CFG); }
    else { s = DEFAULT_SCHED_START; e = DEFAULT_SCHED_END; }
    struct tm tm; localtime_r(&t, &tm);
    int m = tm.tm_hour * 60 + tm.tm_min;
    if (s == e) return true;
    if (s < e) return m >= s && m < e;
    return m >= s || m < e;           /* window wraps past midnight */
}

static void sched_save(void)
{
    if (!bus_lock(RES_CFG, 500)) return;
    nvs_set_u16(g_nvs, "sch_start", g_sched_start);
    nvs_set_u16(g_nvs, "sch_end", g_sched_end);
    nvs_commit(g_nvs);
    bus_unlock(RES_CFG);
}

static void config_load(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase(); nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_open("cocis", NVS_READWRITE, &g_nvs));
    uint16_t v;
    if (nvs_get_u16(g_nvs, "sch_start", &v) == ESP_OK) g_sched_start = v;
    if (nvs_get_u16(g_nvs, "sch_end", &v) == ESP_OK)   g_sched_end = v;
    size_t sz = sizeof g_stats;
    if (nvs_get_blob(g_nvs, "stats", g_stats, &sz) != ESP_OK) memset(g_stats, 0, sizeof g_stats);
}

static int today_key(void)
{
    time_t t = time(NULL); struct tm tm; localtime_r(&t, &tm);
    return (tm.tm_year + 1900) * 10000 + (tm.tm_mon + 1) * 100 + tm.tm_mday;
}

/* kind: 0 entry, 1 exit, 2 denied.  Caller holds RES_DATA. */
static void stats_bump_locked(int kind)
{
    int d = today_key(), slot = -1, oldest = 0;
    for (int i = 0; i < 7; i++) {
        if (g_stats[i].date == d) { slot = i; break; }
        if (g_stats[i].date < g_stats[oldest].date) oldest = i;
    }
    if (slot < 0) { slot = oldest; memset(&g_stats[slot], 0, sizeof g_stats[slot]); g_stats[slot].date = d; }
    if (kind == 0) g_stats[slot].entries++;
    else if (kind == 1) g_stats[slot].exits++;
    else g_stats[slot].denied++;
    g_stats_dirty = true;
}

/* ====================================================================== */
/* BUZZER + SERVO (LEDC)                                                  */
/* ====================================================================== */

enum { BUZ_OK = 1, BUZ_DENY, BUZ_ALERT, BUZ_LOCKOUT };

static void buzz(int pattern)
{
    if (g_q_buzz) xQueueSend(g_q_buzz, &pattern, 0);
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
    for (;;) {
        hb_beat(T_BUZZ);
        if (emergency_active()) {          /* fire siren */
            tone(1800, 400); tone(0, 300);
            continue;
        }
        int p;
        if (xQueueReceive(g_q_buzz, &p, pdMS_TO_TICKS(200)) != pdTRUE) continue;
        switch (p) {
        case BUZ_OK:      tone(2000, 80); break;
        case BUZ_DENY:    tone(400, 150); tone(0, 100); tone(400, 150); break;
        case BUZ_ALERT:   for (int i = 0; i < 5; i++) { tone(2500, 100); tone(0, 100); hb_beat(T_BUZZ); } break;
        case BUZ_LOCKOUT: for (int i = 0; i < 6; i++) { tone(800 + 400 * (i & 1), 500); hb_beat(T_BUZZ); } break;
        }
    }
}

static void servo_write(ledc_channel_t ch, int angle)
{
    int pulse_us = 500 + angle * 2000 / 180;
    uint32_t duty = (uint32_t)pulse_us * 16384 / 20000;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

/* ====================================================================== */
/* 1. ENTRANCE: RC522 driver                                              */
/* ====================================================================== */

#define RC_CommandReg   0x01
#define RC_ComIrqReg    0x04
#define RC_ErrorReg     0x06
#define RC_Status2Reg   0x08
#define RC_FIFODataReg  0x09
#define RC_FIFOLevelReg 0x0A
#define RC_BitFramingReg 0x0D
#define RC_CollReg      0x0E
#define RC_ModeReg      0x11
#define RC_TxControlReg 0x14
#define RC_TxASKReg     0x15
#define RC_TModeReg     0x2A
#define RC_TPrescalerReg 0x2B
#define RC_TReloadRegH  0x2C
#define RC_TReloadRegL  0x2D
#define RC_VersionReg   0x37

static spi_device_handle_t g_rc[2];
static bool g_rc_present[2];

static uint8_t rc_read(int r, uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(((reg << 1) & 0x7E) | 0x80), 0 }, rx[2] = { 0, 0 };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx, .rx_buffer = rx };
    spi_device_transmit(g_rc[r], &t);
    return rx[1];
}
static void rc_write(int r, uint8_t reg, uint8_t v)
{
    uint8_t tx[2] = { (uint8_t)((reg << 1) & 0x7E), v };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx };
    spi_device_transmit(g_rc[r], &t);
}
static void rc_setbits(int r, uint8_t reg, uint8_t m) { rc_write(r, reg, rc_read(r, reg) | m); }
static void rc_clrbits(int r, uint8_t reg, uint8_t m) { rc_write(r, reg, rc_read(r, reg) & ~m); }

/* Caller must hold RES_SPI. */
static void rc_init(int r)
{
    rc_write(r, RC_CommandReg, 0x0F);            /* soft reset */
    vTaskDelay(pdMS_TO_TICKS(50));
    rc_write(r, RC_TModeReg, 0x8D);
    rc_write(r, RC_TPrescalerReg, 0x3E);
    rc_write(r, RC_TReloadRegL, 30);
    rc_write(r, RC_TReloadRegH, 0);
    rc_write(r, RC_TxASKReg, 0x40);
    rc_write(r, RC_ModeReg, 0x3D);
    if (!(rc_read(r, RC_TxControlReg) & 0x03)) rc_setbits(r, RC_TxControlReg, 0x03);
}

static int rc_transceive(int r, const uint8_t *send, int sl, uint8_t *back, int *bl, uint8_t txbits)
{
    rc_write(r, RC_CommandReg, 0x00);
    rc_write(r, RC_ComIrqReg, 0x7F);
    rc_write(r, RC_FIFOLevelReg, 0x80);
    for (int i = 0; i < sl; i++) rc_write(r, RC_FIFODataReg, send[i]);
    rc_write(r, RC_BitFramingReg, txbits);
    rc_write(r, RC_CommandReg, 0x0C);
    rc_setbits(r, RC_BitFramingReg, 0x80);
    int i = 2000; uint8_t n;
    do { n = rc_read(r, RC_ComIrqReg); } while (--i && !(n & 0x31));
    rc_clrbits(r, RC_BitFramingReg, 0x80);
    if (!i) return -1;
    if (rc_read(r, RC_ErrorReg) & 0x1B) return -2;
    if (n & 0x01) return -3;                      /* timer: no card */
    int cnt = rc_read(r, RC_FIFOLevelReg);
    if (cnt > *bl) cnt = *bl;
    for (int k = 0; k < cnt; k++) back[k] = rc_read(r, RC_FIFODataReg);
    *bl = cnt;
    return 0;
}

/* Caller must hold RES_SPI. Returns true and fills uid ("01:02:03:04") if a card answered. */
static bool rc_read_uid(int r, char *uid)
{
    uint8_t buf[8]; int bl = sizeof buf;
    rc_clrbits(r, RC_Status2Reg, 0x08);
    rc_clrbits(r, RC_CollReg, 0x80);
    const uint8_t reqa = 0x26;
    if (rc_transceive(r, &reqa, 1, buf, &bl, 0x07) != 0 || bl != 2) return false;
    const uint8_t anti[2] = { 0x93, 0x20 };
    bl = sizeof buf;
    if (rc_transceive(r, anti, 2, buf, &bl, 0x00) != 0 || bl != 5) return false;
    if ((buf[0] ^ buf[1] ^ buf[2] ^ buf[3]) != buf[4]) return false;
    snprintf(uid, UID_LEN, "%02X:%02X:%02X:%02X", buf[0], buf[1], buf[2], buf[3]);
    return true;
}

static void rc522_setup(void)
{
    gpio_config_t io = { .pin_bit_mask = 1ULL << PIN_RC522_RST, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&io);
    gpio_set_level(PIN_RC522_RST, 0); vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_RC522_RST, 1); vTaskDelay(pdMS_TO_TICKS(50));
    const int ss[2] = { PIN_RC522_SS_IN, PIN_RC522_SS_OUT };
    for (int r = 0; r < 2; r++) {
        spi_device_interface_config_t dc = {
            .clock_speed_hz = 4 * 1000 * 1000, .mode = 0, .spics_io_num = ss[r], .queue_size = 2,
        };
        ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dc, &g_rc[r]));
        if (bus_lock(RES_SPI, 500)) {
            rc_init(r);
            uint8_t v = rc_read(r, RC_VersionReg);
            g_rc_present[r] = (v != 0x00 && v != 0xFF);
            bus_unlock(RES_SPI);
            ESP_LOGI(TAG, "RC522 %s: version 0x%02X (%s)", r == 0 ? "ENTRY" : "EXIT", v,
                     g_rc_present[r] ? "present" : "not detected - use CLI 'scan'");
        }
    }
}

/* ====================================================================== */
/* 1. ENTRANCE: access logic, doors, reader task                          */
/* ====================================================================== */

typedef struct { bool open; int64_t close_at_us; } door_t;
static door_t g_door[2];

static void door_open(dir_t d)
{
    if (bus_lock(RES_DATA, 200)) {
        g_door[d].close_at_us = esp_timer_get_time() + (int64_t)DOOR_OPEN_MS * 1000;
        bus_unlock(RES_DATA);
    }
}

static void door_task(void *arg)
{
    for (;;) {
        hb_beat(T_DOOR);
        bool emg = emergency_active();
        for (int d = 0; d < 2; d++) {
            bool changed = false, want = false;
            if (bus_lock(RES_DATA, 100)) {
                want = emg || esp_timer_get_time() < g_door[d].close_at_us;
                if (want != g_door[d].open) { g_door[d].open = want; changed = true; }
                bus_unlock(RES_DATA);
            }
            if (changed) {
                servo_write(d == 0 ? LEDC_CHANNEL_0 : LEDC_CHANNEL_1, want ? 90 : 0);
                log_event("DOOR", "%s;%s", d == 0 ? "ENTRY" : "EXIT", want ? "OPEN" : "CLOSED");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static user_t *users_find_locked(const char *uid)
{
    for (int i = 0; i < MAX_USERS; i++)
        if (g_users[i].used && strcmp(g_users[i].uid, uid) == 0) return &g_users[i];
    return NULL;
}

/* THE entry point for every card scan (reader task and CLI 'scan' both call this). */
acc_result_t access_handle_scan(dir_t dir, const char *uid_in)
{
    char uid[UID_LEN];
    snprintf(uid, sizeof uid, "%s", uid_in);
    for (char *p = uid; *p; p++) *p = toupper((unsigned char)*p);

    bool emg = emergency_active();
    bool sched_ok = emg || sched_is_open(time(NULL));
    int64_t now = esp_timer_get_time();
    acc_result_t res = ACC_DENIED_UNKNOWN;
    char name[NAME_LEN] = "-";
    bool lockout_started = false;
    int fails = 0;

    if (!bus_lock(RES_DATA, 500)) return ACC_DENIED_LOCKOUT;
    if (!emg && now < g_lockout_until[dir]) {
        res = ACC_DENIED_LOCKOUT;
        stats_bump_locked(2);
    } else {
        user_t *u = users_find_locked(uid);
        if (!u) {
            res = ACC_DENIED_UNKNOWN;
            fails = ++g_fail_count[dir];
            if (fails >= MAX_FAILS) {
                g_lockout_until[dir] = now + (int64_t)LOCKOUT_MS * 1000;
                g_fail_count[dir] = 0;
                lockout_started = true;
            }
            stats_bump_locked(2);
        } else {
            snprintf(name, sizeof name, "%s", u->name);
            g_fail_count[dir] = 0;
            if (!emg && !sched_ok)                              res = ACC_DENIED_SCHEDULE;
            else if (!emg && dir == DIR_ENTRY && u->inside)     res = ACC_DENIED_PASSBACK;
            else if (!emg && dir == DIR_EXIT  && !u->inside)    res = ACC_DENIED_PASSBACK;
            else {
                bool now_inside = (dir == DIR_ENTRY);
                if (u->inside != now_inside) { u->inside = now_inside; g_inside += now_inside ? 1 : -1; }
                if (now_inside) u->last_in = time(NULL);
                g_users_dirty = true;
                res = ACC_GRANTED;
            }
            stats_bump_locked(res == ACC_GRANTED ? (dir == DIR_ENTRY ? 0 : 1) : 2);
        }
    }
    bus_unlock(RES_DATA);

    static const char *RN[] = { "GRANTED", "DENIED_UNKNOWN", "DENIED_SCHEDULE", "DENIED_PASSBACK", "DENIED_LOCKOUT" };
    log_event("ACCESS", "%s;%s;%s;%s", dir == DIR_ENTRY ? "ENTRY" : "EXIT", RN[res], uid, name);
    if (res == ACC_GRANTED) { door_open(dir); buzz(BUZ_OK); }
    else buzz(BUZ_DENY);
    if (lockout_started) {
        raise_alert("LOCKOUT", "%s reader locked %ds after %d bad scans", dir == DIR_ENTRY ? "ENTRY" : "EXIT",
                    LOCKOUT_MS / 1000, MAX_FAILS);
        buzz(BUZ_LOCKOUT);
    }
    return res;
}

static void reader_task(void *arg)
{
    char last_uid[2][UID_LEN] = { "", "" };
    int64_t last_ms[2] = { 0, 0 };
    for (;;) {
        hb_beat(T_READER);
        for (int r = 0; r < 2; r++) {
            if (!g_rc_present[r]) continue;
            char uid[UID_LEN]; bool got = false;
            if (bus_lock(RES_SPI, 100)) { got = rc_read_uid(r, uid); bus_unlock(RES_SPI); }
            if (!got) continue;
            int64_t now = esp_timer_get_time() / 1000;
            if (strcmp(uid, last_uid[r]) == 0 && now - last_ms[r] < 2000) { last_ms[r] = now; continue; }
            snprintf(last_uid[r], UID_LEN, "%s", uid); last_ms[r] = now;
            access_handle_scan(r == 0 ? DIR_ENTRY : DIR_EXIT, uid);
        }
        vTaskDelay(pdMS_TO_TICKS(120));
    }
}

/* ====================================================================== */
/* 2. DATA: SD / FATFS, logger, users.csv                                 */
/* ====================================================================== */

static sdmmc_card_t *g_card;

static void spi_bus_setup(void)
{
    spi_bus_config_t bus = {
        .mosi_io_num = PIN_SPI_MOSI, .miso_io_num = PIN_SPI_MISO, .sclk_io_num = PIN_SPI_SCK,
        .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 4000,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));
}

static void sd_setup(void)
{
    esp_vfs_fat_sdmmc_mount_config_t mc = {
        .format_if_mount_failed = true, .max_files = 4, .allocation_unit_size = 16 * 1024,
    };
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    host.max_freq_khz = 10000;
    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs = PIN_SD_CS;
    slot.host_id = SPI2_HOST;
    esp_err_t e = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot, &mc, &g_card);
    g_sd_ok = (e == ESP_OK);
    if (g_sd_ok) ESP_LOGI(TAG, "SD mounted at %s", MOUNT_POINT);
    else ESP_LOGW(TAG, "SD mount failed (%s) - logging to RAM ring only", esp_err_to_name(e));
}

/* Caller holds RES_SD and RES_SPI. */
static void sd_rotate_locked(void)
{
    remove(LOG_PATH_1);
    rename(LOG_PATH, LOG_PATH_1);
}

static void sd_append_lines(char (*lines)[LOG_LINE], int n)
{
    if (!g_sd_ok || n == 0) return;
    if (!bus_lock(RES_SD, 1000)) return;
    if (!bus_lock(RES_SPI, 1000)) { bus_unlock(RES_SD); return; }
    struct stat st;
    bool fresh = (stat(LOG_PATH, &st) != 0);
    FILE *f = fopen(LOG_PATH, "a");
    if (f) {
        if (fresh) fputs("timestamp,event,detail\n", f);
        for (int i = 0; i < n; i++) { fputs(lines[i], f); fputc('\n', f); }
        long sz = ftell(f);
        fclose(f);
        if (sz > LOG_ROTATE_BYTES) sd_rotate_locked();
    } else {
        ESP_LOGW(TAG, "cannot open %s (errno %d: %s)", LOG_PATH, errno, strerror(errno));
    }
    bus_unlock(RES_SPI);
    bus_unlock(RES_SD);
}

static void users_save(void)
{
    static user_t snap[MAX_USERS];
    if (!bus_lock(RES_DATA, 500)) return;
    memcpy(snap, g_users, sizeof snap);
    g_users_dirty = false;
    bus_unlock(RES_DATA);
    if (!g_sd_ok) return;
    if (!bus_lock(RES_SD, 1000)) return;
    if (!bus_lock(RES_SPI, 1000)) { bus_unlock(RES_SD); return; }
    FILE *f = fopen(USERS_TMP, "w");
    if (f) {
        fputs("uid,name,inside\n", f);
        for (int i = 0; i < MAX_USERS; i++)
            if (snap[i].used) fprintf(f, "%s,%s,%d\n", snap[i].uid, snap[i].name, snap[i].inside ? 1 : 0);
        fclose(f);
        remove(USERS_PATH);
        rename(USERS_TMP, USERS_PATH);
    }
    bus_unlock(RES_SPI);
    bus_unlock(RES_SD);
}

static void users_seed(void)
{
    static const char *seed[][2] = {
        { "01:02:03:04", "Alice Namuli" }, { "05:06:07:08", "Brian Okello" },
        { "A1:B2:C3:D4", "Carol Nakato" }, { "DE:AD:BE:EF", "Dr Mukasa" },
    };
    for (int i = 0; i < 4; i++) {
        snprintf(g_users[i].uid, UID_LEN, "%s", seed[i][0]);
        snprintf(g_users[i].name, NAME_LEN, "%s", seed[i][1]);
        g_users[i].used = true;
    }
}

static void users_load(void)
{
    static user_t tmp[MAX_USERS];
    memset(tmp, 0, sizeof tmp);
    int n = 0;
    bool loaded = false;
    if (g_sd_ok && bus_lock(RES_SD, 1000)) {
        if (bus_lock(RES_SPI, 1000)) {
            FILE *f = fopen(USERS_PATH, "r");
            if (f) {
                char line[96];
                while (fgets(line, sizeof line, f) && n < MAX_USERS) {
                    line[strcspn(line, "\r\n")] = 0;
                    char *c1 = strchr(line, ',');
                    if (!c1 || strncmp(line, "uid,", 4) == 0) continue;
                    *c1++ = 0;
                    char *c2 = strchr(c1, ',');
                    int inside = 0;
                    if (c2) { *c2++ = 0; inside = atoi(c2); }
                    snprintf(tmp[n].uid, UID_LEN, "%.11s", line);
                    snprintf(tmp[n].name, NAME_LEN, "%.23s", c1);
                    tmp[n].inside = inside != 0;
                    tmp[n].used = true;
                    n++;
                }
                fclose(f);
                loaded = n > 0;
            }
            bus_unlock(RES_SPI);
        }
        bus_unlock(RES_SD);
    }
    if (bus_lock(RES_DATA, 500)) {
        if (loaded) {
            memcpy(g_users, tmp, sizeof g_users);
            g_inside = 0;
            for (int i = 0; i < MAX_USERS; i++) if (g_users[i].used && g_users[i].inside) g_inside++;
        } else {
            memset(g_users, 0, sizeof g_users);
            users_seed();
            g_users_dirty = true;
        }
        bus_unlock(RES_DATA);
    }
    ESP_LOGI(TAG, "users: %s", loaded ? "loaded from users.csv" : "seeded defaults");
}

static void logger_task(void *arg)
{
    static char batch[10][LOG_LINE];
    int64_t last_save = 0;
    for (;;) {
        hb_beat(T_LOGGER);
        log_item_t it;
        if (xQueueReceive(g_q_log, &it, pdMS_TO_TICKS(500)) == pdTRUE) {
            int n = 0;
            memcpy(batch[n++], it.line, LOG_LINE);
            while (n < 10 && xQueueReceive(g_q_log, &it, 0) == pdTRUE) memcpy(batch[n++], it.line, LOG_LINE);
            if (bus_lock(RES_LOG, 200)) {
                for (int i = 0; i < n; i++) { memcpy(g_ring[g_ring_n % RING_N], batch[i], LOG_LINE); g_ring_n++; }
                bus_unlock(RES_LOG);
            }
            if (g_echo_log) for (int i = 0; i < n; i++) printf("[LOG] %s\n", batch[i]);
            sd_append_lines(batch, n);
        }
        int64_t now = esp_timer_get_time();
        if (now - last_save > 2000000) {
            last_save = now;
            if (g_users_dirty) users_save();
            if (g_stats_dirty) {
                static day_stat_t snap[7];
                if (bus_lock(RES_DATA, 200)) { memcpy(snap, g_stats, sizeof snap); g_stats_dirty = false; bus_unlock(RES_DATA); }
                if (bus_lock(RES_CFG, 300)) { nvs_set_blob(g_nvs, "stats", snap, sizeof snap); nvs_commit(g_nvs); bus_unlock(RES_CFG); }
            }
        }
    }
}

/* ---- log reading helpers (used by CLI) ---- */
static bool ci_contains(const char *hay, const char *needle)
{
    if (!needle || !*needle) return true;
    size_t nl = strlen(needle);
    for (; *hay; hay++) {
        size_t i = 0;
        while (i < nl && hay[i] && tolower((unsigned char)hay[i]) == tolower((unsigned char)needle[i])) i++;
        if (i == nl) return true;
    }
    return false;
}

/* Keeps the last `max` matching lines. Returns number printed. */
static int log_print_matches(const char *needle, int max)
{
    char (*ring)[LOG_LINE] = malloc((size_t)max * LOG_LINE);
    if (!ring) { printf("out of memory\n"); return 0; }
    int total = 0;
    if (g_sd_ok) {
        if (bus_lock(RES_SD, 2000)) {
            if (bus_lock(RES_SPI, 2000)) {
                FILE *f = fopen(LOG_PATH, "r");
                if (f) {
                    char line[LOG_LINE];
                    while (fgets(line, sizeof line, f)) {
                        line[strcspn(line, "\r\n")] = 0;
                        if (strncmp(line, "timestamp,", 10) == 0) continue;
                        if (ci_contains(line, needle)) snprintf(ring[total++ % max], LOG_LINE, "%s", line);
                    }
                    fclose(f);
                }
                bus_unlock(RES_SPI);
            }
            bus_unlock(RES_SD);
        }
    } else if (bus_lock(RES_LOG, 500)) {
        int cnt = g_ring_n < RING_N ? g_ring_n : RING_N;
        for (int i = 0; i < cnt; i++) {
            int idx = (g_ring_n - cnt + i) % RING_N;
            if (ci_contains(g_ring[idx], needle)) snprintf(ring[total++ % max], LOG_LINE, "%s", g_ring[idx]);
        }
        bus_unlock(RES_LOG);
    }
    int cnt = total < max ? total : max;
    for (int i = 0; i < cnt; i++) printf("%s\n", ring[(total - cnt + i) % max]);
    free(ring);
    return cnt;
}

/* ====================================================================== */
/* 2. AUTH (hashed password, session timeout, login lockout)              */
/* ====================================================================== */

static void pw_hash(const uint8_t salt[16], const char *pw, uint8_t out[32])
{
    mbedtls_sha256_context c; uint8_t h[32];
    mbedtls_sha256_init(&c);
    mbedtls_sha256_starts(&c, 0);
    mbedtls_sha256_update(&c, salt, 16);
    mbedtls_sha256_update(&c, (const uint8_t *)pw, strlen(pw));
    mbedtls_sha256_finish(&c, h);
    for (int i = 0; i < 2000; i++) {
        mbedtls_sha256_starts(&c, 0);
        mbedtls_sha256_update(&c, h, 32);
        mbedtls_sha256_update(&c, salt, 16);
        mbedtls_sha256_update(&c, (const uint8_t *)pw, strlen(pw));
        mbedtls_sha256_finish(&c, h);
    }
    mbedtls_sha256_free(&c);
    memcpy(out, h, 32);
}

static void pw_store(const char *pw)
{
    uint8_t salt[16], h[32];
    esp_fill_random(salt, sizeof salt);
    pw_hash(salt, pw, h);
    if (bus_lock(RES_CFG, 500)) {
        nvs_set_blob(g_nvs, "pw_salt", salt, 16);
        nvs_set_blob(g_nvs, "pw_hash", h, 32);
        nvs_commit(g_nvs);
        bus_unlock(RES_CFG);
    }
}

static bool pw_check(const char *pw)
{
    uint8_t salt[16], stored[32], h[32];
    size_t a = 16, b = 32;
    bool ok = false;
    if (bus_lock(RES_CFG, 500)) {
        ok = nvs_get_blob(g_nvs, "pw_salt", salt, &a) == ESP_OK && nvs_get_blob(g_nvs, "pw_hash", stored, &b) == ESP_OK;
        bus_unlock(RES_CFG);
    }
    if (!ok) return false;
    pw_hash(salt, pw, h);
    uint8_t diff = 0;
    for (int i = 0; i < 32; i++) diff |= h[i] ^ stored[i];      /* constant time */
    return diff == 0;
}

static void auth_init(void)
{
    uint8_t tmp[32]; size_t sz = 32; bool have = false;
    if (bus_lock(RES_CFG, 500)) { have = nvs_get_blob(g_nvs, "pw_hash", tmp, &sz) == ESP_OK; bus_unlock(RES_CFG); }
    if (!have) {
        pw_store(DEFAULT_ADMIN_PW);
        printf("\n*** First boot: admin password is '%s'. Change it with 'passwd'. ***\n", DEFAULT_ADMIN_PW);
    }
}

/* ====================================================================== */
/* 4. ROOMS                                                               */
/* ====================================================================== */

static adc_oneshot_unit_handle_t g_adc;

static int sonar_read_cm(int room)
{
    int trig = room == 0 ? PIN_SONAR1_TRIG : PIN_SONAR2_TRIG;
    int echo = room == 0 ? PIN_SONAR1_ECHO : PIN_SONAR2_ECHO;
    gpio_set_level(trig, 0); esp_rom_delay_us(4);
    gpio_set_level(trig, 1); esp_rom_delay_us(10);
    gpio_set_level(trig, 0);
    int64_t t = esp_timer_get_time();
    while (!gpio_get_level(echo)) if (esp_timer_get_time() - t > 30000) return -1;
    int64_t s = esp_timer_get_time();
    while (gpio_get_level(echo)) if (esp_timer_get_time() - s > 25000) return -1;
    return (int)((esp_timer_get_time() - s) / 58);
}

static bool ldr_bright(void)
{
    bool b = g_ldr_raw > LDR_BRIGHT_RAW;
    return LDR_INVERT ? !b : b;
}

static void rooms_task(void *arg)
{
    (void)arg;
    const int led[2] = { PIN_LED_R1, PIN_LED_R2 };
    bool present[2] = { false, false };
    bool light[2] = { false, false };
    bool previous_present[2] = { false, false };
    bool previous_light[2] = { false, false };
    bool daylight_logged[2] = { false, false };
    bool presence_history[2][3] = { { false } };
    int history_index[2] = { 0, 0 };
    int64_t last_seen[2] = { 0, 0 };
    int64_t last_sonar[2] = { 0, 0 };
    int64_t saved_ms[2] = { 0, 0 };
    int dist[2] = { -1, -1 };
    int64_t previous_ms = esp_timer_get_time() / 1000;
    bool open_hours = true;
    int64_t last_schedule_check = 0;

    for (;;) {
        hb_beat(T_ROOMS);
        vTaskDelay(pdMS_TO_TICKS(250));

        int64_t now = esp_timer_get_time() / 1000;
        int64_t elapsed_ms = now - previous_ms;
        previous_ms = now;
        bool emg = emergency_active();
        bool bright = ldr_bright();
        if (now - last_schedule_check >= 5000) {
            open_hours = sched_is_open(time(NULL));
            last_schedule_check = now;
        }

        for (int room = 0; room < 2; room++) {
            /* Read each HC-SR04 and classify a person within 150 cm. */
            if (now - last_sonar[room] >= 250) {
                last_sonar[room] = now;
                dist[room] = sonar_read_cm(room);

                /* Ignore timeouts and require two of three readings to agree. */
                if (dist[room] > 0) {
                    int slot = history_index[room]++ % 3;
                    presence_history[room][slot] =
                        dist[room] < ROOM_PRESENCE_CM;
                    int votes = presence_history[room][0] +
                                presence_history[room][1] +
                                presence_history[room][2];
                    present[room] = votes >= 2;
                }
            }

            if (emg) {
                last_seen[room] = now;
                light[room] = true;
            } else if (present[room]) {
                last_seen[room] = now;
                if (!bright) {
                    light[room] = true;
                    daylight_logged[room] = false;
                } else if (!daylight_logged[room]) {
                    daylight_logged[room] = true;
                    log_event("ROOM", "R%d;DAYLIGHT_SKIP;ldr=%d", room + 1, g_ldr_raw);
                }
            } else {
                daylight_logged[room] = false;
                if (light[room] && now - last_seen[room] >= ROOM_OFF_DELAY_MS) {
                    light[room] = false;
                    if (bus_lock(RES_DATA, 100)) {
                        g_rooms[room].auto_offs++;
                        bus_unlock(RES_DATA);
                    }
                }
            }

            int remaining_ms = -1;
            if (light[room] && !present[room] && !emg) {
                remaining_ms = (int)(ROOM_OFF_DELAY_MS - (now - last_seen[room]));
            }

            bool output = light[room];
            if (remaining_ms >= 0 && remaining_ms <= ROOM_WARN_MS) {
                output = (now / 250) % 2 == 0;
            }
            gpio_set_level(led[room], output);

            if (!light[room] && open_hours) {
                saved_ms[room] += elapsed_ms;
            }

            if (bus_lock(RES_DATA, 50)) {
                g_rooms[room].dist_cm = dist[room];
                g_rooms[room].off_in_s = remaining_ms >= 0 ? (remaining_ms + 999) / 1000 : -1;
                g_rooms[room].saved_s += (uint32_t)(saved_ms[room] / 1000);
                bus_unlock(RES_DATA);
                saved_ms[room] %= 1000;
            }

            if (present[room] != previous_present[room] ||
                light[room] != previous_light[room]) {
                previous_present[room] = present[room];
                previous_light[room] = light[room];
                msg_t m = { MSG_ROOM_STATE, (uint8_t)room, present[room], light[room] };
                xQueueSend(g_q_msg, &m, 0);
            }
        }
    }
}

/* ====================================================================== */
/* 5. EMERGENCY, GHOST OCCUPANCY, COORDINATOR, IO                         */
/* ====================================================================== */

static void emergency_set(bool on, const char *who)
{
    if (on == emergency_active()) return;
    if (on) xEventGroupSetBits(g_ev, EV_EMERGENCY); else xEventGroupClearBits(g_ev, EV_EMERGENCY);
    log_event("EMERGENCY", "%s;by=%s", on ? "ACTIVATED" : "CLEARED", who);
    if (!on) return;
    /* log_event() takes no lock, so it is safe to call while holding RES_DATA */
    if (bus_lock(RES_DATA, 300)) {
        log_event("EVAC", "roster;count=%d", g_inside);
        for (int i = 0; i < MAX_USERS; i++)
            if (g_users[i].used && g_users[i].inside) log_event("EVAC", "inside;%s;%s", g_users[i].uid, g_users[i].name);
        bus_unlock(RES_DATA);
    }
}

static void coord_task(void *arg)
{
    int64_t ghost_since[2] = { 0, 0 };
    bool ghost_alerted[2] = { 0, 0 };
    for (;;) {
        hb_beat(T_COORD);
        msg_t m;
        while (xQueueReceive(g_q_msg, &m, pdMS_TO_TICKS(250)) == pdTRUE) {
            if (m.type == MSG_ROOM_STATE && m.room < 2) {
                if (bus_lock(RES_DATA, 100)) { g_rooms[m.room].presence = m.presence; g_rooms[m.room].light = m.light; bus_unlock(RES_DATA); }
                log_event("ROOM", "R%d;presence=%d;light=%d", m.room + 1, m.presence, m.light);
            }
        }
        int inside = 0; bool pres[2] = { 0, 0 };
        if (bus_lock(RES_DATA, 100)) {
            inside = g_inside; pres[0] = g_rooms[0].presence; pres[1] = g_rooms[1].presence;
            bus_unlock(RES_DATA);
        }
        int64_t now = esp_timer_get_time() / 1000;
        for (int i = 0; i < 2; i++) {
            if (pres[i] && inside == 0 && !emergency_active()) {
                if (ghost_since[i] == 0) ghost_since[i] = now;
                if (!ghost_alerted[i] && now - ghost_since[i] >= GHOST_CONFIRM_MS) {
                    ghost_alerted[i] = true;
                    raise_alert("GHOST_OCCUPANCY", "Room %d active but building count is 0 - possible intruder", i + 1);
                }
            } else {
                ghost_since[i] = 0;
                if (!pres[i]) ghost_alerted[i] = false;
            }
        }
    }
}

static void io_task(void *arg)
{
    int low = 0; int64_t last_ldr = 0;
    for (;;) {
        hb_beat(T_IO);
        low = gpio_get_level(PIN_EMERG_BTN) == 0 ? low + 1 : 0;
        if (low == 3) emergency_set(true, "BUTTON");
        int64_t now = esp_timer_get_time() / 1000;
        if (now - last_ldr > 500) {
            last_ldr = now;
            int raw;
            if (adc_oneshot_read(g_adc, LDR_ADC_CHANNEL, &raw) == ESP_OK) g_ldr_raw = raw;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ---- supervisor ---- */
static void supervisor_task(void *arg)
{
    bool alerted[T_COUNT] = { 0 };
    uint32_t last_dbg = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        uint32_t now = xTaskGetTickCount();
        for (int i = 0; i < T_COUNT; i++) {
            uint32_t age = (now - g_hb[i]) * portTICK_PERIOD_MS;
            if (age > TASK_TIMEOUT_MS && !alerted[i]) {
                alerted[i] = true;
                raise_alert("TASK_SILENT", "task '%s' silent for %us", TASK_NAME[i], (unsigned)(age / 1000));
            } else if (age <= TASK_TIMEOUT_MS && alerted[i]) {
                alerted[i] = false;
                log_event("SUPERVISOR", "task '%s' recovered", TASK_NAME[i]);
            }
        }
        (void)last_dbg;
    }
}

/* ====================================================================== */
/* 5. OLED (SSD1306 128x64 over I2C) + dashboard                          */
/* ====================================================================== */

static uint8_t g_fb[128 * 8];

static const uint8_t FONT5x7[59][5] = {   /* ASCII 0x20..0x5A */
    {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},{0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},{0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00},{0x00,0x41,0x22,0x1C,0x00},{0x14,0x08,0x3E,0x08,0x14},{0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},{0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},{0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},{0x00,0x36,0x36,0x00,0x00},{0x00,0x56,0x36,0x00,0x00},
    {0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},{0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E},{0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},{0x3E,0x41,0x49,0x49,0x7A},
    {0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},{0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40},{0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},
    {0x63,0x14,0x08,0x14,0x63},{0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43},
};

static void oled_cmd(uint8_t c)
{
    uint8_t b[2] = { 0x00, c };
    i2c_master_write_to_device(I2C_NUM_0, OLED_ADDR, b, 2, pdMS_TO_TICKS(100));
}

static void oled_init(void)
{
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER, .sda_io_num = PIN_I2C_SDA, .scl_io_num = PIN_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE, .scl_pullup_en = GPIO_PULLUP_ENABLE, .master.clk_speed = 400000,
    };
    i2c_param_config(I2C_NUM_0, &cfg);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    static const uint8_t init[] = { 0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14, 0x20, 0x00,
                                    0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF };
    for (size_t i = 0; i < sizeof init; i++) oled_cmd(init[i]);
}

static void oled_row(int row, const char *s)
{
    uint8_t *p = &g_fb[row * 128];
    memset(p, 0, 128);
    for (int i = 0; s[i] && i < 21; i++) {
        int c = toupper((unsigned char)s[i]);
        if (c < 0x20 || c > 0x5A) c = '?';
        memcpy(p + i * 6, FONT5x7[c - 0x20], 5);
    }
}

static void oled_flush(void)
{
    static const uint8_t win[] = { 0x21, 0, 127, 0x22, 0, 7 };
    for (size_t i = 0; i < sizeof win; i++) oled_cmd(win[i]);
    uint8_t buf[129]; buf[0] = 0x40;
    for (int pg = 0; pg < 8; pg++) {
        memcpy(buf + 1, &g_fb[pg * 128], 128);
        i2c_master_write_to_device(I2C_NUM_0, OLED_ADDR, buf, 129, pdMS_TO_TICKS(100));
    }
}

static void display_task(void *arg)
{
    for (;;) {
        hb_beat(T_DISPLAY);
        char l[24]; char ts[24];
        fmt_time(time(NULL), ts, sizeof ts);
        int inside = 0, alerts = 0; room_t r[2]; int64_t lock0, lock1;
        if (bus_lock(RES_DATA, 100)) {
            inside = g_inside; alerts = g_alert_total; r[0] = g_rooms[0]; r[1] = g_rooms[1];
            lock0 = g_lockout_until[0]; lock1 = g_lockout_until[1];
            bus_unlock(RES_DATA);
        } else { vTaskDelay(pdMS_TO_TICKS(500)); continue; }
        int64_t now = esp_timer_get_time();
        bool emg = emergency_active(), lk = now < lock0 || now < lock1, open = sched_is_open(time(NULL));
        uint16_t s, e;
        if (bus_lock(RES_CFG, 100)) { s = g_sched_start; e = g_sched_end; bus_unlock(RES_CFG); } else { s = 0; e = 0; }

        if (bus_lock(RES_I2C, 200)) {
            snprintf(l, sizeof l, "COCIS %s", ts + 11);               oled_row(0, l);
            snprintf(l, sizeof l, "MODE: %s", emg ? "FIRE EVACUATE" : lk ? "LOCKOUT" : "NORMAL"); oled_row(1, l);
            snprintf(l, sizeof l, "INSIDE: %d", inside);               oled_row(2, l);
            snprintf(l, sizeof l, "%02d:%02d-%02d:%02d %s", s / 60, s % 60, e / 60, e % 60, open ? "OPEN" : "SHUT"); oled_row(3, l);
            for (int i = 0; i < 2; i++) {
                snprintf(l, sizeof l, "R%d OCC:%s LIGHT:%s", i + 1, r[i].presence ? "Y" : "N", r[i].light ? "ON" : "OFF");
                oled_row(4 + i, l);
            }
            snprintf(l, sizeof l, "ALERTS:%d SD:%s", alerts, g_sd_ok ? "OK" : "NO"); oled_row(6, l);
            uint32_t cwh = (r[0].saved_s + r[1].saved_s) * ROOM_LIGHT_WATTS / 36;   /* centi-Wh */
            snprintf(l, sizeof l, "SAVED: %u.%02u WH", (unsigned)(cwh / 100), (unsigned)(cwh % 100)); oled_row(7, l);
            oled_flush();
            bus_unlock(RES_I2C);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ====================================================================== */
/* 2. CLI                                                                 */
/* ====================================================================== */

typedef enum { CLI_NORMAL, CLI_LOGIN_PW, CLI_PW_NEW, CLI_PW_CONFIRM } cli_mode_t;
static cli_mode_t g_cli_mode = CLI_NORMAL;
static bool    g_admin;
static int64_t g_admin_until;
static int     g_bad_logins;
static int64_t g_login_locked_until;
static char    g_pw_new[40];

static bool cli_admin(void)
{
    if (!g_admin) return false;
    int64_t now = esp_timer_get_time();
    if (now > g_admin_until) { g_admin = false; printf("(session expired)\n"); log_event("AUTH", "SESSION_TIMEOUT"); return false; }
    g_admin_until = now + (int64_t)SESSION_MS * 1000;
    return true;
}
#define NEED_ADMIN() do { if (!cli_admin()) { printf("ERROR: admin login required (use 'login')\n"); return; } } while (0)

static void print_help(void)
{
    printf("\nCommands:\n"
           "  help                         this list\n"
           "  status                       system summary\n"
           "  inside                       who is in the building (evacuation roster)\n"
           "  rooms                        room occupancy, lights, energy saved\n"
           "  log [n]                      last n log lines (default 10)\n"
           "  log search <text>            search by tag/uid/date (e.g. 2026-10-31)\n"
           "  counters                     entries/exits/denied per day\n"
           "  alerts                       recent alerts\n"
           "  tasks                        task heartbeat ages\n"
           "  users                        list registered cards\n"
           "  schedule                     show access schedule\n"
           "  time                         show clock\n"
           "  scan <in|out> <uid>          simulate an RFID scan (for Wokwi)\n"
           "  emergency [on]               trigger fire mode\n"
           "  echo <on|off>                live log echo on serial\n"
           "  login / logout / whoami\n"
           " ADMIN ONLY:\n"
           "  schedule set HH:MM HH:MM     change access window\n"
           "  time set YYYY-MM-DD HH:MM[:SS]\n"
           "  emergency clear              end fire mode\n"
           "  useradd <uid> <name> / userdel <uid>\n"
           "  passwd                       change admin password\n"
           "  log rotate\n"
           "  hang <n> / unhang <n>        simulate a silent task (supervisor demo)\n"
           "  stress                       concurrency self-test\n\n");
}

static void cmd_status(void)
{
    char ts[24]; fmt_time(time(NULL), ts, sizeof ts);
    uint16_t s, e;
    bus_lock(RES_CFG, 200); s = g_sched_start; e = g_sched_end; bus_unlock(RES_CFG);
    int inside, alerts; room_t r[2];
    bus_lock(RES_DATA, 200); inside = g_inside; alerts = g_alert_total; r[0] = g_rooms[0]; r[1] = g_rooms[1]; bus_unlock(RES_DATA);
    printf("Time        : %s\n", ts);
    printf("Schedule    : %02d:%02d-%02d:%02d (%s now)\n", s / 60, s % 60, e / 60, e % 60, sched_is_open(time(NULL)) ? "OPEN" : "CLOSED");
    printf("Mode        : %s\n", emergency_active() ? "EMERGENCY / FIRE" : "normal");
    printf("Inside      : %d\n", inside);
    for (int i = 0; i < 2; i++)
        printf("Room %d      : %s, light %s\n", i + 1, r[i].presence ? "occupied" : "empty", r[i].light ? "ON" : "OFF");
    printf("Readers     : entry %s, exit %s\n", g_rc_present[0] ? "RC522" : "sim-only", g_rc_present[1] ? "RC522" : "sim-only");
    printf("SD card     : %s | log dropped: %u\n", g_sd_ok ? "OK" : "NOT AVAILABLE", (unsigned)g_log_dropped);
    printf("Alerts      : %d total | lock timeouts D/C/L/S/P/I: %u/%u/%u/%u/%u/%u\n", alerts,
           (unsigned)g_lock_timeouts[0], (unsigned)g_lock_timeouts[1], (unsigned)g_lock_timeouts[2],
           (unsigned)g_lock_timeouts[3], (unsigned)g_lock_timeouts[4], (unsigned)g_lock_timeouts[5]);
    printf("Admin       : %s | LDR raw %d (%s) | heap %u | uptime %llus\n", g_admin ? "logged in" : "logged out",
           g_ldr_raw, ldr_bright() ? "daylight" : "dark", (unsigned)esp_get_free_heap_size(),
           (unsigned long long)(esp_timer_get_time() / 1000000));
}

static void cmd_inside(void)
{
    static user_t snap[MAX_USERS];
    bus_lock(RES_DATA, 300); memcpy(snap, g_users, sizeof snap); bus_unlock(RES_DATA);
    int n = 0;
    printf("UID          NAME                  SINCE\n");
    for (int i = 0; i < MAX_USERS; i++) {
        if (!snap[i].used || !snap[i].inside) continue;
        char ts[24]; fmt_time(snap[i].last_in, ts, sizeof ts);
        printf("%-12s %-21s %s\n", snap[i].uid, snap[i].name, ts);
        n++;
    }
    printf("Total inside: %d\n", n);
}

static void cmd_rooms(void)
{
    room_t r[2];
    bus_lock(RES_DATA, 300); r[0] = g_rooms[0]; r[1] = g_rooms[1]; bus_unlock(RES_DATA);
    for (int i = 0; i < 2; i++) {
        uint32_t cwh = r[i].saved_s * ROOM_LIGHT_WATTS / 36;
        printf("Room %d: %-8s light %-3s dist %d cm  off-in %d s  auto-offs %u  saved %u.%02u Wh\n", i + 1,
               r[i].presence ? "OCCUPIED" : "empty", r[i].light ? "ON" : "OFF", r[i].dist_cm, r[i].off_in_s,
               (unsigned)r[i].auto_offs, (unsigned)(cwh / 100), (unsigned)(cwh % 100));
    }
    printf("Daylight: %s (LDR %d)\n", ldr_bright() ? "bright, lights stay off" : "dark", g_ldr_raw);
}

static void cmd_counters(void)
{
    day_stat_t s[7];
    bus_lock(RES_DATA, 300); memcpy(s, g_stats, sizeof s); bus_unlock(RES_DATA);
    printf("DATE       ENTRIES EXITS DENIED\n");
    for (int i = 0; i < 7; i++)
        if (s[i].date) printf("%08d   %5u %5u %6u\n", s[i].date, s[i].entries, s[i].exits, s[i].denied);
}

static bool parse_uid(const char *in, char *out)
{
    unsigned a, b, c, d;
    if (sscanf(in, "%2x:%2x:%2x:%2x", &a, &b, &c, &d) != 4) return false;
    snprintf(out, UID_LEN, "%02X:%02X:%02X:%02X", a, b, c, d);
    return true;
}

static void cli_exec(char *line)
{
    char *argv[8]; int argc = 0, st = 0; (void)st;
    char *sp; char *tok = strtok_r(line, " \t", &sp);
    while (tok && argc < 8) { argv[argc++] = tok; tok = strtok_r(NULL, " \t", &sp); }
    if (!argc) return;
    const char *c = argv[0];

    if (!strcmp(c, "help") || !strcmp(c, "?")) print_help();
    else if (!strcmp(c, "status")) cmd_status();
    else if (!strcmp(c, "inside")) cmd_inside();
    else if (!strcmp(c, "rooms")) cmd_rooms();
    else if (!strcmp(c, "counters")) cmd_counters();
    else if (!strcmp(c, "tasks")) {
        uint32_t now = xTaskGetTickCount();
        for (int i = 0; i < T_COUNT; i++)
            printf("%-8s last heartbeat %u ms ago%s\n", TASK_NAME[i], (unsigned)((now - g_hb[i]) * portTICK_PERIOD_MS),
                   (g_hang_mask & (1u << i)) ? "  [HANG SIMULATED]" : "");
    }
    else if (!strcmp(c, "alerts")) {
        bus_lock(RES_DATA, 300);
        int total = g_alert_total, n = total < 8 ? total : 8;
        for (int i = 0; i < n; i++) printf("%s\n", g_alerts[(total - n + i) % 8]);
        bus_unlock(RES_DATA);
        if (!total) printf("no alerts\n");
    }
    else if (!strcmp(c, "log")) {
        if (argc >= 2 && !strcmp(argv[1], "search")) {
            if (argc < 3) { printf("usage: log search <text>\n"); return; }
            int n = log_print_matches(argv[2], 30);
            printf("-- %d match(es), showing up to 30 most recent --\n", n);
        } else if (argc >= 2 && !strcmp(argv[1], "rotate")) {
            NEED_ADMIN();
            if (g_sd_ok && bus_lock(RES_SD, 1000)) {
                if (bus_lock(RES_SPI, 1000)) { sd_rotate_locked(); bus_unlock(RES_SPI); }
                bus_unlock(RES_SD);
            }
            log_event("ADMIN", "log rotated manually");
            printf("log rotated -> access_log.1.csv\n");
        } else {
            int n = argc >= 2 ? atoi(argv[1]) : 10;
            if (n < 1) n = 10;
            if (n > 40) n = 40;
            log_print_matches(NULL, n);
        }
    }
    else if (!strcmp(c, "users")) {
        static user_t snap[MAX_USERS];
        bus_lock(RES_DATA, 300); memcpy(snap, g_users, sizeof snap); bus_unlock(RES_DATA);
        for (int i = 0; i < MAX_USERS; i++)
            if (snap[i].used) printf("%-12s %-22s %s\n", snap[i].uid, snap[i].name, snap[i].inside ? "inside" : "outside");
    }
    else if (!strcmp(c, "useradd")) {
        NEED_ADMIN();
        char uid[UID_LEN];
        if (argc < 3 || !parse_uid(argv[1], uid)) { printf("usage: useradd <AA:BB:CC:DD> <name>\n"); return; }
        char name[NAME_LEN] = "";
        for (int i = 2; i < argc; i++) {
            if (i > 2) strncat(name, " ", sizeof name - strlen(name) - 1);
            strncat(name, argv[i], sizeof name - strlen(name) - 1);
        }
        for (char *p = name; *p; p++) if (*p == ',') *p = ' ';
        bool ok = false;
        bus_lock(RES_DATA, 300);
        if (!users_find_locked(uid))
            for (int i = 0; i < MAX_USERS; i++)
                if (!g_users[i].used) {
                    memset(&g_users[i], 0, sizeof g_users[i]);
                    snprintf(g_users[i].uid, UID_LEN, "%s", uid);
                    snprintf(g_users[i].name, NAME_LEN, "%s", name);
                    g_users[i].used = true; g_users_dirty = true; ok = true; break;
                }
        bus_unlock(RES_DATA);
        printf(ok ? "added %s\n" : "failed (duplicate or table full)\n", uid);
        if (ok) log_event("ADMIN", "useradd;%s;%s", uid, name);
    }
    else if (!strcmp(c, "userdel")) {
        NEED_ADMIN();
        char uid[UID_LEN];
        if (argc < 2 || !parse_uid(argv[1], uid)) { printf("usage: userdel <uid>\n"); return; }
        bool ok = false;
        bus_lock(RES_DATA, 300);
        user_t *u = users_find_locked(uid);
        if (u) { if (u->inside) g_inside--; memset(u, 0, sizeof *u); g_users_dirty = true; ok = true; }
        bus_unlock(RES_DATA);
        printf(ok ? "deleted %s\n" : "no such user\n", uid);
        if (ok) log_event("ADMIN", "userdel;%s", uid);
    }
    else if (!strcmp(c, "schedule")) {
        if (argc >= 2 && !strcmp(argv[1], "set")) {
            NEED_ADMIN();
            int h1, m1, h2, m2;
            if (argc < 4 || sscanf(argv[2], "%d:%d", &h1, &m1) != 2 || sscanf(argv[3], "%d:%d", &h2, &m2) != 2 ||
                h1 < 0 || h1 > 23 || h2 < 0 || h2 > 23 || m1 < 0 || m1 > 59 || m2 < 0 || m2 > 59) {
                printf("usage: schedule set HH:MM HH:MM\n"); return;
            }
            bus_lock(RES_CFG, 300); g_sched_start = h1 * 60 + m1; g_sched_end = h2 * 60 + m2; bus_unlock(RES_CFG);
            sched_save();
            log_event("ADMIN", "schedule set;%02d:%02d-%02d:%02d", h1, m1, h2, m2);
            printf("schedule saved to NVS: %02d:%02d-%02d:%02d\n", h1, m1, h2, m2);
        } else {
            uint16_t s, e;
            bus_lock(RES_CFG, 200); s = g_sched_start; e = g_sched_end; bus_unlock(RES_CFG);
            printf("Access window %02d:%02d - %02d:%02d, currently %s\n", s / 60, s % 60, e / 60, e % 60,
                   sched_is_open(time(NULL)) ? "OPEN" : "CLOSED (entry AND exit refused)");
        }
    }
    else if (!strcmp(c, "time")) {
        if (argc >= 2 && !strcmp(argv[1], "set")) {
            NEED_ADMIN();
            int Y, M, D, h = 0, m = 0, s = 0;
            if (argc < 4 || sscanf(argv[2], "%d-%d-%d", &Y, &M, &D) != 3 || sscanf(argv[3], "%d:%d:%d", &h, &m, &s) < 2) {
                printf("usage: time set YYYY-MM-DD HH:MM[:SS]\n"); return;
            }
            struct tm tm = { .tm_year = Y - 1900, .tm_mon = M - 1, .tm_mday = D, .tm_hour = h, .tm_min = m, .tm_sec = s, .tm_isdst = -1 };
            struct timeval tv = { .tv_sec = mktime(&tm) };
            settimeofday(&tv, NULL);
            log_event("ADMIN", "time set;%04d-%02d-%02d %02d:%02d:%02d", Y, M, D, h, m, s);
            printf("clock set\n");
        } else { char ts[24]; fmt_time(time(NULL), ts, sizeof ts); printf("%s (EAT)\n", ts); }
    }
    else if (!strcmp(c, "scan")) {
        char uid[UID_LEN];
        if (argc < 3 || !parse_uid(argv[2], uid) || (strcmp(argv[1], "in") && strcmp(argv[1], "out"))) {
            printf("usage: scan <in|out> <AA:BB:CC:DD>\n"); return;
        }
        static const char *msg[] = { "GRANTED", "DENIED: unknown card", "DENIED: outside access schedule",
                                     "DENIED: anti-passback", "DENIED: reader locked out" };
        acc_result_t r = access_handle_scan(!strcmp(argv[1], "in") ? DIR_ENTRY : DIR_EXIT, uid);
        printf("%s\n", msg[r]);
    }
    else if (!strcmp(c, "emergency")) {
        if (argc >= 2 && !strcmp(argv[1], "clear")) { NEED_ADMIN(); emergency_set(false, "admin"); printf("emergency cleared\n"); }
        else { emergency_set(true, "CLI"); printf("EMERGENCY ACTIVE - doors open, lights on\n"); }
    }
    else if (!strcmp(c, "echo")) { g_echo_log = (argc >= 2 && !strcmp(argv[1], "on")); printf("log echo %s\n", g_echo_log ? "on" : "off"); }
    else if (!strcmp(c, "login")) {
        if (esp_timer_get_time() < g_login_locked_until) {
            printf("login locked for %llds\n", (long long)((g_login_locked_until - esp_timer_get_time()) / 1000000)); return;
        }
        printf("Password: "); fflush(stdout); g_cli_mode = CLI_LOGIN_PW;
    }
    else if (!strcmp(c, "logout")) { g_admin = false; printf("logged out\n"); log_event("AUTH", "LOGOUT"); }
    else if (!strcmp(c, "whoami")) printf("%s\n", cli_admin() ? "admin" : "guest (read-only)");
    else if (!strcmp(c, "passwd")) { NEED_ADMIN(); printf("New password (min 6 chars): "); fflush(stdout); g_cli_mode = CLI_PW_NEW; }
    else if (!strcmp(c, "hang") || !strcmp(c, "unhang")) {
        NEED_ADMIN();
        int n = argc >= 2 ? atoi(argv[1]) : -1;
        if (n < 0 || n >= T_COUNT) { printf("task id 0..%d (see 'tasks')\n", T_COUNT - 1); return; }
        if (c[0] == 'h') g_hang_mask |= 1u << n; else g_hang_mask &= ~(1u << n);
        printf("task %s heartbeat %s\n", TASK_NAME[n], c[0] == 'h' ? "suppressed" : "restored");
    }
    else if (!strcmp(c, "stress")) {
        NEED_ADMIN();
        extern void run_stress(void);
        run_stress();
    }
    else printf("unknown command '%s' - type 'help'\n", c);
}

static void cli_line(char *line)
{
    switch (g_cli_mode) {
    case CLI_LOGIN_PW:
        g_cli_mode = CLI_NORMAL;
        if (pw_check(line)) {
            g_admin = true; g_admin_until = esp_timer_get_time() + (int64_t)SESSION_MS * 1000; g_bad_logins = 0;
            printf("login OK (session %d s idle timeout)\n", SESSION_MS / 1000);
            log_event("AUTH", "LOGIN_OK");
        } else {
            g_bad_logins++;
            log_event("AUTH", "LOGIN_FAIL;%d", g_bad_logins);
            if (g_bad_logins >= MAX_LOGIN_FAILS) {
                g_bad_logins = 0; g_login_locked_until = esp_timer_get_time() + (int64_t)LOGIN_LOCK_MS * 1000;
                raise_alert("AUTH_LOCKOUT", "3 bad admin logins, CLI login locked %ds", LOGIN_LOCK_MS / 1000);
                printf("too many failures - login locked for %d s\n", LOGIN_LOCK_MS / 1000);
            } else printf("wrong password (%d/%d)\n", g_bad_logins, MAX_LOGIN_FAILS);
        }
        return;
    case CLI_PW_NEW:
        if (strlen(line) < 6) { printf("too short, aborted\n"); g_cli_mode = CLI_NORMAL; return; }
        snprintf(g_pw_new, sizeof g_pw_new, "%s", line);
        printf("Confirm: "); fflush(stdout); g_cli_mode = CLI_PW_CONFIRM;
        return;
    case CLI_PW_CONFIRM:
        g_cli_mode = CLI_NORMAL;
        if (strcmp(g_pw_new, line) == 0) { pw_store(g_pw_new); printf("password changed\n"); log_event("AUTH", "PASSWORD_CHANGED"); }
        else printf("passwords differ, unchanged\n");
        memset(g_pw_new, 0, sizeof g_pw_new);
        return;
    default: break;
    }
    cli_exec(line);
}

static void cli_task(void *arg)
{
    char buf[96]; int n = 0; bool skip_lf = false;
    printf("\ncocis> "); fflush(stdout);
    for (;;) {
        hb_beat(T_CLI);
        if (g_admin && esp_timer_get_time() > g_admin_until) { g_admin = false; log_event("AUTH", "SESSION_TIMEOUT"); }
        uint8_t ch;
        if (uart_read_bytes(UART_NUM_0, &ch, 1, pdMS_TO_TICKS(100)) != 1) continue;
        if (ch == '\n' && skip_lf) { skip_lf = false; continue; }
        skip_lf = false;
        if (ch == '\r' || ch == '\n') {
            skip_lf = (ch == '\r');
            buf[n] = 0; n = 0; putchar('\n');
            cli_line(buf);
            printf("%scocis> ", g_cli_mode == CLI_NORMAL ? "" : "");
            if (g_cli_mode != CLI_NORMAL) printf("\r");
            fflush(stdout);
        } else if ((ch == 0x08 || ch == 0x7F) && n > 0) {
            n--; if (g_cli_mode == CLI_NORMAL) { printf("\b \b"); fflush(stdout); }
        } else if (ch >= 0x20 && ch < 0x7F && n < (int)sizeof buf - 1) {
            buf[n++] = (char)ch;
            if (g_cli_mode == CLI_NORMAL) { putchar(ch); fflush(stdout); }     /* no echo for passwords */
        }
    }
}

/* ---- stress test: two scanners + CLI reader + logger, all at once ---- */
static SemaphoreHandle_t g_stress_done;
static void stress_worker(void *arg)
{
    int base = (int)(intptr_t)arg;                 /* workers use disjoint cards: 0,1 and 2,3 */
    static const char *uids[4] = { "01:02:03:04", "05:06:07:08", "A1:B2:C3:D4", "DE:AD:BE:EF" };
    for (int i = 0; i < 40; i++) {
        access_handle_scan(DIR_ENTRY, uids[base * 2]);
        access_handle_scan(DIR_ENTRY, uids[base * 2 + 1]);
        access_handle_scan(DIR_EXIT, uids[base * 2]);
        access_handle_scan(DIR_EXIT, uids[base * 2 + 1]);
        vTaskDelay(1);
    }
    xSemaphoreGive(g_stress_done);
    vTaskDelete(NULL);
}

void run_stress(void)
{
    printf("stress: 2 scanner tasks x 160 scans while CLI reads occupancy and logger writes...\n");
    uint32_t to0 = 0; for (int i = 0; i < RES_COUNT; i++) to0 += g_lock_timeouts[i];
    uint32_t drop0 = g_log_dropped;
    bool old_echo = g_echo_log; g_echo_log = false;
    g_force_open = true;
    g_stress_done = xSemaphoreCreateCounting(2, 0);
    xTaskCreate(stress_worker, "st0", 4096, (void *)0, 5, NULL);
    xTaskCreate(stress_worker, "st1", 4096, (void *)1, 5, NULL);
    int reads = 0, bad = 0, done = 0;
    while (done < 2) {
        if (xSemaphoreTake(g_stress_done, 0) == pdTRUE) done++;
        bus_lock(RES_DATA, 200);
        int cnt = 0; for (int i = 0; i < MAX_USERS; i++) if (g_users[i].used && g_users[i].inside) cnt++;
        if (cnt != g_inside) bad++;
        bus_unlock(RES_DATA);
        reads++; vTaskDelay(1);
        hb_beat(T_CLI);
    }
    g_force_open = false; g_echo_log = old_echo;
    bus_lock(RES_DATA, 200);
    int cnt = 0; for (int i = 0; i < MAX_USERS; i++) if (g_users[i].used && g_users[i].inside) cnt++;
    int final_inside = g_inside;
    bus_unlock(RES_DATA);
    uint32_t to1 = 0; for (int i = 0; i < RES_COUNT; i++) to1 += g_lock_timeouts[i];
    printf("stress: %d CLI reads, %d inconsistent snapshots, final count %d (flags %d), lock timeouts %u, log drops %u\n",
           reads, bad, final_inside, cnt, (unsigned)(to1 - to0), (unsigned)(g_log_dropped - drop0));
    printf("stress: %s\n", (bad == 0 && cnt == final_inside && to1 == to0) ? "PASS" : "FAIL");
    log_event("TEST", "stress;inconsistent=%d;timeouts=%u", bad, (unsigned)(to1 - to0));
}

/* ====================================================================== */
/* app_main                                                               */
/* ====================================================================== */

static void ledc_setup(void)
{
    ledc_timer_config_t t0 = { .speed_mode = LEDC_LOW_SPEED_MODE, .timer_num = LEDC_TIMER_0,
                               .duty_resolution = LEDC_TIMER_14_BIT, .freq_hz = 50, .clk_cfg = LEDC_AUTO_CLK };
    ledc_timer_config(&t0);
    ledc_timer_config_t t1 = { .speed_mode = LEDC_LOW_SPEED_MODE, .timer_num = LEDC_TIMER_1,
                               .duty_resolution = LEDC_TIMER_13_BIT, .freq_hz = 2000, .clk_cfg = LEDC_AUTO_CLK };
    ledc_timer_config(&t1);
    ledc_channel_config_t ch[3] = {
        { .gpio_num = PIN_SERVO_IN,  .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0, .timer_sel = LEDC_TIMER_0 },
        { .gpio_num = PIN_SERVO_OUT, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_1, .timer_sel = LEDC_TIMER_0 },
        { .gpio_num = PIN_BUZZER,    .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_2, .timer_sel = LEDC_TIMER_1 },
    };
    for (int i = 0; i < 3; i++) ledc_channel_config(&ch[i]);
    servo_write(LEDC_CHANNEL_0, 0);
    servo_write(LEDC_CHANNEL_1, 0);
}

static void gpio_setup(void)
{
    gpio_config_t out = { .pin_bit_mask = (1ULL << PIN_LED_R1) | (1ULL << PIN_LED_R2) |
                                          (1ULL << PIN_SONAR1_TRIG) | (1ULL << PIN_SONAR2_TRIG), .mode = GPIO_MODE_OUTPUT };
    gpio_config(&out);
    gpio_config_t in = { .pin_bit_mask = (1ULL << PIN_SONAR1_ECHO) | (1ULL << PIN_SONAR2_ECHO), .mode = GPIO_MODE_INPUT };
    gpio_config(&in);
    gpio_config_t btn = { .pin_bit_mask = 1ULL << PIN_EMERG_BTN, .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE };
    gpio_config(&btn);
    adc_oneshot_unit_init_cfg_t uc = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&uc, &g_adc));
    adc_oneshot_chan_cfg_t cc = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc, LDR_ADC_CHANNEL, &cc));
}

static void clock_init(void)
{
    setenv("TZ", "EAT-3", 1); tzset();
    if (time(NULL) < 1700000000) {              /* no RTC: start from build time, fix with 'time set' */
        struct tm tm = { 0 };
        strptime(__DATE__ " " __TIME__, "%b %d %Y %H:%M:%S", &tm);
        tm.tm_isdst = -1;
        struct timeval tv = { .tv_sec = mktime(&tm) };
        settimeofday(&tv, NULL);
    }
}

void app_main(void)
{
    for (int i = 0; i < RES_COUNT; i++) g_locks[i] = xSemaphoreCreateMutex();
    g_q_log  = xQueueCreate(48, sizeof(log_item_t));
    g_q_msg  = xQueueCreate(16, sizeof(msg_t));
    g_q_buzz = xQueueCreate(8, sizeof(int));
    g_ev     = xEventGroupCreate();

    clock_init();
    config_load();
    auth_init();
    ledc_setup();
    gpio_setup();
    spi_bus_setup();
    rc522_setup();
    sd_setup();
    users_load();
    oled_init();

    uint32_t now = xTaskGetTickCount();
    for (int i = 0; i < T_COUNT; i++) g_hb[i] = now;

    xTaskCreate(logger_task,     "logger",  6144, NULL, 3, NULL);
    xTaskCreate(buzzer_task,     "buzzer",  2048, NULL, 3, NULL);
    xTaskCreate(door_task,       "door",    3072, NULL, 5, NULL);
    xTaskCreate(reader_task,     "reader",  4096, NULL, 5, NULL);
    xTaskCreate(rooms_task,      "rooms",   4096, NULL, 4, NULL);
    xTaskCreate(coord_task,      "coord",   4096, NULL, 4, NULL);
    xTaskCreate(io_task,         "io",      3072, NULL, 6, NULL);
    xTaskCreate(display_task,    "display", 4096, NULL, 1, NULL);
    xTaskCreate(supervisor_task, "super",   3072, NULL, 6, NULL);

    uart_driver_install(UART_NUM_0, 512, 0, 0, NULL, 0);
    xTaskCreate(cli_task,        "cli",     8192, NULL, 2, NULL);

    log_event("SYSTEM", "boot;sd=%d;rc522=%d/%d", g_sd_ok, g_rc_present[0], g_rc_present[1]);
    printf("\n=== CoCIS Smart Control & Monitoring System ===\nType 'help' for commands.\n");
}