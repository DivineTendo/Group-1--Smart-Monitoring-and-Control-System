#include "cocis_app.h"

static const char *TASK_NAMES[TASK_COUNT] = {
    "reader", "rooms", "logger", "cli",
    "coord", "buzzer", "display", "io"
};

/* ---- emergency button GPIO ---- */
void system_gpio_init(void)
{
    gpio_config_t button_pin = {
        .pin_bit_mask = 1ULL << PIN_EMERG_BTN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&button_pin);
}

/* ---- emergency ---- */
void emergency_set(bool on, const char *who)
{
    if (on == emergency_active()) {
        return;
    }
    if (on) {
        xEventGroupSetBits(system_event_group, EVENT_EMERGENCY);
    } else {
        xEventGroupClearBits(system_event_group, EVENT_EMERGENCY);
    }

    log_event("EMERGENCY", "%s;by=%s", on ? "ACTIVATED" : "CLEARED", who);

    if (on) {
        set_servo_angle(90, 0);
        set_servo_angle(90, 1);
    } else {
        set_servo_angle(0, 0);
        set_servo_angle(0, 1);
    }


    if (!on) {
        return;
    }

    if (lock_resource(RESOURCE_DATA, 300)) {
        log_event("EVAC", "roster;count=%d", people_inside_count);
        for (int i = 0; i < MAX_USERS; i++) {
            if (registered_users[i].used && registered_users[i].inside) {
                log_event("EVAC", "inside;%s;%s", registered_users[i].uid, registered_users[i].name);
            }
        }
        unlock_resource(RESOURCE_DATA);
    }
}

/* ---- coordinator / ghost occupancy ---- */
void coordinator_task(void *arg)
{
    int64_t ghost_since[2] = { 0, 0 };
    bool ghost_alerted[2] = { false, false };

    while (1) {
        update_task_heartbeat(TASK_COORD);
        room_message_t message;

        while (xQueueReceive(room_message_queue, &message, pdMS_TO_TICKS(250)) == pdTRUE) {
            if (message.type == MESSAGE_ROOM_STATE && message.room < 2) {
                if (lock_resource(RESOURCE_DATA, 100)) {
                    room_states[message.room].occupied = message.occupied;
                    room_states[message.room].light_on = message.light_on;
                    unlock_resource(RESOURCE_DATA);
                }
                log_event("ROOM", "R%d;presence=%d;light=%d",
                          message.room + 1, message.occupied, message.light_on);
            }
        }

        int inside = 0;
        bool presence[2] = { false, false };
        if (lock_resource(RESOURCE_DATA, 100)) {
            inside = people_inside_count;
            presence[0] = room_states[0].occupied;
            presence[1] = room_states[1].occupied;
            unlock_resource(RESOURCE_DATA);
        }

        int64_t now_ms = esp_timer_get_time() / 1000;
        for (int room = 0; room < 2; room++) {
            if (presence[room] && inside == 0 && !emergency_active()) {
                if (ghost_since[room] == 0) {
                    ghost_since[room] = now_ms;
                }
                if (!ghost_alerted[room] && now_ms - ghost_since[room] >= GHOST_CONFIRM_MS) {
                    ghost_alerted[room] = true;
                    raise_alert("GHOST_OCCUPANCY", "Room %d active but building count is 0", room + 1);
                }
            } else {
                ghost_since[room] = 0;
                if (!presence[room]) {
                    ghost_alerted[room] = false;
                }
            }
        }
    }
}

/* ---- IO: emergency button + LDR ---- */
void io_task(void *arg)
{
    int button_low_count = 0;
    int64_t last_ldr_read_ms = 0;

    while (1) {
        update_task_heartbeat(TASK_IO);

        /* Debounce: 3 consecutive low reads */
        if (gpio_get_level(PIN_EMERG_BTN) == 0) {
            button_low_count++;
        } else {
            button_low_count = 0;
        }
        if (button_low_count == 3) {
            emergency_set(true, "BUTTON");
        }

        /* Read LDR every 500 ms */
        int64_t now_ms = esp_timer_get_time() / 1000;
        if (now_ms - last_ldr_read_ms > 500) {
            last_ldr_read_ms = now_ms;
            int raw = rooms_read_ldr();
            if (raw >= 0) {
                ldr_raw_value = raw;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ---- supervisor ---- */
void supervisor_task(void *arg)
{
    bool alerted[TASK_COUNT] = { false };

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        uint32_t now = xTaskGetTickCount();

        for (int i = 0; i < TASK_COUNT; i++) {
            uint32_t age_ms = (now - task_last_heartbeat[i]) * portTICK_PERIOD_MS;
            if (age_ms > TASK_TIMEOUT_MS && !alerted[i]) {
                alerted[i] = true;
                raise_alert("TASK_SILENT", "task '%s' silent for %u s",
                            TASK_NAMES[i], (unsigned)(age_ms / 1000));
            } else if (age_ms <= TASK_TIMEOUT_MS && alerted[i]) {
                alerted[i] = false;
                log_event("SUPERVISOR", "task '%s' recovered", TASK_NAMES[i]);
            }
        }
    }
}

/* ==================================================================
 * OLED SSD1306 128x64
 * ================================================================== */
static uint8_t oled_framebuffer[128 * 8];

/* Simple 5x7 font, ASCII 0x20 to 0x5A */
static const uint8_t OLED_FONT5X7[59][5] = {
    {0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x5F,0x00,0x00},
    {0x00,0x07,0x00,0x07,0x00},
    {0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12},
    {0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50},
    {0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00},
    {0x00,0x41,0x22,0x1C,0x00},
    {0x14,0x08,0x3E,0x08,0x14},
    {0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},
    {0x08,0x08,0x08,0x08,0x08},
    {0x00,0x60,0x60,0x00,0x00},
    {0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E},
    {0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46},
    {0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10},
    {0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30},
    {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},
    {0x06,0x49,0x49,0x29,0x1E},
    {0x00,0x36,0x36,0x00,0x00},
    {0x00,0x56,0x36,0x00,0x00},
    {0x08,0x14,0x22,0x41,0x00},
    {0x14,0x14,0x14,0x14,0x14},
    {0x00,0x41,0x22,0x14,0x08},
    {0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E},
    {0x7E,0x11,0x11,0x11,0x7E},
    {0x7F,0x49,0x49,0x49,0x36},
    {0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C},
    {0x7F,0x49,0x49,0x49,0x41},
    {0x7F,0x09,0x09,0x09,0x01},
    {0x3E,0x41,0x49,0x49,0x7A},
    {0x7F,0x08,0x08,0x08,0x7F},
    {0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01},
    {0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x0C,0x02,0x7F},
    {0x7F,0x04,0x08,0x10,0x7F},
    {0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06},
    {0x3E,0x41,0x51,0x21,0x5E},
    {0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7F,0x01,0x01},
    {0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F},
    {0x3F,0x40,0x38,0x40,0x3F},
    {0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07},
    {0x61,0x51,0x49,0x45,0x43}
};

static void oled_command(uint8_t command)
{
    uint8_t bytes[2] = { 0x00, command };

    i2c_master_write_to_device(I2C_NUM_0, OLED_ADDR, bytes, sizeof(bytes), pdMS_TO_TICKS(100));
}

static void oled_init(void)
{
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };

    i2c_param_config(I2C_NUM_0, &i2c_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    static const uint8_t init_sequence[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12,
        0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF
    };

    for (size_t i = 0; i < sizeof(init_sequence); i++) {
        oled_command(init_sequence[i]);
    }
}

static void oled_row(int row, const char *text)
{
    uint8_t *pixels = &oled_framebuffer[row * 128];
    memset(pixels, 0, 128);

    for (int i = 0; text[i] != '\0' && i < 21; i++) {
        int character = toupper((unsigned char)text[i]);

        if (character < 0x20 || character > 0x5A) {
            character = '?';
        }

        memcpy(pixels + i * 6, OLED_FONT5X7[character - 0x20], 5);
    }
}

static void oled_flush(void)
{
    /* Set column address 0 to 127 */
    oled_command(0x21);
    oled_command(0);
    oled_command(127);

    /* Set page address 0 to 7 */
    oled_command(0x22);
    oled_command(0);
    oled_command(7);

    uint8_t buffer[129];
    buffer[0] = 0x40;

    for (int page = 0; page < 8; page++) {
        memcpy(&buffer[1], &oled_framebuffer[page * 128], 128);

        i2c_master_write_to_device(I2C_NUM_0, OLED_ADDR, buffer, sizeof(buffer), pdMS_TO_TICKS(100));
    }
}

void display_task(void *arg)
{
    if (lock_resource(RESOURCE_I2C, 500)) {
        oled_init();
        unlock_resource(RESOURCE_I2C);
    }

    while (1) {
        update_task_heartbeat(TASK_DISPLAY);

        char line[24];
        char timestamp[24];

        format_time(time(NULL), timestamp, sizeof(timestamp));

        int inside = 0;
        int alerts = 0;
        room_t rooms[2];

        if (!lock_resource(RESOURCE_DATA, 100)) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        inside = people_inside_count;
        alerts = alert_total;
        rooms[0] = room_states[0];
        rooms[1] = room_states[1];

        unlock_resource(RESOURCE_DATA);

        uint16_t start = DEFAULT_SCHED_START;
        uint16_t end = DEFAULT_SCHED_END;

        if (lock_resource(RESOURCE_CONFIG, 100)) {
            start = schedule_start_minutes;
            end = schedule_end_minutes;
            unlock_resource(RESOURCE_CONFIG);
        }

        bool emergency = emergency_active();
        bool open = schedule_is_open(time(NULL));

        if (lock_resource(RESOURCE_I2C, 200)) {
            snprintf(line, sizeof(line), "COCIS %s", timestamp + 11);
            oled_row(0, line);

            snprintf(line, sizeof(line), "MODE: %s", emergency ? "FIRE EVACUATE" : "NORMAL");
            oled_row(1, line);

            snprintf(line, sizeof(line), "INSIDE: %d", inside);
            oled_row(2, line);

            snprintf(line, sizeof(line), "%02d:%02d-%02d:%02d %s", start / 60, start % 60, end / 60, end % 60, open ? "OPEN" : "SHUT");
            oled_row(3, line);

            for (int room = 0; room < 2; room++) {
                snprintf(line, sizeof(line), "R%d OCC:%s LIGHT:%s", room + 1, rooms[room].occupied ? "Y" : "N", rooms[room].light_on ? "ON" : "OFF");
                oled_row(4 + room, line);
            }

            snprintf(line, sizeof(line), "ALERTS:%d SD:%s", alerts, sd_card_ok ? "OK" : "NO");
            oled_row(6, line);

            uint32_t centi_wh = (rooms[0].saved_seconds + rooms[1].saved_seconds) * ROOM_LIGHT_WATTS / 36;

            snprintf(line, sizeof(line), "SAVED:%u.%02uWH", (unsigned)(centi_wh / 100), (unsigned)(centi_wh % 100));
            oled_row(7, line);

            oled_flush();

            unlock_resource(RESOURCE_I2C);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}