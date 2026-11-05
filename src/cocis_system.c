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

/* ====================================================================== */
/* 5. LCD 16x2 (I2C) + dashboard                                         */
/* ====================================================================== */

#define LCD_BACKLIGHT  0x08
#define LCD_ENABLE     0x04
#define LCD_RS         0x01

static void lcd_write_expander(uint8_t data)
{
    i2c_master_write_to_device(
        I2C_NUM_0, LCD_ADDR, &data, 1, pdMS_TO_TICKS(100)
    );
}

static void lcd_write4(uint8_t nibble)
{
       uint8_t data = (nibble & 0xF0) | (nibble & 0x03) | LCD_BACKLIGHT;

    lcd_write_expander(data);
    lcd_write_expander(data | LCD_ENABLE);
    esp_rom_delay_us(1);
    lcd_write_expander(data & ~LCD_ENABLE);
    esp_rom_delay_us(50);
}

static void lcd_send(uint8_t value, bool is_data)
{
    uint8_t mode = is_data ? LCD_RS : 0;

    lcd_write4((value & 0xF0) | mode);
    lcd_write4(((value << 4) & 0xF0) | mode);
}

static void lcd_command(uint8_t command)
{
    lcd_send(command, false);
}

static void lcd_character(char character)
{
    lcd_send((uint8_t)character, true);
}

static void lcd_clear(void)
{
    lcd_command(0x01);
    vTaskDelay(pdMS_TO_TICKS(2));
}

static void lcd_set_cursor(uint8_t column, uint8_t row)
{
    static const uint8_t row_offsets[] = { 0x00, 0x40 };

    lcd_command(0x80 | (row_offsets[row] + column));
}

static void lcd_print(const char *text)
{
    while (*text) {
        lcd_character(*text++);
    }
}

static void lcd_print_line(uint8_t row, const char *text)
{
    char line[17];

    snprintf(line, sizeof(line), "%-16.16s", text);

    lcd_set_cursor(0, row);
    lcd_print(line);
}

static void lcd_init(void)
{
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };

    i2c_param_config(I2C_NUM_0, &cfg);
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    vTaskDelay(pdMS_TO_TICKS(50));

    /* Initialize the LCD in 4-bit mode. */
    lcd_write4(0x30);
    vTaskDelay(pdMS_TO_TICKS(5));

    lcd_write4(0x30);
    vTaskDelay(pdMS_TO_TICKS(1));

    lcd_write4(0x30);
    vTaskDelay(pdMS_TO_TICKS(1));

    lcd_write4(0x20);

    lcd_command(0x28);  // 4-bit mode, 2 lines, 5x8 characters
    lcd_command(0x0C);  // Display ON, cursor OFF
    lcd_command(0x06);  // Move cursor forward
    lcd_clear();
}

void display_task(void *arg)
{
    // Initialize the LCD once before entering the display loop.
    lcd_init();

    for (;;) {
        update_task_heartbeat(TASK_DISPLAY);

        int inside = 0;
        room_t r[2];

        // Read shared system data safely.
        if (lock_resource(RESOURCE_DATA, 100)) {
            inside = people_inside_count;
            r[0] = room_states[0];
            r[1] = room_states[1];

            unlock_resource(RESOURCE_DATA);
        } else {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        char line1[17];
        char line2[17];

        // Check emergency status.
        bool emg = emergency_active();

        if (emg) {
            snprintf(line1, sizeof(line1), "!!FIRE ALERT!!");
            snprintf(line2, sizeof(line2), "EVACUATE NOW");
        } else {
            snprintf(line1, sizeof(line1),
                   "IN:%02d R1:%s L:%s",
                   inside,
                   r[0].occupied ? "O" : "E",
                   r[0].light_on ? "ON" : "OFF");

            snprintf(line2, sizeof(line2),
                   "R2:%s L:%s",
                   r[1].occupied ? "O" : "E",
                   r[1].light_on ? "ON" : "OFF");
        }
        // Write to the LCD using the I2C resource lock.
        if (lock_resource(RESOURCE_I2C, 200)) {
            lcd_print_line(0, line1);
            lcd_print_line(1, line2);

            unlock_resource(RESOURCE_I2C);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}