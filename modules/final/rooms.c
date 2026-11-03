#include "cocis_app.h"

static adc_oneshot_unit_handle_t adc_handle;

/* ---- GPIO for room pins ---- */
void rooms_gpio_init(void)
{
    gpio_config_t output_pins = {
        .pin_bit_mask = (1ULL << PIN_LED_R1) | (1ULL << PIN_LED_R2) | (1ULL << PIN_SONAR1_TRIG) | (1ULL << PIN_SONAR2_TRIG),
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&output_pins);

    gpio_config_t input_pins = {
        .pin_bit_mask = (1ULL << PIN_SONAR1_ECHO) | (1ULL << PIN_SONAR2_ECHO),
        .mode = GPIO_MODE_INPUT
    };
    gpio_config(&input_pins);
}

/* ---- ADC for LDR ---- */
void rooms_adc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT_1
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_config, &adc_handle));

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, LDR_ADC_CHANNEL,
                                               &channel_config));
}

int rooms_read_ldr(void)
{
    int raw = 0;
    if (adc_oneshot_read(adc_handle, LDR_ADC_CHANNEL, &raw) == ESP_OK) {
        return raw;
    }
    return -1;
}

/* ---- sonar ---- */
static int sonar_read_cm(int room)
{
    int trigger_pin = (room == 0) ? PIN_SONAR1_TRIG : PIN_SONAR2_TRIG;
    int echo_pin    = (room == 0) ? PIN_SONAR1_ECHO : PIN_SONAR2_ECHO;

    gpio_set_level(trigger_pin, 0);
    esp_rom_delay_us(4);
    gpio_set_level(trigger_pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level(trigger_pin, 0);

    int64_t start_time = esp_timer_get_time();
    while (!gpio_get_level(echo_pin)) {
        if (esp_timer_get_time() - start_time > 30000) {
            return -1;
        }
    }

    int64_t edge_start = esp_timer_get_time();
    while (gpio_get_level(echo_pin)) {
        if (esp_timer_get_time() - edge_start > 25000) {
            return -1;
        }
    }

    return (int)((esp_timer_get_time() - edge_start) / 58);
}

/* ---- daylight ---- */
bool ldr_is_bright(void)
{
    bool bright = ldr_raw_value > LDR_BRIGHT_RAW;
    return LDR_INVERT ? !bright : bright;
}

/* ---- rooms task ---- */
void rooms_task(void *arg)
{
    const int room_led[2] = { PIN_LED_R1, PIN_LED_R2 };
    bool present[2] = { false, false };
    bool light[2] = { false, false };
    bool previous_present[2] = { false, false };
    bool previous_light[2] = { false, false };
    int64_t last_seen_ms[2] = { 0, 0 };
    int64_t saved_ms[2] = { 0, 0 };
    int64_t previous_ms = esp_timer_get_time() / 1000;
    bool open_hours = true;
    int64_t last_schedule_check = 0;

    while (1) {
        update_task_heartbeat(TASK_ROOMS);
        vTaskDelay(pdMS_TO_TICKS(250));

        int64_t now_ms = esp_timer_get_time() / 1000;
        int64_t elapsed_ms = now_ms - previous_ms;
        previous_ms = now_ms;

        bool emergency = emergency_active();
        bool bright = ldr_is_bright();

        if (now_ms - last_schedule_check >= 5000) {
            open_hours = schedule_is_open(time(NULL));
            last_schedule_check = now_ms;
        }

        for (int room = 0; room < 2; room++) {
            int distance = sonar_read_cm(room);

            if (distance > 0 && distance < ROOM_PRESENCE_CM) {
                present[room] = true;
                last_seen_ms[room] = now_ms;
            } else {
                present[room] = false;
            }

            if (emergency) {
                light[room] = true;
                last_seen_ms[room] = now_ms;
            } else if (present[room]) {
                light[room] = !bright;
            } else {
                if (light[room] && now_ms - last_seen_ms[room] >= ROOM_OFF_DELAY_MS) {
                    light[room] = false;
                    if (lock_resource(RESOURCE_DATA, 100)) {
                        room_states[room].auto_off_count++;
                        unlock_resource(RESOURCE_DATA);
                    }
                }
            }

            gpio_set_level(room_led[room], light[room]);

            if (!light[room] && open_hours) {
                saved_ms[room] += elapsed_ms;
            }

            int remaining_ms = -1;
            if (light[room] && !present[room] && !emergency) {
                remaining_ms = ROOM_OFF_DELAY_MS - (now_ms - last_seen_ms[room]);
            }

            if (lock_resource(RESOURCE_DATA, 50)) {
                room_states[room].occupied = present[room];
                room_states[room].light_on = light[room];
                room_states[room].distance_cm = distance;
                room_states[room].off_in_seconds =
                    remaining_ms >= 0 ? (remaining_ms + 999) / 1000 : -1;
                room_states[room].saved_seconds += (uint32_t)(saved_ms[room] / 1000);
                unlock_resource(RESOURCE_DATA);
                saved_ms[room] %= 1000;
            }

            if (present[room] != previous_present[room] ||
                light[room] != previous_light[room]) {
                previous_present[room] = present[room];
                previous_light[room] = light[room];

                room_message_t message = {
                    MESSAGE_ROOM_STATE,
                    (uint8_t)room,
                    (uint8_t)present[room],
                    (uint8_t)light[room]
                };
                xQueueSend(room_message_queue, &message, 0);
            }
        }
    }
}