#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

/* ---------- config and pins ---------- */

#define PIN_LED_R1          14
#define PIN_LED_R2          27

#define PIN_SONAR1_TRIG     32
#define PIN_SONAR1_ECHO     34

#define PIN_SONAR2_TRIG     33
#define PIN_SONAR2_ECHO     35

#define LDR_ADC_CHANNEL     ADC_CHANNEL_0   /* GPIO36 / VP */

#define ROOM_OFF_DELAY_MS   30000
#define ROOM_WARN_MS        5000
#define ROOM_PRESENCE_CM    150

#define LDR_BRIGHT_RAW      2500
#define LDR_INVERT          1

/* ---------- shared state ---------- */

static adc_oneshot_unit_handle_t adc_handle;
static volatile int ldr_raw_value;

/* ---------- sonar ---------- */

/* Returns distance in cm, or -1 on timeout. */
static int sonar_read_cm(int room)
{
    int trig = (room == 0) ? PIN_SONAR1_TRIG : PIN_SONAR2_TRIG;
    int echo = (room == 0) ? PIN_SONAR1_ECHO : PIN_SONAR2_ECHO;

    gpio_set_level(trig, 0);
    esp_rom_delay_us(4);

    gpio_set_level(trig, 1);
    esp_rom_delay_us(10);

    gpio_set_level(trig, 0);

    int64_t start = esp_timer_get_time();

    while (!gpio_get_level(echo)) {
        if (esp_timer_get_time() - start > 30000) {
            return -1;
        }
    }

    int64_t edge_start = esp_timer_get_time();

    while (gpio_get_level(echo)) {
        if (esp_timer_get_time() - edge_start > 25000) {
            return -1;
        }
    }

    return (int)((esp_timer_get_time() - edge_start) / 58);
}

/* ---------- LDR ---------- */

static bool ldr_is_bright(void)
{
    bool bright = ldr_raw_value > LDR_BRIGHT_RAW;

    return LDR_INVERT ? !bright : bright;
}

/* ---------- hardware setup ---------- */

static void gpio_setup(void)
{
    gpio_config_t output_pins = {
        .pin_bit_mask = (1ULL << PIN_LED_R1) |
                        (1ULL << PIN_LED_R2) |
                        (1ULL << PIN_SONAR1_TRIG) |
                        (1ULL << PIN_SONAR2_TRIG),
        .mode = GPIO_MODE_OUTPUT
    };

    gpio_config(&output_pins);

    gpio_config_t input_pins = {
        .pin_bit_mask = (1ULL << PIN_SONAR1_ECHO) |
                        (1ULL << PIN_SONAR2_ECHO),
        .mode = GPIO_MODE_INPUT
    };

    gpio_config(&input_pins);
}

static void adc_setup(void)
{
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT_1
    };

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_config, &adc_handle));

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, LDR_ADC_CHANNEL, &channel_config));
}

/* ---------- rooms ---------- */

static void rooms_task(void *arg)
{
    const int room_led[2] = {
        PIN_LED_R1,
        PIN_LED_R2
    };

    bool present[2] = { false, false };
    bool light[2] = { false, false };

    bool previous_present[2] = { false, false };
    bool previous_light[2] = { false, false };

    /* 3-slot history per room for the 2-of-3 vote. */
    bool history[2][3] = {
        { false, false, false },
        { false, false, false }
    };
    int history_index[2] = { 0, 0 };

    int64_t last_seen_ms[2] = { 0, 0 };
    int64_t saved_ms[2] = { 0, 0 };

    int distance[2] = { -1, -1 };

    int64_t previous_ms = esp_timer_get_time() / 1000;
    int64_t last_ldr_read_ms = 0;
    // int64_t last_print_ms = 0;

    printf("\n=== CoCIS rooms: 2x HC-SR04 + 2x light + LDR ===\n");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(250));

        int64_t now_ms = esp_timer_get_time() / 1000;
        int64_t elapsed_ms = now_ms - previous_ms;

        previous_ms = now_ms;

        if (now_ms - last_ldr_read_ms >= 500) {
            last_ldr_read_ms = now_ms;

            int raw = 0;

            if (adc_oneshot_read(adc_handle, LDR_ADC_CHANNEL, &raw) == ESP_OK) {
                ldr_raw_value = raw;
            }
        }

        bool bright = ldr_is_bright();

        for (int room = 0; room < 2; room++) {
            distance[room] = sonar_read_cm(room);

            if (distance[room] > 0) {
                int slot = history_index[room]++ % 3;

                history[room][slot] = distance[room] < ROOM_PRESENCE_CM;

                int votes = history[room][0] + history[room][1] + history[room][2];

                present[room] = votes >= 2;
            }

            if (present[room]) {
                last_seen_ms[room] = now_ms;

                if (!bright) {
                    light[room] = true;
                }
            } else if (light[room] &&
                       now_ms - last_seen_ms[room] >= ROOM_OFF_DELAY_MS) {
                light[room] = false;
            }

            int remaining_ms = -1;

            if (light[room] && !present[room]) {
                remaining_ms = (int)(ROOM_OFF_DELAY_MS - (now_ms - last_seen_ms[room]));
            }

            /* Blink during the last ROOM_WARN_MS before switch-off. */
            bool output = light[room];

            if (remaining_ms >= 0 && remaining_ms <= ROOM_WARN_MS) {
                output = (now_ms / 250) % 2 == 0;
            }

            gpio_set_level(room_led[room], output);

            if (!light[room]) {
                saved_ms[room] += elapsed_ms;
            }

            if (present[room] != previous_present[room] ||
                light[room] != previous_light[room]) {

                previous_present[room] = present[room];
                previous_light[room] = light[room];

                printf("[ROOM %d] presence=%d light=%d dist=%d cm\n",
                       room + 1,
                       present[room],
                       light[room],
                       distance[room]);
            }
            
            // Optional 1 Hz status line
            // if (now_ms - last_print_ms >= 1000) {
            //     last_print_ms = now_ms;
            //     printf("LDR=%d (%s) | R1 %s %s d=%dcm saved=%us | R2 %s %s d=%dcm saved=%us\n",
            //             ldr_raw_value, bright ? "daylight" : "dark",
            //             present[0] ? "OCC" : "empty",
            //             light[0] ? "ON" : "OFF",
            //             distance[0], (unsigned)(saved_ms[0] / 1000),
            //             present[1] ? "OCC" : "empty",
            //             light[1] ? "ON" : "OFF",
            //             distance[1], (unsigned)(saved_ms[1] / 1000));
            // }
        }
    }
}

/* ---------- app_main ---------- */

void app_main(void)
{
    gpio_setup();
    adc_setup();

    xTaskCreate(rooms_task, "rooms", 4096, NULL, 4, NULL);
}