#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ROOM_SCAN_PERIOD_MS 250
#define OCCUPIED_DISTANCE_CM 150
#define SENSOR_TIMEOUT_US 30000
#define TRIGGER_PULSE_US 10

static const char *TAG = "room_tasks";

typedef struct {
    const char *name;
    gpio_num_t trigger_pin;
    gpio_num_t echo_pin;
    gpio_num_t relay_pin;
} room_config_t;

static const room_config_t room_configs[] = {
    {
        .name = "Lecture Room 1",
        .trigger_pin = GPIO_NUM_5,
        .echo_pin = GPIO_NUM_18,
        .relay_pin = GPIO_NUM_14,
    },
    {
        .name = "Lecture Room 2",
        .trigger_pin = GPIO_NUM_4,
        .echo_pin = GPIO_NUM_19,
        .relay_pin = GPIO_NUM_27,
    },
};

static bool measure_distance_cm(const room_config_t *room, float *distance_cm)
{
    gpio_set_level(room->trigger_pin, 0);
    esp_rom_delay_us(2);
    gpio_set_level(room->trigger_pin, 1);
    esp_rom_delay_us(TRIGGER_PULSE_US);
    gpio_set_level(room->trigger_pin, 0);

    int64_t wait_started = esp_timer_get_time();
    while (gpio_get_level(room->echo_pin) == 0) {
        if (esp_timer_get_time() - wait_started >= SENSOR_TIMEOUT_US) {
            return false;
        }
    }

    int64_t echo_started = esp_timer_get_time();
    while (gpio_get_level(room->echo_pin) == 1) {
        if (esp_timer_get_time() - echo_started >= SENSOR_TIMEOUT_US) {
            return false;
        }
    }

    int64_t pulse_width_us = esp_timer_get_time() - echo_started;
    *distance_cm = (float)pulse_width_us / 58.0f;
    return true;
}

static void room_task(void *arg)
{
    const room_config_t *room = (const room_config_t *)arg;
    bool light_on = false;

    gpio_set_direction(room->trigger_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(room->echo_pin, GPIO_MODE_INPUT);
    gpio_set_direction(room->relay_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(room->trigger_pin, 0);
    gpio_set_level(room->relay_pin, 0);

    ESP_LOGI(TAG, "%s task started (trigger=%d, echo=%d, relay=%d)",
             room->name, room->trigger_pin, room->echo_pin, room->relay_pin);

    for (;;) {
        float distance_cm = 0.0f;
        bool occupied = false;

        if (measure_distance_cm(room, &distance_cm)) {
            occupied = distance_cm <= OCCUPIED_DISTANCE_CM;
            if (occupied != light_on) {
                light_on = occupied;
                gpio_set_level(room->relay_pin, light_on);
                ESP_LOGI(TAG, "%s: %.1f cm, light %s",
                         room->name, distance_cm, light_on ? "ON" : "OFF");
            }
        } else {
            ESP_LOGW(TAG, "%s: ultrasonic sensor timeout; retaining light state",
                     room->name);
        }

        vTaskDelay(pdMS_TO_TICKS(ROOM_SCAN_PERIOD_MS));
    }
}

void app_main(void)
{
    for (size_t index = 0; index < sizeof(room_configs) / sizeof(room_configs[0]); ++index) {
        BaseType_t result = xTaskCreate(
            room_task,
            room_configs[index].name,
            3072,
            (void *)&room_configs[index],
            5,
            NULL);

        if (result != pdPASS) {
            ESP_LOGE(TAG, "Failed to create task for %s", room_configs[index].name);
        }
    }
}
