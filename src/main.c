#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ROOM_SCAN_PERIOD_MS 500
#define OCCUPIED_DISTANCE_CM 150
#define ECHO_TIMEOUT_US 30000

static const char *TAG = "lecture_rooms";

typedef struct {
    const char *name;
    gpio_num_t trigger_pin;
    gpio_num_t echo_pin;
    gpio_num_t light_pin;
} room_t;

static const room_t rooms[] = {
    {
        .name = "Lecture Room 1",
        .trigger_pin = GPIO_NUM_5,
        .echo_pin = GPIO_NUM_18,
        .light_pin = GPIO_NUM_14,
    },
    {
        .name = "Lecture Room 2",
        .trigger_pin = GPIO_NUM_4,
        .echo_pin = GPIO_NUM_19,
        .light_pin = GPIO_NUM_27,
    },
};

static bool configure_room_gpio(const room_t *room)
{
    gpio_config_t trigger_config = {
        .pin_bit_mask = 1ULL << room->trigger_pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config_t echo_config = {
        .pin_bit_mask = 1ULL << room->echo_pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config_t light_config = {
        .pin_bit_mask = 1ULL << room->light_pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    if (gpio_config(&trigger_config) != ESP_OK ||
        gpio_config(&echo_config) != ESP_OK ||
        gpio_config(&light_config) != ESP_OK) {
        ESP_LOGE(TAG, "%s GPIO configuration failed", room->name);
        return false;
    }

    gpio_set_level(room->trigger_pin, 0);
    gpio_set_level(room->light_pin, 0);
    return true;
}

static bool read_distance_cm(const room_t *room, float *distance_cm)
{
    gpio_set_level(room->trigger_pin, 0);
    esp_rom_delay_us(2);
    gpio_set_level(room->trigger_pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level(room->trigger_pin, 0);

    int64_t start = esp_timer_get_time();
    while (gpio_get_level(room->echo_pin) == 0) {
        if (esp_timer_get_time() - start >= ECHO_TIMEOUT_US) {
            return false;
        }
    }

    int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(room->echo_pin) == 1) {
        if (esp_timer_get_time() - echo_start >= ECHO_TIMEOUT_US) {
            return false;
        }
    }

    *distance_cm = (float)(esp_timer_get_time() - echo_start) / 58.0f;
    return true;
}

static void lecture_room_task(void *argument)
{
    const room_t *room = argument;
    bool light_on = false;

    if (!configure_room_gpio(room)) {
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "%s task started: trigger GPIO%d, echo GPIO%d, light GPIO%d",
             room->name, room->trigger_pin, room->echo_pin, room->light_pin);

    while (true) {
        float distance_cm;

        if (read_distance_cm(room, &distance_cm)) {
            bool occupied = distance_cm <= OCCUPIED_DISTANCE_CM;

            if (occupied != light_on) {
                light_on = occupied;
                gpio_set_level(room->light_pin, light_on ? 1 : 0);
            }
            ESP_LOGI(TAG, "%s | distance: %.1f cm | occupied: %s | light: %s",
                     room->name, distance_cm, occupied ? "YES" : "NO",
                     light_on ? "ON" : "OFF");
        } else {
            if (light_on) {
                light_on = false;
                gpio_set_level(room->light_pin, 0);
            }
            ESP_LOGW(TAG, "%s | no echo received within %d ms | occupied: NO | light: OFF",
                     room->name, ECHO_TIMEOUT_US / 1000);
        }

        vTaskDelay(pdMS_TO_TICKS(ROOM_SCAN_PERIOD_MS));
    }
}

void app_main(void)
{
    for (size_t i = 0; i < sizeof(rooms) / sizeof(rooms[0]); ++i) {
        if (xTaskCreate(lecture_room_task, rooms[i].name, 3072,
                        (void *)&rooms[i], 5, NULL) != pdPASS) {
            ESP_LOGE(TAG, "Could not create task for %s", rooms[i].name);
        }
    }
}