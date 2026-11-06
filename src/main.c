#include "cocis_app.h"

static const char *TAG = "COCIS_MAIN";

void app_main(void)
{
    /* Create resource mutexes */
    for (int i = 0; i < RESOURCE_COUNT; i++) {
        resource_mutexes[i] = xSemaphoreCreateMutex();
    }

    /* Create queues and event group */
    log_queue = xQueueCreate(48, sizeof(log_item_t));
    room_message_queue = xQueueCreate(16, sizeof(room_message_t));
    buzzer_queue = xQueueCreate(8, sizeof(int));
    system_event_group = xEventGroupCreate();

    /* System setup */
    clock_init();
    config_load();
    auth_init();

    if (wifi_start()) {
        ESP_LOGI(TAG, "WiFi started successfully.");
        start_webserver();
    } else {
        ESP_LOGE(TAG, "WiFi failed to start!");
    }

    /* Peripheral setup (each in its own module) */
    init_access_servos();
    buzzer_ledc_setup();
    rooms_gpio_init();
    rooms_adc_init();
    system_gpio_init();
    spi_bus_setup();
    rc522_setup();
    sd_card_setup();
    users_load();

    /* Seed heartbeats before tasks start */
    uint32_t now = xTaskGetTickCount();
    for (int i = 0; i < TASK_COUNT; i++) {
        task_last_heartbeat[i] = now;
    }

    /* Create tasks */
    xTaskCreate(logger_task,      "logger",     6144, NULL, 3, NULL);
    xTaskCreate(buzzer_task,      "buzzer",     2048, NULL, 3, NULL);
    xTaskCreate(reader_task,      "reader",     6144, NULL, 5, NULL);
    xTaskCreate(rooms_task,       "rooms",      4096, NULL, 4, NULL);
    xTaskCreate(coordinator_task, "coord",      4096, NULL, 4, NULL);
    xTaskCreate(io_task,          "io",         3072, NULL, 6, NULL);
    xTaskCreate(display_task,     "display",    4096, NULL, 1, NULL);
    xTaskCreate(supervisor_task,  "supervisor", 3072, NULL, 6, NULL);

    /* UART + CLI */
    uart_driver_install(UART_NUM_0, 512, 0, 0, NULL, 0);
    xTaskCreate(cli_task, "cli", 8192, NULL, 2, NULL);

    log_event("SYSTEM", "boot;sd=%d;rc522=%d/%d", sd_card_ok, rfid_reader_present[0], rfid_reader_present[1]);

    printf("\n=== CoCIS Smart Control & Monitoring System ===\n");
    printf("Type 'help' for commands.\n");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}