#include "cocis_app.h"

static const char *TAG = "cocis";
static sdmmc_card_t *sd_card;

/* ---- SPI bus ---- */
void spi_bus_setup(void)
{
    spi_bus_config_t bus = {
        .mosi_io_num = PIN_SPI_MOSI,
        .miso_io_num = PIN_SPI_MISO,
        .sclk_io_num = PIN_SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000};
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));
}

/* ---- SD card ---- */
void sd_card_setup(void)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 4,
        .allocation_unit_size = 16 * 1024};
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    host.max_freq_khz = 10000;

    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs = PIN_SD_CS;
    slot.host_id = SPI2_HOST;

    esp_err_t err = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot,
                                            &mount_config, &sd_card);
    sd_card_ok = (err == ESP_OK);

    if (sd_card_ok)
    {
        ESP_LOGI(TAG, "SD mounted at %s", MOUNT_POINT);
    }
    else
    {
        ESP_LOGW(TAG, "SD mount failed (%s) - RAM ring only",
                 esp_err_to_name(err));
    }
}

/* Caller must hold RESOURCE_SD + RESOURCE_SPI */
static void sd_rotate_locked(void)
{
    remove(LOG_PATH_1);
    rename(LOG_PATH, LOG_PATH_1);
}

static void sd_append_lines(char lines[][LOG_LINE], int count)
{
    if (!sd_card_ok || count <= 0)
    {
        return;
    }
    if (!lock_resource(RESOURCE_SD, 1000))
    {
        return;
    }
    if (!lock_resource(RESOURCE_SPI, 1000))
    {
        unlock_resource(RESOURCE_SD);
        return;
    }

    struct stat st;
    bool fresh_file = (stat(LOG_PATH, &st) != 0);
    FILE *file = fopen(LOG_PATH, "a");

    if (file != NULL)
    {
        if (fresh_file)
        {
            fputs("timestamp,event,detail\n", file);
        }
        for (int i = 0; i < count; i++)
        {
            fputs(lines[i], file);
            fputc('\n', file);
        }
        long size = ftell(file);
        fclose(file);
        if (size > LOG_ROTATE_BYTES)
        {
            sd_rotate_locked();
        }
    }
    else
    {
        ESP_LOGW(TAG, "cannot open %s (errno %d)", LOG_PATH, errno);
    }

    unlock_resource(RESOURCE_SPI);
    unlock_resource(RESOURCE_SD);
}

/* ---- users ---- */
static void users_save(void)
{
    static user_t snapshot[MAX_USERS];
    if (!lock_resource(RESOURCE_DATA, 500))
    {
        return;
    }
    memcpy(snapshot, registered_users, sizeof(snapshot));
    users_changed = false;
    unlock_resource(RESOURCE_DATA);

    if (!sd_card_ok)
    {
        return;
    }
    if (!lock_resource(RESOURCE_SD, 1000))
    {
        return;
    }
    if (!lock_resource(RESOURCE_SPI, 1000))
    {
        unlock_resource(RESOURCE_SD);
        return;
    }

    FILE *file = fopen(USERS_TMP, "w");
    if (file != NULL)
    {
        fputs("uid,name,inside\n", file);
        for (int i = 0; i < MAX_USERS; i++)
        {
            if (snapshot[i].used)
            {
                fprintf(file, "%s,%s,%d\n",
                        snapshot[i].uid, snapshot[i].name,
                        snapshot[i].inside ? 1 : 0);
            }
        }
        fclose(file);
        remove(USERS_PATH);
        rename(USERS_TMP, USERS_PATH);
    }
    unlock_resource(RESOURCE_SPI);
    unlock_resource(RESOURCE_SD);
}

static void users_seed(void)
{
    static const char *seed[][2] = {
        {"01:02:03:04", "Alice Namuli"},
        {"11:22:33:44", "Brian Okello"},
        {"55:66:77:88", "Carol Nakato"},
        {"AA:BB:CC:DD", "Dr Mukasa"}};
    for (int i = 0; i < 4; i++)
    {
        snprintf(registered_users[i].uid, UID_LEN, "%s", seed[i][0]);
        snprintf(registered_users[i].name, NAME_LEN, "%s", seed[i][1]);
        registered_users[i].used = true;
    }
}

void users_load(void)
{
    static user_t temp[MAX_USERS];
    memset(temp, 0, sizeof(temp));
    int count = 0;
    bool loaded = false;

    if (sd_card_ok && lock_resource(RESOURCE_SD, 1000))
    {
        if (lock_resource(RESOURCE_SPI, 1000))
        {
            FILE *file = fopen(USERS_PATH, "r");
            if (file != NULL)
            {
                char line[96];
                while (fgets(line, sizeof(line), file) != NULL && count < MAX_USERS)
                {
                    line[strcspn(line, "\r\n")] = 0;
                    char *comma1 = strchr(line, ',');
                    if (comma1 == NULL || strncmp(line, "uid,", 4) == 0)
                    {
                        continue;
                    }
                    *comma1++ = 0;
                    char *comma2 = strchr(comma1, ',');
                    int inside = 0;
                    if (comma2 != NULL)
                    {
                        *comma2++ = 0;
                        inside = atoi(comma2);
                    }
                    snprintf(temp[count].uid, UID_LEN, "%.11s", line);
                    snprintf(temp[count].name, NAME_LEN, "%.23s", comma1);
                    temp[count].inside = (inside != 0);
                    temp[count].used = true;
                    count++;
                }
                fclose(file);
                loaded = count > 0;
            }
            unlock_resource(RESOURCE_SPI);
        }
        unlock_resource(RESOURCE_SD);
    }

    if (lock_resource(RESOURCE_DATA, 500))
    {
        if (loaded)
        {
            memcpy(registered_users, temp, sizeof(registered_users));
        }
        else
        {
            memset(registered_users, 0, sizeof(registered_users));
            users_seed();
            users_changed = true;
        }
        unlock_resource(RESOURCE_DATA);
    }
    ESP_LOGI(TAG, "users: %s", loaded ? "loaded from users.csv" : "seeded defaults");
}

/* ---- logger task ---- */
void logger_task(void *arg)
{
    static char batch[10][LOG_LINE];
    int64_t last_save_time = 0;

    while (1)
    {
        update_task_heartbeat(TASK_LOGGER);
        log_item_t item;

        if (xQueueReceive(log_queue, &item, pdMS_TO_TICKS(500)) == pdTRUE)
        {
            int count = 0;
            memcpy(batch[count++], item.line, LOG_LINE);
            while (count < 10 && xQueueReceive(log_queue, &item, 0) == pdTRUE)
            {
                memcpy(batch[count++], item.line, LOG_LINE);
            }

            if (lock_resource(RESOURCE_LOG, 200))
            {
                for (int i = 0; i < count; i++)
                {
                    memcpy(log_ring[log_ring_total % RING_N], batch[i], LOG_LINE);
                    log_ring_total++;
                }
                unlock_resource(RESOURCE_LOG);
            }

            if (echo_log)
            {
                for (int i = 0; i < count; i++)
                {
                    printf("[LOG] %s\n", batch[i]);
                }
            }
            sd_append_lines(batch, count);
        }

        int64_t now = esp_timer_get_time();
        if (now - last_save_time > 2000000)
        {
            last_save_time = now;
            if (users_changed)
            {
                users_save();
            }
            if (statistics_changed)
            {
                static day_stat_t snapshot[7];
                if (lock_resource(RESOURCE_DATA, 200))
                {
                    memcpy(snapshot, daily_statistics, sizeof(snapshot));
                    statistics_changed = false;
                    unlock_resource(RESOURCE_DATA);
                }
                if (lock_resource(RESOURCE_CONFIG, 300))
                {
                    nvs_set_blob(nvshandle, "stats", snapshot, sizeof(snapshot));
                    nvs_commit(nvshandle);
                    unlock_resource(RESOURCE_CONFIG);
                }
            }
        }
    }
}