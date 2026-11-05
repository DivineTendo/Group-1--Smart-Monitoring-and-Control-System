#include "cocis_app.h"

/* ---- schedule ---- */
bool schedule_is_open(time_t now)
{
    uint16_t start;
    uint16_t end;

    if (lock_resource(RESOURCE_CONFIG, 200))
    {
        start = schedule_start_minutes;
        end = schedule_end_minutes;
        unlock_resource(RESOURCE_CONFIG);
    }
    else
    {
        start = DEFAULT_SCHED_START;
        end = DEFAULT_SCHED_END;
    }

    struct tm tm;
    localtime_r(&now, &tm);
    int current_minutes = tm.tm_hour * 60 + tm.tm_min;

    if (start == end)
    {
        return true;
    }

    /* Normal window (e.g. 06:00–22:00) */
    if (start < end)
    {
        return current_minutes >= start && current_minutes < end;
    }

    /* Window crosses midnight (e.g. 22:00–06:00) */
    return current_minutes >= start || current_minutes < end;
}

void schedule_save(void)
{
    if (!lock_resource(RESOURCE_CONFIG, 500))
    {
        return;
    }
    nvs_set_u16(nvshandle, "sch_start", schedule_start_minutes);
    nvs_set_u16(nvshandle, "sch_end", schedule_end_minutes);
    nvs_commit(nvshandle);
    unlock_resource(RESOURCE_CONFIG);
}

/* ---- NVS / config ---- */
void config_load(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_open("cocis", NVS_READWRITE, &nvshandle));

    uint16_t value;
    if (nvs_get_u16(nvshandle, "sch_start", &value) == ESP_OK)
    {
        schedule_start_minutes = value;
    }
    if (nvs_get_u16(nvshandle, "sch_end", &value) == ESP_OK)
    {
        schedule_end_minutes = value;
    }

    size_t size = sizeof(daily_statistics);
    if (nvs_get_blob(nvshandle, "stats", daily_statistics, &size) != ESP_OK)
    {
        memset(daily_statistics, 0, sizeof(daily_statistics));
    }
}

int today_key(void)
{
    time_t now = time(NULL);
    struct tm tm;
    localtime_r(&now, &tm);
    return (tm.tm_year + 1900) * 10000 + (tm.tm_mon + 1) * 100 + tm.tm_mday;
}

/* kind: 0=entry  1=exit  2=denied.  Caller must hold RESOURCE_DATA. */
void statistics_add_locked(int kind)
{
    int today = today_key();
    int slot = -1;
    int oldest = 0;

    for (int i = 0; i < 7; i++)
    {
        if (daily_statistics[i].date == today)
        {
            slot = i;
            break;
        }
        if (daily_statistics[i].date < daily_statistics[oldest].date)
        {
            oldest = i;
        }
    }
    if (slot < 0)
    {
        slot = oldest;
        memset(&daily_statistics[slot], 0, sizeof(daily_statistics[slot]));
        daily_statistics[slot].date = today;
    }

    if (kind == 0)
    {
        daily_statistics[slot].entries++;
    }
    else if (kind == 1)
    {
        daily_statistics[slot].exits++;
    }
    else
    {
        daily_statistics[slot].denied++;
    }
    statistics_changed = true;
}

/* ---- auth ---- */
void password_store(const char *password)
{
    if (!lock_resource(RESOURCE_CONFIG, 500))
    {
        return;
    }
    nvs_set_str(nvshandle, "admin_pw", password);
    nvs_commit(nvshandle);
    unlock_resource(RESOURCE_CONFIG);
}

bool password_check(const char *password)
{
    char stored[64];
    size_t length = sizeof(stored);
    bool found = false;

    if (lock_resource(RESOURCE_CONFIG, 500))
    {
        found = nvs_get_str(nvshandle, "admin_pw", stored, &length) == ESP_OK;
        unlock_resource(RESOURCE_CONFIG);
    }
    if (!found)
    {
        return strcmp(password, DEFAULT_ADMIN_PW) == 0;
    }
    return strcmp(password, stored) == 0;
}

void auth_init(void)
{
    char temp[64];
    size_t length = sizeof(temp);
    bool have_password = false;

    if (lock_resource(RESOURCE_CONFIG, 500))
    {
        have_password = nvs_get_str(nvshandle, "admin_pw", temp, &length) == ESP_OK;
        unlock_resource(RESOURCE_CONFIG);
    }
    if (!have_password)
    {
        password_store(DEFAULT_ADMIN_PW);
        printf("\n*** First boot: admin password is '%s'. Change with 'passwd'. ***\n",
               DEFAULT_ADMIN_PW);
    }
}

/* ---- clock ---- */
void clock_init(void)
{
    setenv("TZ", "EAT-3", 1);
    tzset();

    if (time(NULL) < 1700000000)
    {
        struct tm tm = {0};
        tm.tm_year = 2026 - 1900;
        tm.tm_mon = 10;
        tm.tm_mday = 4;
        tm.tm_hour = 12;
        tm.tm_min = 0;
        tm.tm_sec = 0;
        tm.tm_isdst = -1;

        struct timeval tv = {.tv_sec = mktime(&tm)};
        settimeofday(&tv, NULL);
    }
}