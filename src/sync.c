#include "cocis_app.h"

/* ---- resource names ---- */
static const char *RESOURCE_NAMES[RESOURCE_COUNT] = {
    "DATA", "CONFIG", "LOG", "SD", "SPI", "I2C"};

/* ---- shared state definitions ---- */
SemaphoreHandle_t resource_mutexes[RESOURCE_COUNT];
volatile uint32_t resource_lock_timeouts[RESOURCE_COUNT];

QueueHandle_t log_queue;
QueueHandle_t room_message_queue;
QueueHandle_t buzzer_queue;
EventGroupHandle_t system_event_group;
volatile uint32_t task_last_heartbeat[TASK_COUNT];

user_t registered_users[MAX_USERS];
int people_inside_count;
bool users_changed;
room_t room_states[2];
day_stat_t daily_statistics[7];
bool statistics_changed;
char recent_alerts[8][96];
int alert_total;
volatile int ldr_raw_value;

uint16_t schedule_start_minutes = DEFAULT_SCHED_START;
uint16_t schedule_end_minutes = DEFAULT_SCHED_END;
nvs_handle_t nvshandle;

char log_ring[RING_N][LOG_LINE];
int log_ring_total;
volatile bool sd_card_ok;
volatile uint32_t log_dropped_count;
volatile bool echo_log = true;

bool rfid_reader_present[2];
volatile bool stress_test_active = false;

/* ---- locking ---- */
bool lock_resource(resource_t resource, uint32_t timeout_ms)
{
    if (resource_mutexes[resource] == NULL)
    {
        return false;
    }
    if (xSemaphoreTake(resource_mutexes[resource], pdMS_TO_TICKS(timeout_ms)) == pdTRUE)
    {
        return true;
    }
    resource_lock_timeouts[resource]++;
    printf("!! lock timeout on %s\n", RESOURCE_NAMES[resource]);
    return false;
}

void unlock_resource(resource_t resource)
{
    if (resource_mutexes[resource] != NULL)
    {
        xSemaphoreGive(resource_mutexes[resource]);
    }
}

/* ---- heartbeat ---- */
void update_task_heartbeat(task_id_t task)
{
    task_last_heartbeat[task] = xTaskGetTickCount();
}

/* ---- emergency flag ---- */
bool emergency_active(void)
{
    if (system_event_group == NULL)
    {
        return false;
    }
    return (xEventGroupGetBits(system_event_group) & EVENT_EMERGENCY) != 0;
}

/* ---- time formatting ---- */
void format_time(time_t t, char *out, size_t size)
{
    struct tm tm;
    localtime_r(&t, &tm);
    strftime(out, size, "%Y-%m-%d %H:%M:%S", &tm);
}

/* ---- logging front-end ---- */
void log_event(const char *type, const char *fmt, ...)
{
    if (log_queue == NULL)
    {
        return;
    }
    char detail[100];
    va_list args;
    va_start(args, fmt);
    vsnprintf(detail, sizeof(detail), fmt, args);
    va_end(args);

    char timestamp[24];
    format_time(time(NULL), timestamp, sizeof(timestamp));

    log_item_t item;
    snprintf(item.line, sizeof(item.line), "%s,%s,%s", timestamp, type, detail);

    if (xQueueSend(log_queue, &item, pdMS_TO_TICKS(20)) != pdTRUE)
    {
        log_dropped_count++;
    }
}

void raise_alert(const char *kind, const char *fmt, ...)
{
    char detail[80];
    va_list args;
    va_start(args, fmt);
    vsnprintf(detail, sizeof(detail), fmt, args);
    va_end(args);

    char timestamp[24];
    format_time(time(NULL), timestamp, sizeof(timestamp));

    if (lock_resource(RESOURCE_DATA, 200))
    {
        int slot = alert_total % 8;
        snprintf(recent_alerts[slot], sizeof(recent_alerts[slot]),
                 "%s %s: %s", timestamp + 11, kind, detail);
        alert_total++;
        unlock_resource(RESOURCE_DATA);
    }
    log_event("ALERT", "%s;%s", kind, detail);
    buzz(BUZZER_DENIED);
}

void log_print_recent(int max_lines)
{
    if (max_lines < 1)
    {
        max_lines = 10;
    }
    if (max_lines > RING_N)
    {
        max_lines = RING_N;
    }
    if (!lock_resource(RESOURCE_LOG, 200))
    {
        return;
    }
    int total = log_ring_total;
    if (total == 0)
    {
        unlock_resource(RESOURCE_LOG);
        printf("no log lines\n");
        return;
    }
    int count = total < max_lines ? total : max_lines;
    for (int i = 0; i < count; i++)
    {
        int index = (total - count + i) % RING_N;
        printf("%s\n", log_ring[index]);
    }
    unlock_resource(RESOURCE_LOG);
}