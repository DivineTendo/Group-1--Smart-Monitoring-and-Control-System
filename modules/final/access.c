#include "cocis_app.h"

/* ================================================================
 * BUZZER
 * ================================================================ */
void buzz(int pattern)
{
    if (buzzer_queue != NULL) {
        xQueueSend(buzzer_queue, &pattern, 0);
    }
}

void tone(int hz, int ms)
{
    if (hz > 0) {
        ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_1, hz);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 4096);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
    }
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    vTaskDelay(pdMS_TO_TICKS(ms));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
}

void buzzer_task(void *arg)
{
    while (1) {
        update_task_heartbeat(TASK_BUZZER);

        if (emergency_active()) {          
            tone(1800, 400); 
            tone(0, 300);
            continue;
        }

        int pattern;
        if (xQueueReceive(buzzer_queue, &pattern, pdMS_TO_TICKS(200)) != pdTRUE) {
            continue;
        }

        switch (pattern) {
            case BUZZER_OK:
                tone(2000, 80);
                break;
            case BUZZER_DENIED:
                tone(400, 150);
                tone(0, 100);
                tone(400, 150);
                break;
            default:
                break;
        }
    }
}

/* Buzzer LEDC: 2 kHz, 13-bit */
void buzzer_ledc_setup(void)
{
    ledc_timer_config_t buzzer_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 2000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&buzzer_timer);

    ledc_channel_config_t buzzer_channel = {
        .gpio_num = PIN_BUZZER,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_2,
        .timer_sel = LEDC_TIMER_1
    };
    ledc_channel_config(&buzzer_channel);
}

/* ================================================================
 * SERVO / DOOR
 * ================================================================ */
void set_servo_angle(int angle, int servo)
{
    if (angle < 0) {
        angle = 0;
    } else if (angle > 180) {
        angle = 180;
    }

    uint32_t pulse = 500 + ((uint32_t)angle * 1900U) / 180U;
    uint32_t duty = (pulse * 8192U) / 20000U;

    if (servo == 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    }
}

/* Servo LEDC: 50 Hz, 13-bit */
void init_access_servos(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel1 = {
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .gpio_num = SERVO_1,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .intr_type = LEDC_INTR_DISABLE,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config_t channel2 = {
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_0,
        .gpio_num = SERVO_2,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .intr_type = LEDC_INTR_DISABLE,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel1);
    ledc_channel_config(&channel2);

    set_servo_angle(0, 0);
    set_servo_angle(180, 1);
}

/* Open both doors */
void opendoors(void)
{
    /* If emergency is already active, force doors open and do not close */
    if (emergency_active()) {
        set_servo_angle(90, 0);
        set_servo_angle(90, 1);
        return;
    }

    /* Normal access: open both doors for 2 seconds */
    set_servo_angle(90, 0);
    set_servo_angle(90, 1);

    vTaskDelay(pdMS_TO_TICKS(2000));

    /* If emergency was activated while doors were open, keep them open */
    if (emergency_active()) {
        return;
    }

    /* Close both doors */
    set_servo_angle(0, 0);
    set_servo_angle(180, 1);
}

/* ================================================================
 * ACCESS LOGIC  (eria.c version)
 * ================================================================ */
user_t *find_user(const char *uid)
{
    for (int i = 0; i < MAX_USERS; i++) {
        if (registered_users[i].used &&
            strcmp(registered_users[i].uid, uid) == 0) {
            return &registered_users[i];
        }
    }
    return NULL;
}

access_result_t access_handle_scan(direction_t direction, const char *uid_input)
{
    char uid[UID_LEN];
    snprintf(uid, sizeof(uid), "%s", uid_input);
    for (char *p = uid; *p; p++) {
        *p = toupper((unsigned char)*p);
    }

    bool schedule_ok = schedule_is_open(time(NULL));
    access_result_t result = ACCESS_DENIED_UNKNOWN;
    char name[NAME_LEN] = "-";

    user_t *user = find_user(uid);

    if (user == NULL) {
        result = ACCESS_DENIED_UNKNOWN;
    } else {
        snprintf(name, sizeof(name), "%s", user->name);

        if (!schedule_ok) {
            result = ACCESS_DENIED_SCHEDULE;
        } else if (direction == DIRECTION_ENTRY && user->inside) {
            result = ACCESS_DENIED_PASSBACK;
        } else if (direction == DIRECTION_EXIT && !user->inside) {
            result = ACCESS_DENIED_PASSBACK;
        } else {
            user->inside = (direction == DIRECTION_ENTRY);
            result = ACCESS_GRANTED;
        }
    }

    static const char *result_names[] = {
        "GRANTED", "DENIED_UNKNOWN", "DENIED_SCHEDULE", "DENIED_PASSBACK"
    };

    printf("[RFID] %s UID %s -> %s (%s)\n",
           direction == DIRECTION_ENTRY ? "ENTRY" : "EXIT",
           uid, result_names[result], name);

    if (result == ACCESS_GRANTED) {
        buzz(BUZZER_OK);
        opendoors();
    } else {
        buzz(BUZZER_DENIED);
    }

    return result;
}

/* ================================================================
 * READER TASK
 * ================================================================ */
void reader_task(void *arg)
{
    char last_uid[2][UID_LEN] = { "", "" };
    int64_t last_time_ms[2] = { 0, 0 };

    while (1) {
        update_task_heartbeat(TASK_READER);

        for (int reader = 0; reader < 2; reader++) {
            if (!rfid_reader_present[reader]) {
                continue;
            }

            char uid[UID_LEN];
            if (!rfid_read_uid(reader, uid)) {
                continue;
            }

            int64_t now_ms = esp_timer_get_time() / 1000;

            /* Debounce: ignore same UID on same reader for 2 s */
            if (strcmp(uid, last_uid[reader]) == 0 &&
                now_ms - last_time_ms[reader] < 2000) {
                last_time_ms[reader] = now_ms;
                continue;
            }

            snprintf(last_uid[reader], UID_LEN, "%s", uid);
            last_time_ms[reader] = now_ms;
            access_handle_scan(reader == 0 ? DIRECTION_ENTRY : DIRECTION_EXIT, uid);
        }
        vTaskDelay(pdMS_TO_TICKS(120));
    }
}