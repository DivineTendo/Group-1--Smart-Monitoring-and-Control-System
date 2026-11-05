#include "cocis_app.h"

typedef enum {
    CLI_NORMAL,
    CLI_LOGIN_PASSWORD,
    CLI_PASSWORD_NEW,
    CLI_PASSWORD_CONFIRM
} cli_mode_t;

static cli_mode_t cli_mode = CLI_NORMAL;
static bool admin_logged_in = false;
static int64_t admin_session_until = 0;
static int bad_login_count = 0;
static int64_t login_locked_until = 0;
static char new_password[40];

static bool cli_admin(void)
{
    if (!admin_logged_in) {
        return false;
    }
    int64_t now = esp_timer_get_time();
    if (now > admin_session_until) {
        admin_logged_in = false;
        printf("(session expired)\n");
        log_event("AUTH", "SESSION_TIMEOUT");
        return false;
    }
    admin_session_until = now + (int64_t)SESSION_MS * 1000;
    return true;
}

#define NEED_ADMIN()                                                \
    do {                                                            \
        if (!cli_admin()) {                                         \
            printf("ERROR: admin login required (use 'login')\n");  \
            return;                                                 \
        }                                                           \
    } while (0)

static void print_help(void)
{
    printf("\nCommands:\n"
           "  help                       this list\n"
           "  status                     system summary\n"
           "  inside                     who is in the building\n"
           "  rooms                      room occupancy and lights\n"
           "  counters                   entries/exits/denied per day\n"
           "  alerts                     recent alerts\n"
           "  tasks                      task heartbeat ages\n"
           "  log [n]                    last n log lines\n"
           "  users                      list registered cards\n"
           "  schedule                   show access schedule\n"
           "  time                       show clock\n"
           "  scan <in|out> <uid>        simulate RFID scan\n"
           "  emergency [on]             trigger fire mode\n"
           "  echo <on|off>              live log echo\n"
           "  login / logout / whoami\n\n"
           "ADMIN ONLY:\n"
           "  schedule set HH:MM HH:MM\n"
           "  time set YYYY-MM-DD HH:MM[:SS]\n"
           "  emergency clear\n"
           "  useradd <uid> <name>\n"
           "  userdel <uid>\n"
           "  passwd\n\n");
}

static void cmd_status(void)
{
    char timestamp[24];
    format_time(time(NULL), timestamp, sizeof(timestamp));

    uint16_t start, end;
    if (lock_resource(RESOURCE_CONFIG, 200)) {
        start = schedule_start_minutes;
        end = schedule_end_minutes;
        unlock_resource(RESOURCE_CONFIG);
    } else {
        start = DEFAULT_SCHED_START;
        end = DEFAULT_SCHED_END;
    }

    int inside = 0;
    int alerts = 0;
    room_t rooms[2];
    if (lock_resource(RESOURCE_DATA, 200)) {
        inside = people_inside_count;
        alerts = alert_total;
        rooms[0] = room_states[0];
        rooms[1] = room_states[1];
        unlock_resource(RESOURCE_DATA);
    }

    printf("Time     : %s\n", timestamp);
    printf("Schedule : %02d:%02d-%02d:%02d (%s)\n",
           start / 60, start % 60, end / 60, end % 60,
           schedule_is_open(time(NULL)) ? "OPEN" : "CLOSED");
    printf("Mode     : %s\n", emergency_active() ? "EMERGENCY" : "normal");
    printf("Inside   : %d\n", inside);
    for (int i = 0; i < 2; i++) {
        printf("Room %d   : %s, light %s\n", i + 1,
               rooms[i].occupied ? "occupied" : "empty",
               rooms[i].light_on ? "ON" : "OFF");
    }
    printf("Readers  : entry %s, exit %s\n",
           rfid_reader_present[0] ? "RC522" : "sim",
           rfid_reader_present[1] ? "RC522" : "sim");
    printf("SD       : %s\n", sd_card_ok ? "OK" : "N/A");
    printf("Alerts   : %d\n", alerts);
    printf("Admin    : %s | LDR %d (%s) | heap %u\n",
           admin_logged_in ? "yes" : "no",
           ldr_raw_value, ldr_is_bright() ? "bright" : "dark",
           (unsigned)esp_get_free_heap_size());
}

static void cmd_inside(void)
{
    static user_t snapshot[MAX_USERS];
    if (!lock_resource(RESOURCE_DATA, 300)) {
        printf("busy\n");
        return;
    }
    memcpy(snapshot, registered_users, sizeof(snapshot));
    unlock_resource(RESOURCE_DATA);

    printf("UID          NAME\n");
    int count = 0;
    for (int i = 0; i < MAX_USERS; i++) {
        if (snapshot[i].used && snapshot[i].inside) {
            printf("%-12s %s\n", snapshot[i].uid, snapshot[i].name);
            count++;
        }
    }
    printf("Total: %d\n", count);
}

static void cmd_rooms(void)
{
    room_t rooms[2];
    if (!lock_resource(RESOURCE_DATA, 300)) {
        return;
    }
    rooms[0] = room_states[0];
    rooms[1] = room_states[1];
    unlock_resource(RESOURCE_DATA);

    for (int i = 0; i < 2; i++) {
        uint32_t wh = rooms[i].saved_seconds * ROOM_LIGHT_WATTS / 3600;
        printf("Room %d: %-8s light %-3s dist %d cm  saved %u Wh\n",
               i + 1,
               rooms[i].occupied ? "OCCUPIED" : "empty",
               rooms[i].light_on ? "ON" : "OFF",
               rooms[i].distance_cm, (unsigned)wh);
    }
}

static void cmd_counters(void)
{
    day_stat_t stats[7];
    if (!lock_resource(RESOURCE_DATA, 300)) {
        return;
    }
    memcpy(stats, daily_statistics, sizeof(stats));
    unlock_resource(RESOURCE_DATA);

    printf("DATE       ENTRIES EXITS DENIED\n");
    for (int i = 0; i < 7; i++) {
        if (stats[i].date != 0) {
            printf("%08d   %5u %5u %6u\n",
                   stats[i].date, stats[i].entries,
                   stats[i].exits, stats[i].denied);
        }
    }
}

static void cmd_alerts(void)
{
    if (!lock_resource(RESOURCE_DATA, 300)) {
        return;
    }
    int total = alert_total;
    int count = total < 8 ? total : 8;
    for (int i = 0; i < count; i++) {
        printf("%s\n", recent_alerts[(total - count + i) % 8]);
    }
    unlock_resource(RESOURCE_DATA);
    if (total == 0) {
        printf("no alerts\n");
    }
}

static void cmd_tasks(void)
{
    static const char *names[TASK_COUNT] = {
        "reader", "rooms", "logger", "cli",
        "coord", "buzzer", "display", "io"
    };
    uint32_t now = xTaskGetTickCount();
    for (int i = 0; i < TASK_COUNT; i++) {
        uint32_t age = (now - task_last_heartbeat[i]) * portTICK_PERIOD_MS;
        printf("%-8s %u ms ago\n", names[i], (unsigned)age);
    }
}

static void cmd_users(void)
{
    static user_t snapshot[MAX_USERS];
    if (!lock_resource(RESOURCE_DATA, 300)) {
        return;
    }
    memcpy(snapshot, registered_users, sizeof(snapshot));
    unlock_resource(RESOURCE_DATA);

    for (int i = 0; i < MAX_USERS; i++) {
        if (snapshot[i].used) {
            printf("%-12s %-22s %s\n", snapshot[i].uid, snapshot[i].name,
                   snapshot[i].inside ? "inside" : "outside");
        }
    }
}

static bool parse_uid(const char *input, char *output)
{
    unsigned a, b, c, d;
    if (sscanf(input, "%2x:%2x:%2x:%2x", &a, &b, &c, &d) != 4) {
        return false;
    }
    snprintf(output, UID_LEN, "%02X:%02X:%02X:%02X", a, b, c, d);
    return true;
}

static void cli_exec(char *line)
{
    char *argv[8];
    int argc = 0;
    char *save_ptr;
    char *token = strtok_r(line, " \t", &save_ptr);
    while (token != NULL && argc < 8) {
        argv[argc++] = token;
        token = strtok_r(NULL, " \t", &save_ptr);
    }
    if (argc == 0) {
        return;
    }

    const char *cmd = argv[0];

    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        print_help();
    }
    else if (strcmp(cmd, "status") == 0) {
        cmd_status();
    }
    else if (strcmp(cmd, "inside") == 0) {
        cmd_inside();
    }
    else if (strcmp(cmd, "rooms") == 0) {
        cmd_rooms();
    }
    else if (strcmp(cmd, "counters") == 0) {
        cmd_counters();
    }
    else if (strcmp(cmd, "alerts") == 0) {
        cmd_alerts();
    }
    else if (strcmp(cmd, "tasks") == 0) {
        cmd_tasks();
    }
    else if (strcmp(cmd, "log") == 0) {
        int lines = (argc >= 2) ? atoi(argv[1]) : 10;
        log_print_recent(lines);
    }
    else if (strcmp(cmd, "users") == 0) {
        cmd_users();
    }
    else if (strcmp(cmd, "useradd") == 0) {
        NEED_ADMIN();
        char uid[UID_LEN];
        if (argc < 3 || !parse_uid(argv[1], uid)) {
            printf("usage: useradd <AA:BB:CC:DD> <name>\n");
            return;
        }
        char name[NAME_LEN] = "";
        for (int i = 2; i < argc; i++) {
            if (i > 2) {
                strncat(name, " ", sizeof(name) - strlen(name) - 1);
            }
            strncat(name, argv[i], sizeof(name) - strlen(name) - 1);
        }
        bool ok = false;
        if (lock_resource(RESOURCE_DATA, 300)) {
            if (find_user(uid) == NULL) {
                for (int i = 0; i < MAX_USERS; i++) {
                    if (!registered_users[i].used) {
                        memset(&registered_users[i], 0, sizeof(registered_users[i]));
                        snprintf(registered_users[i].uid, UID_LEN, "%s", uid);
                        snprintf(registered_users[i].name, NAME_LEN, "%s", name);
                        registered_users[i].used = true;
                        users_changed = true;
                        ok = true;
                        break;
                    }
                }
            }
            unlock_resource(RESOURCE_DATA);
        }
        printf(ok ? "added %s\n" : "failed\n", uid);
    }
    else if (strcmp(cmd, "userdel") == 0) {
        NEED_ADMIN();
        char uid[UID_LEN];
        if (argc < 2 || !parse_uid(argv[1], uid)) {
            printf("usage: userdel <uid>\n");
            return;
        }
        bool ok = false;
        if (lock_resource(RESOURCE_DATA, 300)) {
            user_t *user = find_user(uid);
            if (user != NULL) {
                memset(user, 0, sizeof(*user));
                users_changed = true;
                ok = true;
            }
            unlock_resource(RESOURCE_DATA);
        }
        printf(ok ? "deleted %s\n" : "not found\n", uid);
    }
    else if (strcmp(cmd, "schedule") == 0) {
        if (argc >= 2 && strcmp(argv[1], "set") == 0) {
            NEED_ADMIN();
            int h1, m1, h2, m2;
            if (argc < 4 ||
                sscanf(argv[2], "%d:%d", &h1, &m1) != 2 ||
                sscanf(argv[3], "%d:%d", &h2, &m2) != 2) {
                printf("usage: schedule set HH:MM HH:MM\n");
                return;
            }
            if (lock_resource(RESOURCE_CONFIG, 300)) {
                schedule_start_minutes = h1 * 60 + m1;
                schedule_end_minutes = h2 * 60 + m2;
                unlock_resource(RESOURCE_CONFIG);
            }
            schedule_save();
            printf("saved %02d:%02d-%02d:%02d\n", h1, m1, h2, m2);
        } else {
            uint16_t s, e;
            if (lock_resource(RESOURCE_CONFIG, 200)) {
                s = schedule_start_minutes;
                e = schedule_end_minutes;
                unlock_resource(RESOURCE_CONFIG);
            } else {
                s = DEFAULT_SCHED_START;
                e = DEFAULT_SCHED_END;
            }
            printf("Window %02d:%02d-%02d:%02d (%s)\n",
                   s / 60, s % 60, e / 60, e % 60,
                   schedule_is_open(time(NULL)) ? "OPEN" : "CLOSED");
        }
    }
    else if (strcmp(cmd, "time") == 0) {
        if (argc >= 2 && strcmp(argv[1], "set") == 0) {
            NEED_ADMIN();
            int Y, M, D, h = 0, m = 0, s = 0;
            if (argc < 4 ||
                sscanf(argv[2], "%d-%d-%d", &Y, &M, &D) != 3 ||
                sscanf(argv[3], "%d:%d:%d", &h, &m, &s) < 2) {
                printf("usage: time set YYYY-MM-DD HH:MM[:SS]\n");
                return;
            }
            struct tm tm = {
                .tm_year = Y - 1900, .tm_mon = M - 1, .tm_mday = D,
                .tm_hour = h, .tm_min = m, .tm_sec = s, .tm_isdst = -1
            };
            struct timeval tv = { .tv_sec = mktime(&tm) };
            settimeofday(&tv, NULL);
            printf("clock set\n");
        } else {
            char ts[24];
            format_time(time(NULL), ts, sizeof(ts));
            printf("%s\n", ts);
        }
    }
    else if (strcmp(cmd, "scan") == 0) {
        char uid[UID_LEN];
        if (argc < 3 || !parse_uid(argv[2], uid) ||
            (strcmp(argv[1], "in") != 0 && strcmp(argv[1], "out") != 0)) {
            printf("usage: scan <in|out> <AA:BB:CC:DD>\n");
            return;
        }
        static const char *messages[] = {
            "GRANTED", "DENIED: unknown", "DENIED: schedule", "DENIED: passback"
        };
        access_result_t result = access_handle_scan(
            strcmp(argv[1], "in") == 0 ? DIRECTION_ENTRY : DIRECTION_EXIT, uid);
        printf("%s\n", messages[result]);
    }
    else if (strcmp(cmd, "emergency") == 0) {
        if (argc >= 2 && strcmp(argv[1], "clear") == 0) {
            NEED_ADMIN();
            emergency_set(false, "admin");
            printf("cleared\n");
        } else {
            emergency_set(true, "CLI");
            printf("EMERGENCY ACTIVE\n");
        }
    }
    else if (strcmp(cmd, "echo") == 0) {
        echo_log = (argc >= 2 && strcmp(argv[1], "on") == 0);
        printf("echo %s\n", echo_log ? "on" : "off");
    }
    else if (strcmp(cmd, "login") == 0) {
        if (esp_timer_get_time() < login_locked_until) {
            printf("locked\n");
            return;
        }
        printf("Password: ");
        fflush(stdout);
        cli_mode = CLI_LOGIN_PASSWORD;
    }
    else if (strcmp(cmd, "logout") == 0) {
        admin_logged_in = false;
        printf("logged out\n");
    }
    else if (strcmp(cmd, "whoami") == 0) {
        printf("%s\n", cli_admin() ? "admin" : "guest");
    }
    else if (strcmp(cmd, "passwd") == 0) {
        NEED_ADMIN();
        printf("New password: ");
        fflush(stdout);
        cli_mode = CLI_PASSWORD_NEW;
    }
    else {
        printf("unknown '%s'\n", cmd);
    }
}

static void cli_line(char *line)
{
    switch (cli_mode) {
        case CLI_LOGIN_PASSWORD:
            cli_mode = CLI_NORMAL;
            if (password_check(line)) {
                admin_logged_in = true;
                admin_session_until = esp_timer_get_time() + (int64_t)SESSION_MS * 1000;
                bad_login_count = 0;
                printf("login OK\n");
            } else {
                bad_login_count++;
                if (bad_login_count >= MAX_LOGIN_FAILS) {
                    bad_login_count = 0;
                    login_locked_until = esp_timer_get_time() + (int64_t)LOGIN_LOCK_MS * 1000;
                    printf("locked %d s\n", LOGIN_LOCK_MS / 1000);
                } else {
                    printf("wrong (%d/%d)\n", bad_login_count, MAX_LOGIN_FAILS);
                }
            }
            return;
        case CLI_PASSWORD_NEW:
            if (strlen(line) < 6) {
                printf("too short\n");
                cli_mode = CLI_NORMAL;
                return;
            }
            snprintf(new_password, sizeof(new_password), "%s", line);
            printf("Confirm: ");
            fflush(stdout);
            cli_mode = CLI_PASSWORD_CONFIRM;
            return;
        case CLI_PASSWORD_CONFIRM:
            cli_mode = CLI_NORMAL;
            if (strcmp(new_password, line) == 0) {
                password_store(new_password);
                printf("changed\n");
            } else {
                printf("mismatch\n");
            }
            memset(new_password, 0, sizeof(new_password));
            return;
        default:
            break;
    }
    cli_exec(line);
}

void cli_task(void *arg)
{
    char buffer[96];
    int length = 0;
    bool skip_lf = false;

    printf("\ncocis > ");
    fflush(stdout);

    while (1) {
        update_task_heartbeat(TASK_CLI);

        if (admin_logged_in && esp_timer_get_time() > admin_session_until) {
            admin_logged_in = false;
        }

        uint8_t ch;
        if (uart_read_bytes(UART_NUM_0, &ch, 1, pdMS_TO_TICKS(100)) != 1) {
            continue;
        }

        if (ch == '\n' && skip_lf) {
            skip_lf = false;
            continue;
        }
        skip_lf = false;

        if (ch == '\r' || ch == '\n') {
            skip_lf = (ch == '\r');
            buffer[length] = 0;
            length = 0;
            putchar('\n');
            cli_line(buffer);
            if (cli_mode == CLI_NORMAL) {
                printf("cocis > ");
            }
            fflush(stdout);
        }
        else if ((ch == 0x08 || ch == 0x7F) && length > 0) {
            length--;
            if (cli_mode == CLI_NORMAL) {
                printf("\b \b");
                fflush(stdout);
            }
        }
        else if (ch >= 0x20 && ch < 0x7F && length < (int)sizeof(buffer) - 1) {
            buffer[length++] = (char)ch;
            if (cli_mode == CLI_NORMAL) {
                putchar(ch);
                fflush(stdout);
            }
        }
    }
}