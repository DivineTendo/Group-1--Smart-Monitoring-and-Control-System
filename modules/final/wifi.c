#include "cocis_app.h"

#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_sntp.h"

static const char *TAG = "cocis_wifi";

#define WIFI_NAMESPACE   "wifi"
#define WIFI_SSID_MAX    33
#define WIFI_PASS_MAX    65

static volatile bool wifi_connected = false;
static volatile bool ntp_time_synced = false;
static bool sntp_started = false;

static char wifi_ssid[WIFI_SSID_MAX] = WIFI_DEFAULT_SSID;
static char wifi_password[WIFI_PASS_MAX] = WIFI_DEFAULT_PASS;
static char wifi_ip[16] = "0.0.0.0";

/* ==================================================================
 * NTP / SNTP
 * ================================================================== */
static void time_sync_callback(struct timeval *tv)
{
    (void)tv;

    ntp_time_synced = true;

    char timestamp[24];
    format_time(time(NULL), timestamp, sizeof(timestamp));

    ESP_LOGI(TAG, "NTP time synced: %s", timestamp);
}

static void start_sntp(void)
{
    if (sntp_started) {
        return;
    }

    sntp_started = true;

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);

    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.google.com");
    esp_sntp_setservername(2, "time.nist.gov");

    esp_sntp_set_time_sync_notification_cb(time_sync_callback);

    esp_sntp_init();

    ESP_LOGI(TAG, "SNTP started");
}

/* ==================================================================
 * WIFI EVENT HANDLER
 * ================================================================== */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            ESP_LOGI(TAG, "WiFi station started, connecting...");
            esp_wifi_connect();
        }
        else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            wifi_connected = false;
            snprintf(wifi_ip, sizeof(wifi_ip), "0.0.0.0");

            ESP_LOGW(TAG, "WiFi disconnected, retrying...");
            esp_wifi_connect();
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;

        snprintf(wifi_ip, sizeof(wifi_ip), IPSTR, IP2STR(&event->ip_info.ip));

        wifi_connected = true;

        ESP_LOGI(TAG, "WiFi connected, IP: %s", wifi_ip);

        /* Once WiFi has IP, start internet time sync */
        start_sntp();
    }
}

/* ==================================================================
 * WIFI CREDENTIAL STORAGE
 * ================================================================== */
static void wifi_load_credentials(void)
{
    nvs_handle_t handle;

    if (nvs_open(WIFI_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        ESP_LOGW(TAG, "No saved WiFi credentials, using defaults");
        return;
    }

    size_t length = sizeof(wifi_ssid);
    if (nvs_get_str(handle, "ssid", wifi_ssid, &length) != ESP_OK) {
        snprintf(wifi_ssid, sizeof(wifi_ssid), "%s", WIFI_DEFAULT_SSID);
    }

    length = sizeof(wifi_password);
    if (nvs_get_str(handle, "pass", wifi_password, &length) != ESP_OK) {
        snprintf(wifi_password, sizeof(wifi_password), "%s", WIFI_DEFAULT_PASS);
    }

    nvs_close(handle);

    ESP_LOGI(TAG, "WiFi credentials loaded, SSID: %s", wifi_ssid);
}

bool wifi_save_credentials(const char *ssid, const char *password)
{
    if (ssid == NULL || ssid[0] == '\0') {
        return false;
    }

    nvs_handle_t handle;

    if (nvs_open(WIFI_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        return false;
    }

    esp_err_t err_ssid = nvs_set_str(handle, "ssid", ssid);
    esp_err_t err_pass = nvs_set_str(handle, "pass", password != NULL ? password : "");
    esp_err_t err_commit = nvs_commit(handle);

    nvs_close(handle);

    if (err_ssid == ESP_OK && err_pass == ESP_OK && err_commit == ESP_OK) {
        snprintf(wifi_ssid, sizeof(wifi_ssid), "%s", ssid);
        snprintf(wifi_password, sizeof(wifi_password), "%s", password != NULL ? password : "");
        return true;
    }

    return false;
}

/* ==================================================================
 * WIFI START / RECONNECT
 * ================================================================== */
bool wifi_start(void)
{
    wifi_load_credentials();

    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(err));
        return false;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s", esp_err_to_name(err));
        return false;
    }

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));

    esp_event_handler_instance_t instance_any_id = NULL;
    esp_event_handler_instance_t instance_got_ip = NULL;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id
    ));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip
    ));

    wifi_config_t wifi_config = { 0 };

    strncpy((char *)wifi_config.sta.ssid, wifi_ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, wifi_password, sizeof(wifi_config.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi started, connecting to '%s'", wifi_ssid);

    return true;
}

void wifi_reconnect(void)
{
    wifi_config_t wifi_config = { 0 };

    strncpy((char *)wifi_config.sta.ssid, wifi_ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, wifi_password, sizeof(wifi_config.sta.password) - 1);

    esp_wifi_disconnect();
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_connect();

    ESP_LOGI(TAG, "WiFi reconnecting to '%s'", wifi_ssid);
}

/* ==================================================================
 * STATUS HELPERS
 * ================================================================== */
bool wifi_is_connected(void)
{
    return wifi_connected;
}

bool time_is_synced(void)
{
    return ntp_time_synced;
}

const char *wifi_get_ip(void)
{
    return wifi_ip;
}

void wifi_print_status(void)
{
    char timestamp[24];
    format_time(time(NULL), timestamp, sizeof(timestamp));

    printf("WiFi SSID : %s\n", wifi_ssid);
    printf("WiFi state: %s\n", wifi_connected ? "connected" : "not connected");
    printf("IP address: %s\n", wifi_ip);
    printf("NTP sync  : %s\n", ntp_time_synced ? "synced" : "not synced");
    printf("Time      : %s\n", timestamp);
}