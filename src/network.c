#include "cocis_app.h"

#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_sntp.h"
#include "esp_http_server.h"
#include "cJSON.h"

static const char *TAG = "cocis_net";

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


/* ==================================================================
 * HTTP SERVER (READ-ONLY DASHBOARD)
 * ================================================================== */

static const char HTML_PAGE[] =
"<!DOCTYPE html><html><head><title>CoCIS Dashboard</title>"
"<meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>"
"<style>"
"body{font-family:Arial;background:#0f172a;color:#e5e7eb;margin:20px;}"
"h1{color:#38bdf8;} .card{background:#111827;border:1px solid #1f2937;border-radius:10px;padding:16px;margin-bottom:16px;}"
".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:16px;}"
".value{font-size:24px;font-weight:bold;} .ok{color:#22c55e;} .bad{color:#ef4444;} .warn{color:#f59e0b;}"
"table{width:100%;border-collapse:collapse;} th,td{padding:8px;border-bottom:1px solid #1f2937;text-align:left;}"
"input{background:#0f172a;color:#e5e7eb;border:1px solid #1f2937;border-radius:6px;padding:8px;margin:6px;}"
"button{background:#2563eb;color:white;border:none;border-radius:6px;padding:8px 12px;cursor:pointer;}"
"</style></head><body>"
"<h1>CoCIS Smart Control Dashboard</h1>"
"<div class='grid'>"
"  <div class='card'><div>Time</div><div class='value' id='time'>-</div></div>"
"  <div class='card'><div>WiFi</div><div class='value' id='wifi'>-</div></div>"
"  <div class='card'><div>NTP</div><div class='value' id='ntp'>-</div></div>"
"  <div class='card'><div>SD Card</div><div class='value' id='sd'>-</div></div>"
"  <div class='card'><div>Mode</div><div class='value' id='mode'>-</div></div>"
"  <div class='card'><div>Occupancy</div><div class='value ok' id='occ'>-</div></div>"
"  <div class='card'><div>Schedule</div><div class='value' id='sched'>-</div></div>"
"</div>"
"<div class='card'><h2>Rooms</h2><table><thead><tr><th>Room</th><th>Occupied</th><th>Lights</th><th>Distance</th></tr></thead><tbody id='rooms'></tbody></table></div>"
"<div class='card'><h2>People Inside</h2><div id='inside'>-</div></div>"
"<div class='card'><h2>Daily Counters</h2><table><thead><tr><th>Date</th><th>Entries</th><th>Exits</th><th>Denied</th></tr></thead><tbody id='counters'></tbody></table></div>"
"<div class='card'><h2>Recent Alerts</h2><div id='alerts'>None</div></div>"
"<div class='card'><h2>Log Search</h2>"
"<input id='search_q' placeholder='Search UID, event, date...'><button onclick='searchLogs()'>Search</button>"
"<pre id='logs' style='max-height:300px;overflow:auto;background:#0f172a;padding:10px;border-radius:6px;'></pre></div>"
"<script>"
"function setText(id,t,c){var e=document.getElementById(id);e.textContent=t;e.className='value '+(c||'');}"
"function boolText(v){return v?'YES':'NO';}"
"async function refresh(){"
"  try{"
"    var r=await fetch('/api/status');if(!r.ok)throw 0;var d=await r.json();"
"    setText('time',d.time||'-');"
"    setText('wifi',d.wifi?'CONNECTED ('+d.ip+')':'OFFLINE',d.wifi?'ok':'bad');"
"    setText('ntp',d.ntp?'SYNCED':'WAITING',d.ntp?'ok':'warn');"
"    setText('sd',d.sd?'MOUNTED':'NOT READY',d.sd?'ok':'warn');"
"    setText('mode',d.emergency?'EMERGENCY':'NORMAL',d.emergency?'bad':'ok');"
"    setText('occ',d.occupancy,'ok');"
"    var s=d.schedule?d.schedule.start+'-'+d.schedule.end+' ('+(d.schedule.open?'OPEN':'CLOSED')+')':'-';"
"    setText('sched',s,'ok');"
"    var rh='';for(var i=0;i<(d.rooms||[]).length;i++){"
"      var rm=d.rooms[i];"
"      rh+='<tr><td>Room '+rm.id+'</td><td class=\"'+(rm.occupied?'ok':'')+'\">'+boolText(rm.occupied)+'</td>';"
"      rh+='<td class=\"'+(rm.lights_on?'ok':'')+'\">'+boolText(rm.lights_on)+'</td>';"
"      rh+='<td>'+(rm.dist_cm>0?rm.dist_cm+' cm':'-')+'</td></tr>';"
"    }document.getElementById('rooms').innerHTML=rh;"
"    var ins=d.inside||[];document.getElementById('inside').textContent=ins.length?ins.join(', '):'None';"
"    var ch='';for(var i=0;i<(d.counters||[]).length;i++){"
"      var c=d.counters[i];ch+='<tr><td>'+c.date+'</td><td>'+c.entries+'</td><td>'+c.exits+'</td><td>'+c.denied+'</td></tr>';"
"    }document.getElementById('counters').innerHTML=ch||'<tr><td colspan=4>No data</td></tr>';"
"    document.getElementById('alerts').innerHTML=(d.alerts&&d.alerts.length)?d.alerts.join('<br>'):'None';"
"  }catch(e){setText('wifi','ERROR','bad');}"
"}"
"async function searchLogs(){"
"  var q=document.getElementById('search_q').value;"
"  try{"
"    var r=await fetch('/api/logs?q='+encodeURIComponent(q));var d=await r.json();"
"    document.getElementById('logs').textContent=(d.matches||[]).join('\\n')||'No matches';"
"  }catch(e){document.getElementById('logs').textContent='Search failed';}"
"}"
"refresh();setInterval(refresh,2000);"
"</script></body></html>";

static bool ci_contains(const char *hay, const char *needle) {
    if (!needle || !*needle) return true;
    size_t nl = strlen(needle);
    for (; *hay; hay++) {
        size_t i = 0;
        while (i < nl && hay[i] && tolower((unsigned char)hay[i]) == tolower((unsigned char)needle[i])) i++;
        if (i == nl) return true;
    }
    return false;
}

static esp_err_t get_home_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, HTML_PAGE, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t api_status_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    if (!root) return ESP_FAIL;
    
    char ts[24];
    time_t current_time = time(NULL);
    format_time(current_time, ts, sizeof(ts));
    
    // FIX: Calculate this BEFORE locking RESOURCE_CONFIG to avoid self-deadlock
    bool is_open = schedule_is_open(current_time);

    cJSON_AddStringToObject(root, "time", ts);
    cJSON_AddBoolToObject(root, "wifi", wifi_is_connected());
    cJSON_AddStringToObject(root, "ip", wifi_get_ip());
    cJSON_AddBoolToObject(root, "ntp", time_is_synced());
    cJSON_AddBoolToObject(root, "sd", sd_card_ok);
    cJSON_AddBoolToObject(root, "emergency", emergency_active());
    
    if (lock_resource(RESOURCE_DATA, 200)) {
        cJSON_AddNumberToObject(root, "occupancy", people_inside_count);
        
        cJSON *inside_arr = cJSON_CreateArray();
        for (int i = 0; i < MAX_USERS; i++) {
            if (registered_users[i].used && registered_users[i].inside) {
                cJSON_AddItemToArray(inside_arr, cJSON_CreateString(registered_users[i].name));
            }
        }
        cJSON_AddItemToObject(root, "inside", inside_arr);
        
        cJSON *counters_arr = cJSON_CreateArray();
        for (int i = 0; i < 7; i++) {
            if (daily_statistics[i].date != 0) {
                cJSON *day = cJSON_CreateObject();
                cJSON_AddNumberToObject(day, "date", daily_statistics[i].date);
                cJSON_AddNumberToObject(day, "entries", daily_statistics[i].entries);
                cJSON_AddNumberToObject(day, "exits", daily_statistics[i].exits);
                cJSON_AddNumberToObject(day, "denied", daily_statistics[i].denied);
                cJSON_AddItemToArray(counters_arr, day);
            }
        }
        cJSON_AddItemToObject(root, "counters", counters_arr);
        
        cJSON *alerts_arr = cJSON_CreateArray();
        int total = alert_total;
        int count = total < 8 ? total : 8;
        for (int i = 0; i < count; i++) {
            cJSON_AddItemToArray(alerts_arr, cJSON_CreateString(recent_alerts[(total - count + i) % 8]));
        }
        cJSON_AddItemToObject(root, "alerts", alerts_arr);
        
        cJSON *rooms_arr = cJSON_CreateArray();
        for (int i = 0; i < 2; i++) {
            cJSON *room = cJSON_CreateObject();
            cJSON_AddNumberToObject(room, "id", i + 1);
            cJSON_AddBoolToObject(room, "occupied", room_states[i].occupied);
            cJSON_AddBoolToObject(room, "lights_on", room_states[i].light_on);
            cJSON_AddNumberToObject(room, "dist_cm", room_states[i].distance_cm);
            cJSON_AddItemToArray(rooms_arr, room);
        }
        cJSON_AddItemToObject(root, "rooms", rooms_arr);
        
        unlock_resource(RESOURCE_DATA);
    }
    
    if (lock_resource(RESOURCE_CONFIG, 200)) {
        cJSON *sched = cJSON_CreateObject();
        char start_str[8], end_str[8];

        snprintf(start_str, sizeof(start_str), "%02d:%02d", schedule_start_minutes / 60, schedule_start_minutes % 60);
        snprintf(end_str, sizeof(end_str), "%02d:%02d", schedule_end_minutes / 60, schedule_end_minutes % 60);
        
        cJSON_AddStringToObject(sched, "start", start_str);
        cJSON_AddStringToObject(sched, "end", end_str);
        cJSON_AddBoolToObject(sched, "open", is_open);
        cJSON_AddItemToObject(root, "schedule", sched);
        
        unlock_resource(RESOURCE_CONFIG);
    }
    
    char *resp = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    
    free(resp);
    cJSON_Delete(root);

    return ESP_OK;
}

static esp_err_t api_logs_handler(httpd_req_t *req) {
    char query[128] = {0};
    char search_term[64] = {0};
    
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        httpd_query_key_value(query, "q", search_term, sizeof(search_term));
    }
    
    cJSON *root = cJSON_CreateObject();
    cJSON *matches = cJSON_CreateArray();
    
    if (lock_resource(RESOURCE_LOG, 200)) {
        int total = log_ring_total;
        int count = total < RING_N ? total : RING_N;
        for (int i = 0; i < count; i++) {
            int idx = (total - count + i) % RING_N;
            if (ci_contains(log_ring[idx], search_term)) {
                cJSON_AddItemToArray(matches, cJSON_CreateString(log_ring[idx]));
            }
        }
        unlock_resource(RESOURCE_LOG);
    }
    
    cJSON_AddItemToObject(root, "matches", matches);
    
    char *resp = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    
    free(resp);
    cJSON_Delete(root);

    return ESP_OK;
}

void start_webserver(void) {

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192; 

    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {

        httpd_uri_t home_uri = { 
            .uri = "/",
            .method = HTTP_GET, 
            .handler = get_home_handler 
        };

        httpd_uri_t status_uri = { 
            .uri = "/api/status", 
            .method = HTTP_GET, 
            .handler = api_status_handler
        };

        httpd_uri_t logs_uri = { 
            .uri = "/api/logs", 
            .method = HTTP_GET, 
            .handler = api_logs_handler 
        };
        
        httpd_register_uri_handler(server, &home_uri);
        httpd_register_uri_handler(server, &status_uri);
        httpd_register_uri_handler(server, &logs_uri);
        
        ESP_LOGI(TAG, "HTTP dashboard started on port %d", config.server_port);
    } else {
        ESP_LOGE(TAG, "Failed to start HTTP server");
    }
}