#include "web_server.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "cJSON.h"
#include "rda5807m.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "WEB_SERVER";

extern float current_frequency;
extern volatile int battery_percent;
extern SemaphoreHandle_t i2c_mutex;
extern volatile bool force_ui_update;

static const char* INDEX_HTML = 
"<!DOCTYPE html>\n"
"<html>\n"
"<head>\n"
"<meta charset=\"utf-8\">\n"
"<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
"<title>Radio Dashboard</title>\n"
"<style>\n"
"body {\n"
"    background: linear-gradient(135deg, #0f172a, #1e1b4b);\n"
"    color: #e2e8f0;\n"
"    font-family: 'Segoe UI', sans-serif;\n"
"    margin: 0; padding: 20px;\n"
"    display: flex; flex-direction: column;\n"
"    align-items: center; justify-content: center;\n"
"    min-height: 100vh;\n"
"}\n"
".glass-panel {\n"
"    background: rgba(255, 255, 255, 0.05);\n"
"    backdrop-filter: blur(20px);\n"
"    -webkit-backdrop-filter: blur(20px);\n"
"    border: 1px solid rgba(255, 255, 255, 0.1);\n"
"    box-shadow: 0 8px 32px 0 rgba(0, 0, 0, 0.37);\n"
"    border-radius: 20px;\n"
"    padding: 30px;\n"
"    max-width: 400px; width: 100%;\n"
"    box-sizing: border-box;\n"
"    text-align: center;\n"
"}\n"
"h1 { margin-top: 0; font-size: 1.5rem; color: #a5b4fc; }\n"
".freq-display {\n"
"    font-size: 3.5rem;\n"
"    font-weight: bold;\n"
"    color: #818cf8;\n"
"    text-shadow: 0 0 10px rgba(129, 140, 248, 0.5);\n"
"    margin: 20px 0;\n"
"}\n"
"input[type=range] {\n"
"    width: 100%;\n"
"    margin: 20px 0;\n"
"    accent-color: #818cf8;\n"
"}\n"
".telemetry-grid {\n"
"    display: grid;\n"
"    grid-template-columns: 1fr 1fr 1fr;\n"
"    gap: 15px;\n"
"    margin-top: 25px;\n"
"}\n"
".card {\n"
"    background: rgba(0, 0, 0, 0.2);\n"
"    padding: 15px 10px;\n"
"    border-radius: 12px;\n"
"    border: 1px solid rgba(255,255,255,0.05);\n"
"    transition: all 0.3s ease;\n"
"}\n"
".card:hover { background: rgba(255, 255, 255, 0.05); }\n"
".card span { display: block; font-size: 0.8rem; color: #94a3b8; }\n"
".card strong { display: block; font-size: 1.2rem; margin-top: 5px; }\n"
"</style>\n"
"</head>\n"
"<body>\n"
"<div class=\"glass-panel\">\n"
"    <h1>FM Radio Tuning</h1>\n"
"    <div class=\"freq-display\"><span id=\"freqVal\">--.-</span> <span style=\"font-size: 1rem\">MHz</span></div>\n"
"    <div style=\"display: flex; align-items: center; justify-content: center; width: 100%; gap: 15px; margin: 20px 0;\">\n"
"        <button id=\"btnMinus\" style=\"font-size: 1.5rem; width: 45px; height: 45px; border-radius: 50%; background: rgba(129, 140, 248, 0.2); border: 1px solid rgba(129, 140, 248, 0.4); color: #a5b4fc; cursor: pointer; transition: 0.2s;\">-</button>\n"
"        <input type=\"range\" id=\"freqSlider\" min=\"87.5\" max=\"108.0\" step=\"0.1\" value=\"100.0\" style=\"flex-grow: 1; margin: 0;\">\n"
"        <button id=\"btnPlus\" style=\"font-size: 1.5rem; width: 45px; height: 45px; border-radius: 50%; background: rgba(129, 140, 248, 0.2); border: 1px solid rgba(129, 140, 248, 0.4); color: #a5b4fc; cursor: pointer; transition: 0.2s;\">+</button>\n"
"    </div>\n"
"    <div class=\"telemetry-grid\">\n"
"        <div class=\"card\"><span>Battery</span><strong id=\"batVal\">--%</strong></div>\n"
"        <div class=\"card\"><span>RSSI</span><strong id=\"rssiVal\">--</strong></div>\n"
"        <div class=\"card\"><span>Status</span><strong id=\"validVal\">--</strong></div>\n"
"    </div>\n"
"</div>\n"
"<script>\n"
"let timer;\n"
"const freqSlider = document.getElementById('freqSlider');\n"
"const freqVal = document.getElementById('freqVal');\n"
"\n"
"function updateUI(freq, bat, rssi, valid) {\n"
"    if(document.activeElement !== freqSlider) {\n"
"        freqSlider.value = freq;\n"
"        freqVal.innerText = parseFloat(freq).toFixed(1);\n"
"    }\n"
"    document.getElementById('batVal').innerText = bat + '%';\n"
"    document.getElementById('rssiVal').innerText = rssi;\n"
"    document.getElementById('validVal').innerText = valid ? 'VALID' : 'NOISE';\n"
"    document.getElementById('validVal').style.color = valid ? '#4ade80' : '#f87171';\n"
"}\n"
"\n"
"function fetchStatus() {\n"
"    fetch('/api/status')\n"
"    .then(r => r.json())\n"
"    .then(data => updateUI(data.freq, data.bat, data.rssi, data.valid))\n"
"    .catch(console.error);\n"
"}\n"
"\n"
"freqSlider.addEventListener('input', (e) => {\n"
"    freqVal.innerText = parseFloat(e.target.value).toFixed(1);\n"
"    clearTimeout(timer);\n"
"    timer = setTimeout(() => {\n"
"        fetch('/api/set_freq', {\n"
"            method: 'POST',\n"
"            headers: {'Content-Type': 'application/json'},\n"
"            body: JSON.stringify({freq: parseFloat(e.target.value)})\n"
"        });\n"
"    }, 300);\n"
"});\n"
"\n"
"document.getElementById('btnMinus').addEventListener('click', () => {\n"
"    freqSlider.value = Math.max(87.5, parseFloat(freqSlider.value) - 0.1).toFixed(1);\n"
"    freqSlider.dispatchEvent(new Event('input'));\n"
"});\n"
"document.getElementById('btnPlus').addEventListener('click', () => {\n"
"    freqSlider.value = Math.min(108.0, parseFloat(freqSlider.value) + 0.1).toFixed(1);\n"
"    freqSlider.dispatchEvent(new Event('input'));\n"
"});\n"
"\n"
"setInterval(fetchStatus, 2000);\n"
"fetchStatus();\n"
"</script>\n"
"</body>\n"
"</html>\n";

static esp_err_t get_index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t api_status_handler(httpd_req_t *req) {
    int rssi = 0;
    bool fm_true = false;
    bool stereo = false;
    
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    rda5807_get_telemetry(&rssi, &fm_true, &stereo);
    xSemaphoreGive(i2c_mutex);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "freq", current_frequency);
    cJSON_AddNumberToObject(root, "bat", battery_percent);
    cJSON_AddNumberToObject(root, "rssi", rssi);
    cJSON_AddBoolToObject(root, "valid", fm_true);

    const char *sys_info = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    if (sys_info) {
        httpd_resp_send(req, sys_info, HTTPD_RESP_USE_STRLEN);
        free((void *)sys_info);
    } else {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "JSON Error");
    }
    
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t api_set_freq_handler(httpd_req_t *req) {
    char buf[100];
    int ret, remaining = req->content_len;

    if (remaining > sizeof(buf) - 1) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Payload too large");
        return ESP_FAIL;
    }

    if ((ret = httpd_req_recv(req, buf, remaining)) <= 0) {
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *freq_item = cJSON_GetObjectItem(root, "freq");
    if (cJSON_IsNumber(freq_item)) {
        float f = freq_item->valuedouble;
        if (f >= FREQ_MIN && f <= FREQ_MAX) {
            xSemaphoreTake(i2c_mutex, portMAX_DELAY);
            current_frequency = f;
            rda5807_set_frequency(f);
            xSemaphoreGive(i2c_mutex);
            
            force_ui_update = true;
        }
    }

    cJSON_Delete(root);
    httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static httpd_uri_t uri_get_index = {
    .uri      = "/",
    .method   = HTTP_GET,
    .handler  = get_index_handler,
    .user_ctx = NULL
};

static httpd_uri_t uri_api_status = {
    .uri      = "/api/status",
    .method   = HTTP_GET,
    .handler  = api_status_handler,
    .user_ctx = NULL
};

static httpd_uri_t uri_api_set_freq = {
    .uri      = "/api/set_freq",
    .method   = HTTP_POST,
    .handler  = api_set_freq_handler,
    .user_ctx = NULL
};

void start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.core_id = 0; // PRO_CPU (Core 0)
    config.max_uri_handlers = 8;
    
    ESP_LOGI(TAG, "Starting Web GUI on Core 0");
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_get_index);
        httpd_register_uri_handler(server, &uri_api_status);
        httpd_register_uri_handler(server, &uri_api_set_freq);
    } else {
        ESP_LOGE(TAG, "Failed to start web server");
    }
}
