#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "driver/i2c.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_random.h"

// ADC для батареи
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// U8G2 для экрана
#include "u8g2.h"
#include "rda5807m.h"

// Wi-Fi и сеть
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_event.h"

#include "web_server.h"
// ================= НАСТРОЙКИ ПИНОВ И ПАРАМЕТРОВ =================
#define I2C_MASTER_PORT             I2C_NUM_0
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_FREQ_HZ          100000 
#define I2C_MASTER_TIMEOUT_MS       100

#define NVS_SAVE_DELAY_MS   3000
#define ENCODER_PIN_CLK             25
#define ENCODER_PIN_DT              26
#define ENCODER_PIN_SW              27
#define ENCODER_PCNT_HIGH_LIMIT     100
#define ENCODER_PCNT_LOW_LIMIT      -100

#define BATT_ADC_UNIT               ADC_UNIT_1
#define BATT_ADC_CHANNEL            ADC_CHANNEL_7   // GPIO35 = ADC1_CH7

// RDA5807 constants are now inside rda5807m.h

static const char *TAG = "RADIO_APP";

// ================= ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ =================
SemaphoreHandle_t i2c_mutex;
pcnt_unit_handle_t pcnt_unit = NULL;
float current_frequency = 100.0f;
volatile int battery_percent = 0;
u8g2_t u8g2;

// Батарея
adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t adc1_cali_handle = NULL;
bool do_calibration = false;
float moving_avg_volts = 0.0f;

// Состояние UI
typedef enum {
    MODE_SCALE = 0,
    MODE_IP = 1,
    MODE_TELEMETRY = 2
} DisplayMode;

volatile DisplayMode current_mode = MODE_SCALE;
volatile bool force_ui_update = true;

// Service Menu & Network
volatile bool in_service_menu = false;
volatile int service_menu_state = 0; // 0=Main, 1=WifiConfirm, 2=Bright, 3=ADCOffset, 4=Step
volatile int service_menu_cursor = 0;
char wifi_password[16] = {0};

// Button handling (task-side double-click detection)
volatile uint32_t last_button_press_tick = 0;
volatile uint8_t button_press_count = 0;

// Settings Profile
uint8_t oled_brightness = 255;
int32_t adc_offset_mv = 0;
uint32_t freq_step_khz = 100;

static void load_nvs_settings() {
    nvs_handle_t nvs_handle;
    if (nvs_open("storage", NVS_READWRITE, &nvs_handle) == ESP_OK) {
        size_t required_size = sizeof(wifi_password);
        if (nvs_get_str(nvs_handle, "wifi_pass", wifi_password, &required_size) != ESP_OK) {
            uint32_t r1 = esp_random() % 10000;
            uint32_t r2 = esp_random() % 10000;
            snprintf(wifi_password, sizeof(wifi_password), "%04u%04u", (unsigned)r1, (unsigned)r2);
            nvs_set_str(nvs_handle, "wifi_pass", wifi_password);
        }
        
        if (nvs_get_u8(nvs_handle, "oled_br", &oled_brightness) != ESP_OK) oled_brightness = 255;
        if (nvs_get_i32(nvs_handle, "adc_off", &adc_offset_mv) != ESP_OK) adc_offset_mv = 0;
        if (nvs_get_u32(nvs_handle, "freq_step", &freq_step_khz) != ESP_OK) freq_step_khz = 100;

        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
    }
}

// ================= ИНИЦИАЛИЗАЦИЯ I2C =================
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_PORT, &conf);
    if (err != ESP_OK) return err;
    return i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
}

// ================= ИНИЦИАЛИЗАЦИЯ БАТАРЕИ (ADC) =================
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle) {
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) calibrated = true;
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
            .default_vref = 1100,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) calibrated = true;
    }
#endif

    *out_handle = handle;
    return calibrated;
}

static void battery_init(void) {
    ESP_LOGI(TAG, "Инициализация ADC для батареи (GPIO35)...");
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = BATT_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, BATT_ADC_CHANNEL, &config));

    do_calibration = adc_calibration_init(BATT_ADC_UNIT, BATT_ADC_CHANNEL, ADC_ATTEN_DB_12, &adc1_cali_handle);
}

static float get_battery_voltage() {
    int adc_raw = 0;
    int voltage_mv = 0;
    adc_oneshot_read(adc1_handle, BATT_ADC_CHANNEL, &adc_raw);
    
    if (do_calibration) {
        adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw, &voltage_mv);
    } else {
        voltage_mv = adc_raw * 3000 / 4095;
    }
    // Делитель напряжения 1:2 (10k + 10k)
    float bat_voltage = (voltage_mv * 2.0f) / 1000.0f;
    // Применяем ADC Offset из сервисного меню
    bat_voltage += (adc_offset_mv / 1000.0f);
    if (bat_voltage < 0.0f) bat_voltage = 0.0f;
    return bat_voltage;
}

// ISR минимальна: только фиксируем нажатие, всю логику обрабатывает task
static void IRAM_ATTR encoder_sw_isr_handler(void* arg) {
    if (gpio_get_level(ENCODER_PIN_SW) != 0) return; // Игнорируем отпускание
    
    uint32_t now = xTaskGetTickCountFromISR();
    if ((now - last_button_press_tick) < pdMS_TO_TICKS(200)) return; // Аппаратный debounce
    last_button_press_tick = now;
    button_press_count++;
}

// ================= ИНИЦИАЛИЗАЦИЯ ENCODER (PCNT v5.x) =================
static void encoder_init(void) {
    ESP_LOGI(TAG, "Установка PCNT для KY-040 (v5.x API)");

    pcnt_unit_config_t unit_config = {
        .high_limit = ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = ENCODER_PCNT_LOW_LIMIT,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    gpio_set_pull_mode(ENCODER_PIN_CLK, GPIO_PULLUP_ENABLE);
    gpio_set_pull_mode(ENCODER_PIN_DT, GPIO_PULLUP_ENABLE);

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000, // Аппаратный лимит фильтра ESP32 ~12.5 мкс (1023 такта), ставим безопасно 1 мкс.
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    // Настройка каналов для полноценного квадратурного чтения
    pcnt_chan_config_t chan_a_config = { .edge_gpio_num = ENCODER_PIN_CLK, .level_gpio_num = ENCODER_PIN_DT };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    pcnt_chan_config_t chan_b_config = { .edge_gpio_num = ENCODER_PIN_DT, .level_gpio_num = ENCODER_PIN_CLK };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    // Настройка кнопки энкодера
    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << ENCODER_PIN_SW),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&btn_cfg));
    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(isr_err);
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENCODER_PIN_SW, encoder_sw_isr_handler, NULL));
}

// ================= РАБОТА С RDA5807M =================
// Логика управления радио и RDS вынесена в библиотеку rda5807m.c

// ================= РАБОТА С NVS (ПАМЯТЬ) =================
// ... (оставим те же без изменений)
static void load_nvs_freq() {
    nvs_handle_t nvs_handle;
    if (nvs_open("storage", NVS_READONLY, &nvs_handle) == ESP_OK) {
        uint32_t saved_freq_int;
        if (nvs_get_u32(nvs_handle, "freq", &saved_freq_int) == ESP_OK) {
            current_frequency = saved_freq_int / 10.0f;
        }
        nvs_close(nvs_handle);
    }
}

static void save_nvs_freq(float freq) {
    nvs_handle_t nvs_handle;
    if (nvs_open("storage", NVS_READWRITE, &nvs_handle) == ESP_OK) {
        uint32_t freq_int = (uint32_t)(freq * 10.0f + 0.5f);
        nvs_set_u32(nvs_handle, "freq", freq_int);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
    }
}

// ================= ИНТЕГРАЦИЯ U8G2 И ESP-IDF I2C =================
uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    static uint8_t buffer[2048];
    static uint16_t buf_idx = 0;
    
    switch(msg) {
        case U8X8_MSG_BYTE_SEND:
            if (buf_idx + arg_int <= sizeof(buffer)) {
                memcpy(buffer + buf_idx, arg_ptr, arg_int);
                buf_idx += arg_int;
            }
            break;
        case U8X8_MSG_BYTE_START_TRANSFER:
            buf_idx = 0;
            break;
        case U8X8_MSG_BYTE_END_TRANSFER:
            i2c_master_write_to_device(I2C_MASTER_PORT, u8x8_GetI2CAddress(u8x8) >> 1, buffer, buf_idx, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
            break;
        default:
            return 1;
    }
    return 1;
}

uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch(msg) {
        case U8X8_MSG_DELAY_MILLI:
            vTaskDelay(pdMS_TO_TICKS(arg_int));
            break;
    }
    return 1;
}

// ================= РАБОТА С OLED =================
static void oled_init(void) {
    u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_hw_i2c, u8x8_gpio_and_delay);
    u8g2_SetI2CAddress(&u8g2, 0x3C * 2);

    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0); 
    u8g2_SetContrast(&u8g2, oled_brightness);
    xSemaphoreGive(i2c_mutex);
}

static void oled_update_ui(float freq, bool is_tuning, int battery_pct, bool battery_low_state, int rssi_val, bool fm_true_val, bool stereo_val) {
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    
    u8g2_ClearBuffer(&u8g2);
    
    if (in_service_menu) {
        u8g2_SetFont(&u8g2, u8g2_font_5x7_tr);
        if (service_menu_state == 0) {
            u8g2_DrawStr(&u8g2, 22, 8, "--- SERVICE MENU ---");
            const char* items[] = {"1. Reset Wi-Fi", "2. Brightness", "3. ADC Offset", "4. Tuning Step", "5. Exit"};
            for (int i=0; i<5; i++) {
                if (service_menu_cursor == i) u8g2_DrawBox(&u8g2, 5, 12 + i*10, 118, 10);
                u8g2_SetDrawColor(&u8g2, service_menu_cursor == i ? 0 : 1);
                u8g2_DrawStr(&u8g2, 10, 20 + i*10, items[i]);
                u8g2_SetDrawColor(&u8g2, 1);
            }
        } else if (service_menu_state == 1) {
            u8g2_DrawStr(&u8g2, 10, 20, "RESET WI-FI PASS?");
            u8g2_DrawStr(&u8g2, 10, 40, "Dbl-Click: CONFIRM");
            u8g2_DrawStr(&u8g2, 10, 50, "1-Click: CANCEL");
        } else if (service_menu_state == 2) {
            u8g2_DrawStr(&u8g2, 10, 20, "BRIGHTNESS (0-255):");
            char buf[16]; snprintf(buf, sizeof(buf), "%d", oled_brightness);
            u8g2_DrawStr(&u8g2, 10, 40, buf);
        } else if (service_menu_state == 3) {
            u8g2_DrawStr(&u8g2, 10, 20, "ADC OFFSET (mV):");
            char buf[16]; snprintf(buf, sizeof(buf), "%+d mV", (int)adc_offset_mv);
            u8g2_DrawStr(&u8g2, 10, 40, buf);
        } else if (service_menu_state == 4) {
            u8g2_DrawStr(&u8g2, 10, 20, "TUNING STEP (kHz):");
            char buf[16]; snprintf(buf, sizeof(buf), "%d kHz", (int)freq_step_khz);
            u8g2_DrawStr(&u8g2, 10, 40, buf);
        }
        
        u8g2_SendBuffer(&u8g2);
        xSemaphoreGive(i2c_mutex);
        return;
    }

    if (battery_low_state) {
        u8g2_SetFont(&u8g2, u8g2_font_helvB12_tr); // крупный шрифт
        u8g2_DrawStr(&u8g2, 10, 38, "BATTERY LOW!");
    } else {
        // --- 1. Батарея (В левом верхнем углу) ---
        int bat_w = 20;
        int bat_h = 10;
        int bat_x = 2; 
        int bat_y = 2;
        
        // контур
        u8g2_DrawFrame(&u8g2, bat_x, bat_y, bat_w, bat_h);
        // пимпочка
        u8g2_DrawBox(&u8g2, bat_x + bat_w, bat_y + 3, 2, 4); 
        
        // заливка (% заряда)
        int fill_width = (battery_pct * (bat_w - 4)) / 100;
        if (fill_width > 0) {
            u8g2_DrawBox(&u8g2, bat_x + 2, bat_y + 2, fill_width, bat_h - 4);
        }
        
        // Текст %
        char bat_str[16];
        snprintf(bat_str, sizeof(bat_str), "%d%%", battery_pct);
        u8g2_SetFont(&u8g2, u8g2_font_helvR08_te);
        u8g2_DrawStr(&u8g2, bat_x + bat_w + 5, bat_y + 9, bat_str);

        // --- 2. Сигнал RSSI (в правом верхнем углу) & Tuning ---
        int rssi = rssi_val;
        int rssi_percent = (rssi * 100) / 63; 
        if (rssi_percent > 100) rssi_percent = 100;
        
        int rssi_x = 106;
        int rssi_y = 2;

        if (is_tuning) {
            u8g2_SetFont(&u8g2, u8g2_font_helvR08_te); 
            u8g2_DrawStr(&u8g2, rssi_x - 42, rssi_y + 9, "Tuning");
        }

        // Отрисовка 4-х столбиков антенны
        int bars = (rssi_percent * 4) / 100;
        if (bars > 4) bars = 4;
        
        for(int i = 0; i < 4; i++) {
            int bh = 4 + i * 2;
            int by = rssi_y + 10 - bh;
            int bx = rssi_x + i * 5;
            if (i < bars) {
                u8g2_DrawBox(&u8g2, bx, by, 3, bh);
            } else {
                u8g2_DrawFrame(&u8g2, bx, by, 3, bh);
            }
        }

        // --- 3. Отображение частоты по центру ---
        char freq_str[16];
        snprintf(freq_str, sizeof(freq_str), "%.1f MHz", (double)freq);
        u8g2_SetFont(&u8g2, u8g2_font_logisoso18_tr);
        u8g2_DrawStr(&u8g2, 10, 36, freq_str);

        // --- 4. Отображение нижней панели (UI Modes) ---
        switch (current_mode) {
            case MODE_SCALE: {
                int scale_x = 18;
                int scale_y = 56;
                int scale_w = 92;
                
                // Отрисуем основную линию шкалы
                u8g2_DrawHLine(&u8g2, scale_x, scale_y, scale_w);

                // Отрисуем деления
                for (float f = 88.0f; f <= 108.0f; f += 2.0f) {
                    int tick_x = scale_x + (int)(((f - 87.5f) / 20.5f) * scale_w);
                    int tick_h = ((int)f % 4 == 0) ? 5 : 2; // длинные засечки каждые 4 МГц
                    u8g2_DrawVLine(&u8g2, tick_x, scale_y - tick_h, tick_h);
                }

                // Отрисуем ползунок текущей частоты
                int cursor_x = scale_x + (int)(((freq - 87.5f) / 20.5f) * scale_w);
                if (cursor_x < scale_x) cursor_x = scale_x;
                if (cursor_x > scale_x + scale_w) cursor_x = scale_x + scale_w;
                
                // Рисуем ползунок (жирный прямоугольник)
                u8g2_DrawBox(&u8g2, cursor_x - 1, scale_y - 7, 3, 9);
                
                // Подписи краёв шкалы 
                u8g2_SetFont(&u8g2, u8g2_font_5x7_tr);
                u8g2_DrawStr(&u8g2, scale_x - 18, scale_y + 4, "87");
                u8g2_DrawStr(&u8g2, scale_x + scale_w + 3, scale_y + 4, "108");
                break;
            }
            case MODE_IP: {
                u8g2_SetFont(&u8g2, u8g2_font_helvB08_tr);
                char ip_str[32] = "IP: Offline";
                esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
                if (netif) {
                    esp_netif_ip_info_t ip_info;
                    if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
                        if (ip_info.ip.addr != 0) {
                            if ((xTaskGetTickCount() / pdMS_TO_TICKS(2500)) % 2 == 0) {
                                snprintf(ip_str, sizeof(ip_str), "IP: " IPSTR, IP2STR(&ip_info.ip));
                            } else {
                                snprintf(ip_str, sizeof(ip_str), "PASS: %s", wifi_password);
                            }
                        }
                    }
                }
                int sw = u8g2_GetUTF8Width(&u8g2, ip_str);
                u8g2_DrawStr(&u8g2, 128 > sw ? (128 - sw)/2 : 0, 58, ip_str);
                break;
            }
            case MODE_TELEMETRY: {
                u8g2_SetFont(&u8g2, u8g2_font_helvR08_te);
                int rssi = rssi_val;
                bool fm_true = fm_true_val;
                bool stereo = stereo_val;
                
                char tele_str[40];
                snprintf(tele_str, sizeof(tele_str), "RSSI: %d | %s | %s", 
                         rssi, 
                         stereo ? "ST" : "MONO", 
                         fm_true ? "VALID" : "NOISE");
                         
                int sw = u8g2_GetUTF8Width(&u8g2, tele_str);
                u8g2_DrawStr(&u8g2, 128 > sw ? (128 - sw)/2 : 0, 58, tele_str);
                break;
            }
        }
    }
    
    u8g2_SendBuffer(&u8g2);
    xSemaphoreGive(i2c_mutex);
}

static void wifi_init_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "FM_RADIO_ESP32",
            .ssid_len = strlen("FM_RADIO_ESP32"),
            .channel = 1,
            .password = "",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    strncpy((char *)wifi_config.ap.password, wifi_password, sizeof(wifi_config.ap.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. AP ON (IP: 192.168.4.1)");
}

// ================= ОСНОВНАЯ ЗАДАЧА УПРАВЛЕНИЯ =================
void radio_task(void *pvParameter) {
    int accumulated_counts = 0;
    TickType_t last_movement_time = 0;
    TickType_t last_bat_check_time = 0;
    bool is_tuning = false;
    bool nvs_save_pending = false;
    bool battery_low_state = false;

    // Кешированная телеметрия (обновляется каждые 500мс)
    int cached_rssi = 0;
    bool cached_fm_true = false;
    bool cached_stereo = false;
    int prev_rssi_display = -1;

    float bat_buffer[20];
    int bat_idx = 0;
    float sum = 0.0f;
    for (int i = 0; i < 20; i++) {
        bat_buffer[i] = get_battery_voltage();
        sum += bat_buffer[i];
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    moving_avg_volts = sum / 20.0f;

    battery_percent = (int)((moving_avg_volts - 3.0f) / (4.2f - 3.0f) * 100.0f);
    if (battery_percent < 0) battery_percent = 0;
    if (battery_percent > 100) battery_percent = 100;

    if (moving_avg_volts > 3.0f) {
        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        rda5807_set_frequency(current_frequency);
        rda5807_get_telemetry(&cached_rssi, &cached_fm_true, &cached_stereo);
        xSemaphoreGive(i2c_mutex);
    } else {
        battery_low_state = true;
        ESP_LOGW(TAG, "BATTERY LOW! Voltage: %.2f. Radio and Encoder disabled.", moving_avg_volts);
        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        rda5807_power_down();
        xSemaphoreGive(i2c_mutex);
    }
    oled_update_ui(current_frequency, false, battery_percent, battery_low_state, cached_rssi, cached_fm_true, cached_stereo);

    while (1) {
        if (!battery_low_state) {
            // --- Обработка нажатий кнопки (task-side double-click) ---
            if (button_press_count > 0 && (xTaskGetTickCount() - last_button_press_tick) >= pdMS_TO_TICKS(450)) {
                uint8_t clicks = button_press_count;
                button_press_count = 0;

                if (clicks >= 2) {
                    // === DOUBLE CLICK ===
                    if (in_service_menu && service_menu_state == 1) {
                        // Подтверждение сброса Wi-Fi
                        nvs_handle_t nvs;
                        if (nvs_open("storage", NVS_READWRITE, &nvs) == ESP_OK) {
                            nvs_erase_key(nvs, "wifi_pass");
                            nvs_commit(nvs);
                            nvs_close(nvs);
                        }
                        esp_restart();
                    } else {
                        // Вход/выход из сервисного меню
                        in_service_menu = !in_service_menu;
                        service_menu_state = 0;
                        service_menu_cursor = 0;
                    }
                } else {
                    // === SINGLE CLICK ===
                    if (in_service_menu) {
                        if (service_menu_state == 0) {
                            if (service_menu_cursor == 0) service_menu_state = 1;
                            else if (service_menu_cursor == 1) service_menu_state = 2;
                            else if (service_menu_cursor == 2) service_menu_state = 3;
                            else if (service_menu_cursor == 3) service_menu_state = 4;
                            else if (service_menu_cursor == 4) in_service_menu = false;
                        } else if (service_menu_state == 1) {
                            service_menu_state = 0; // Отмена сброса
                        } else if (service_menu_state >= 2) {
                            // Сохранение настроек и возврат
                            nvs_handle_t nvs;
                            if (nvs_open("storage", NVS_READWRITE, &nvs) == ESP_OK) {
                                nvs_set_u8(nvs, "oled_br", oled_brightness);
                                nvs_set_i32(nvs, "adc_off", adc_offset_mv);
                                nvs_set_u32(nvs, "freq_step", freq_step_khz);
                                nvs_commit(nvs);
                                nvs_close(nvs);
                            }
                            service_menu_state = 0;
                        }
                    } else {
                        current_mode = (current_mode + 1) % 3;
                    }
                }
                force_ui_update = true;
            }

            if (force_ui_update) {
                force_ui_update = false;
                oled_update_ui(current_frequency, is_tuning, battery_percent, battery_low_state, cached_rssi, cached_fm_true, cached_stereo);
            }
            
            int count = 0;
            pcnt_unit_get_count(pcnt_unit, &count);
            pcnt_unit_clear_count(pcnt_unit);
            accumulated_counts += count;

            if (abs(accumulated_counts) >= 4) {
                int steps = accumulated_counts / 4;
                accumulated_counts %= 4;

                if (in_service_menu) {
                    if (service_menu_state == 0) {
                        service_menu_cursor += (steps > 0) ? 1 : -1;
                        if (service_menu_cursor < 0) service_menu_cursor = 4;
                        if (service_menu_cursor > 4) service_menu_cursor = 0;
                    } else if (service_menu_state == 2) {
                        int nb = (int)oled_brightness + steps * 10;
                        if (nb < 0) nb = 0;
                        if (nb > 255) nb = 255;
                        oled_brightness = nb;
                        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
                        u8g2_SetContrast(&u8g2, oled_brightness);
                        xSemaphoreGive(i2c_mutex);
                    } else if (service_menu_state == 3) {
                        adc_offset_mv += steps * 10;
                        if (adc_offset_mv < -1000) adc_offset_mv = -1000;
                        if (adc_offset_mv > 1000) adc_offset_mv = 1000;
                    } else if (service_menu_state == 4) {
                        if (steps > 0) {
                            if (freq_step_khz == 50) freq_step_khz = 100;
                            else if (freq_step_khz == 100) freq_step_khz = 200;
                        } else {
                            if (freq_step_khz == 200) freq_step_khz = 100;
                            else if (freq_step_khz == 100) freq_step_khz = 50;
                        }
                    }
                    force_ui_update = true;
                } else {
                    current_frequency += (steps * (freq_step_khz / 1000.0f));
                    if (current_frequency > FREQ_MAX) current_frequency = FREQ_MIN;
                    if (current_frequency < FREQ_MIN) current_frequency = FREQ_MAX;

                    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
                    rda5807_set_frequency(current_frequency);
                    xSemaphoreGive(i2c_mutex);
                    
                    is_tuning = true;
                    nvs_save_pending = true;
                    last_movement_time = xTaskGetTickCount();
                    
                    oled_update_ui(current_frequency, is_tuning, battery_percent, battery_low_state, cached_rssi, cached_fm_true, cached_stereo);
                }
            }

            if (is_tuning && (xTaskGetTickCount() - last_movement_time) > pdMS_TO_TICKS(NVS_SAVE_DELAY_MS)) {
                is_tuning = false;
                oled_update_ui(current_frequency, is_tuning, battery_percent, battery_low_state, cached_rssi, cached_fm_true, cached_stereo); 
            }

            if (nvs_save_pending && !is_tuning) {
                save_nvs_freq(current_frequency);
                nvs_save_pending = false;
            }
        }

        if ((xTaskGetTickCount() - last_bat_check_time) > pdMS_TO_TICKS(500)) {
            last_bat_check_time = xTaskGetTickCount();
            
            float raw_v = get_battery_voltage();
            bat_buffer[bat_idx] = raw_v;
            bat_idx = (bat_idx + 1) % 20;
            
            float current_sum = 0.0f;
            for(int i = 0; i < 20; i++) {
                current_sum += bat_buffer[i];
            }
            moving_avg_volts = current_sum / 20.0f;

            int new_pct = (int)((moving_avg_volts - 3.0f) / (4.2f - 3.0f) * 100.0f);
            if (new_pct < 0) new_pct = 0;
            if (new_pct > 100) new_pct = 100;

            // Читаем свежую телеметрию
            if (!battery_low_state) {
                xSemaphoreTake(i2c_mutex, portMAX_DELAY);
                rda5807_get_telemetry(&cached_rssi, &cached_fm_true, &cached_stereo);
                xSemaphoreGive(i2c_mutex);
            }

            bool ui_needs_update = false;

            if (abs(new_pct - battery_percent) > 1 || (new_pct == 0 && battery_percent != 0) || (new_pct == 100 && battery_percent != 100)) {
                battery_percent = new_pct;
                ui_needs_update = true;
            }

            if (!battery_low_state && moving_avg_volts <= 3.0f) {
                battery_low_state = true;
                ESP_LOGW(TAG, "BATTERY LOW DETECTED at %.2fV", moving_avg_volts);
                xSemaphoreTake(i2c_mutex, portMAX_DELAY);
                rda5807_power_down(); 
                xSemaphoreGive(i2c_mutex);
                ui_needs_update = true;
            }

            if (battery_low_state && moving_avg_volts > 3.2f) { 
                battery_low_state = false;
                xSemaphoreTake(i2c_mutex, portMAX_DELAY);
                rda5807_set_frequency(current_frequency); 
                xSemaphoreGive(i2c_mutex);
                ui_needs_update = true;
            }

            // Обновляем UI только при реальных изменениях (RSSI или батарея)
            if (!battery_low_state && abs(cached_rssi - prev_rssi_display) > 2) {
                prev_rssi_display = cached_rssi;
                ui_needs_update = true;
            }

            if (ui_needs_update) {
                oled_update_ui(current_frequency, is_tuning, battery_percent, battery_low_state, cached_rssi, cached_fm_true, cached_stereo);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ================= ТОЧКА ВХОДА =================
void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    load_nvs_settings();
    wifi_init_softap();

    i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL) {
        ESP_LOGE(TAG, "Не удалось создать Mutex. Остановка.");
        return;
    }

    ESP_ERROR_CHECK(i2c_master_init());
    encoder_init();
    battery_init(); 
    oled_init();
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    rda5807_init(); 
    xSemaphoreGive(i2c_mutex);
    load_nvs_freq();

    start_webserver();

    xTaskCreatePinnedToCore(radio_task, "radio_task", 8192, NULL, 5, NULL, 1);
}
