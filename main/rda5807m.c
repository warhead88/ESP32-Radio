#include "rda5807m.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"

#define I2C_MASTER_PORT         I2C_NUM_0
#define RDA5807M_I2C_ADDR_SEQ   0x10
#define I2C_MASTER_TIMEOUT_MS   100

static const char *TAG = "RDA5807M";

static bool is_rda5807_powered = false;

void rda5807_init(void) {
    uint8_t boot_cfg[] = {
        0xC0, 0x01, // REG 02H: ENABLE=1, RDS_EN=0
        0x00, 0x00, // REG 03H: Band defaults
        0x06, 0x00, // REG 04H: SOFTMUTE_EN=1 (Bit 10,9)
        0x90, 0x8F, // REG 05H: Volume 15
    };

    i2c_master_write_to_device(I2C_MASTER_PORT, RDA5807M_I2C_ADDR_SEQ, boot_cfg, sizeof(boot_cfg), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    
    is_rda5807_powered = true;
    ESP_LOGI(TAG, "RDA5807M Initialized without RDS");
}

void rda5807_set_frequency(float freq) {
    uint16_t channel = (uint16_t)((freq - 87.0f) * 10.0f + 0.5f);
    uint8_t buffer[8];

    buffer[0] = 0xC0; // REG 02 High
    buffer[1] = 0x01; // REG 02 Low (RDS_EN=0, ENABLE=1)
    buffer[2] = (channel >> 2) & 0xFF; // REG 03 High
    buffer[3] = ((channel & 0x03) << 6) | (1 << 4); // REG 03 Low
    buffer[4] = 0x06; // REG 04 High: Softmute on 
    buffer[5] = 0x00; // REG 04 Low 
    buffer[6] = 0x88; // REG 05 High
    buffer[7] = 0x8F; // REG 05 Low

    esp_err_t err = i2c_master_write_to_device(I2C_MASTER_PORT, RDA5807M_I2C_ADDR_SEQ, buffer, 8, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "FREQ tuned to %.1f MHz", (double)freq);
        is_rda5807_powered = true;
    }
}

void rda5807_power_down() {
    uint8_t buffer[2] = {0x00, 0x00}; // REG 02H: DISABLE
    i2c_master_write_to_device(I2C_MASTER_PORT, RDA5807M_I2C_ADDR_SEQ, buffer, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    
    is_rda5807_powered = false;
    ESP_LOGI(TAG, "Power Down");
}

int rda5807_get_rssi(void) {
    if (!is_rda5807_powered) return 0;
    uint8_t buffer[4] = {0};
    esp_err_t err = i2c_master_read_from_device(I2C_MASTER_PORT, RDA5807M_I2C_ADDR_SEQ, buffer, 4, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    if (err == ESP_OK) {
        return buffer[2] >> 1; 
    }
    return 0;
}

bool rda5807_get_stereo(void) {
    if (!is_rda5807_powered) return false;
    uint8_t buffer[4] = {0};
    esp_err_t err = i2c_master_read_from_device(I2C_MASTER_PORT, RDA5807M_I2C_ADDR_SEQ, buffer, 4, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    if (err == ESP_OK) {
        // Bit 10 of Reg 0x0A => Bit 2 of buffer[0]
        return (buffer[0] & 0x04) != 0; 
    }
    return false;
}

void rda5807_get_telemetry(int *rssi, bool *fm_true, bool *stereo) {
    if (!is_rda5807_powered) {
        if (rssi) *rssi = 0;
        if (fm_true) *fm_true = false;
        if (stereo) *stereo = false;
        return;
    }
    uint8_t buffer[4] = {0};
    esp_err_t err = i2c_master_read_from_device(I2C_MASTER_PORT, RDA5807M_I2C_ADDR_SEQ, buffer, 4, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    if (err == ESP_OK) {
        if (rssi) *rssi = buffer[2] >> 1;
        if (fm_true) *fm_true = (buffer[2] & 0x01) != 0;
        if (stereo) *stereo = (buffer[0] & 0x04) != 0;
    }
}
