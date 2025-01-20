#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sen66/sen66_i2c.h"
#include "sen66/sensirion_common.h"
#include "sen66/sensirion_i2c_hal.h"

#define I2C_MASTER_SCL_IO 22       // GPIO pour SCL
#define I2C_MASTER_SDA_IO 10       // GPIO pour SDA
#define I2C_MASTER_NUM 0           // I2C port number
#define I2C_MASTER_FREQ_HZ 100000  // Fréquence I2C
#define SEN66_I2C_ADDRESS 0x6b     // Adresse I2C du SEN66

static const char* TAG = "SEN66";

// Add global semaphore
static SemaphoreHandle_t i2c_mutex = NULL;

// Implémentation des fonctions HAL requises
int8_t sensirion_i2c_hal_write(uint8_t address, const uint8_t* data, uint8_t count) {
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire I2C mutex for write");
        return -1;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, count, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    xSemaphoreGive(i2c_mutex);
    return ret == ESP_OK ? 0 : -1;
}

int8_t sensirion_i2c_hal_read(uint8_t address, uint8_t* data, uint8_t count) {
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire I2C mutex for read");
        return -1;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
    if (count > 1) {
        i2c_master_read(cmd, data, count - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + count - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    xSemaphoreGive(i2c_mutex);
    return ret == ESP_OK ? 0 : -1;
}

static esp_err_t sen66_i2c_init(void) {
    ESP_LOGI(TAG, "Initializing I2C for SEN66...");
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C parameter configuration failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver installation failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "I2C initialization completed successfully");
    }

    if (i2c_mutex == NULL) {
        i2c_mutex = xSemaphoreCreateMutex();
        if (i2c_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create I2C mutex");
            return ESP_FAIL;
        }
    }

    return ret;
}

esp_err_t sen66_sensor_init(void) {
    sen66_i2c_init();
    vTaskDelay(pdMS_TO_TICKS(1200));

    ESP_LOGI(TAG, "Initializing SEN66 sensor...");
    int16_t error = NO_ERROR;
    sensirion_i2c_hal_init();
    sen66_init(SEN66_I2C_ADDR_6B);
    ESP_LOGI(TAG, "SEN66 initialization completed");

    // Add delay after initialization
    vTaskDelay(pdMS_TO_TICKS(100));
    error = sen66_device_reset();
    if (error != NO_ERROR) {
        ESP_LOGE(TAG, "error executing device_reset(): %i\n", error);
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(1200));

    // Start measurement
    ESP_LOGI(TAG, "Starting continuous measurement...");
    error = sen66_start_continuous_measurement();
    if (error) {
        ESP_LOGE(TAG, "Error starting measurement: %d", error);
        // Debug I2C communication
        uint8_t data = 0;
        esp_err_t i2c_error = sensirion_i2c_hal_read(SEN66_I2C_ADDRESS, &data, 1);
        ESP_LOGE(TAG, "I2C read test result: %d", i2c_error);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Continuous measurement started successfully");
    return ESP_OK;
}
