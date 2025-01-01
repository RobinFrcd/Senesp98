#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sen66/sen66_i2c.h"
#include "sen66/sensirion_common.h"
#include "sen66/sensirion_i2c_hal.h"
#include "freertos/semphr.h"

#define I2C_MASTER_SCL_IO 22        // GPIO pour SCL
#define I2C_MASTER_SDA_IO 10        // GPIO pour SDA
#define I2C_MASTER_NUM 0            // I2C port number
#define I2C_MASTER_FREQ_HZ 100000   // Fréquence I2C 
#define SEN66_I2C_ADDRESS 0x6b      // Adresse I2C du SEN66

static const char *TAG = "SEN66";

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

    // uint8_t serial_number[32] = {0};
    // error = sen66_get_serial_number(serial_number, 32);
    // if (error != NO_ERROR) {
    //     ESP_LOGE(TAG, "error executing get_serial_number(): %i\n", error);
    //     return ESP_FAIL;
    // }

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

void read_sen66_values(void) {
    ESP_LOGI(TAG, "Starting SEN66 sensor reading task...");
    
    uint16_t mass_concentration_pm1p0, mass_concentration_pm2p5;
    uint16_t mass_concentration_pm4p0, mass_concentration_pm10p0;
    int16_t ambient_humidity, ambient_temperature, voc_index;
    int16_t nox_index;
    uint16_t co2;
    int16_t error;

    // Initialize sensor using the new dedicated function
    if (sen66_sensor_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SEN66 sensor");
        return;
    }

    while (1) {
        bool data_ready = false;
        uint8_t padding;

        // Vérifier si les données sont prêtes
        error = sen66_get_data_ready(&padding, &data_ready);
        if (error) {
            ESP_LOGE(TAG, "Error checking data ready: %d", error);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (!data_ready) {
            ESP_LOGD(TAG, "Waiting for data to be ready...");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Lire les valeurs
        error = sen66_read_measured_values_as_integers(
            &mass_concentration_pm1p0, &mass_concentration_pm2p5,
            &mass_concentration_pm4p0, &mass_concentration_pm10p0,
            &ambient_humidity, &ambient_temperature, &voc_index,
            &nox_index, &co2);

        if (error) {
            ESP_LOGE(TAG, "Error reading values: %d", error);
        } else {
            // Afficher les valeurs
            ESP_LOGI(TAG, "PM1.0: %.1f µg/m³", mass_concentration_pm1p0 / 10.0f);
            ESP_LOGI(TAG, "PM2.5: %.1f µg/m³", mass_concentration_pm2p5 / 10.0f);
            ESP_LOGI(TAG, "PM4.0: %.1f µg/m³", mass_concentration_pm4p0 / 10.0f);
            ESP_LOGI(TAG, "PM10: %.1f µg/m³", mass_concentration_pm10p0 / 10.0f);
            ESP_LOGI(TAG, "Temperature: %.1f °C", ambient_temperature / 200.0f);
            ESP_LOGI(TAG, "Humidity: %.1f %%", ambient_humidity / 100.0f);
            ESP_LOGI(TAG, "VOC Index: %.1f", voc_index / 10.0f);
            ESP_LOGI(TAG, "NOx Index: %.1f", nox_index / 10.0f);
            ESP_LOGI(TAG, "CO2: %d ppm", co2);
        }

        // Attendre 1 seconde avant la prochaine lecture
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// Ajoutez cette fonction globale pour récupérer la température
int16_t get_sen66_temperature(void) {
    int16_t ambient_temperature;
    int16_t error = sen66_read_measured_values_as_integers(
        NULL, NULL, NULL, NULL,  // Ignorer les autres valeurs PM
        NULL, &ambient_temperature, NULL, 
        NULL, NULL  // Ignorer VOC, NOx, CO2
    );
    
    if (error) {
        ESP_LOGE(TAG, "Error reading temperature: %d", error);
        return 0;  // Valeur par défaut en cas d'erreur
    }
    
    return ambient_temperature;
}

// void app_main(void) {
//     ESP_LOGI(TAG, "Starting sensor reading task...");
//     read_sen66_values();
// } 