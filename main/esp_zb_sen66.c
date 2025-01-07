/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee HA_temperature_sensor Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "sen66_esp32.h"
#include "esp_zb_sen66.h"
#include "string.h"

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile sensor (End Device) source code.
#endif

#define DEFINE_PSTRING(var, str)   \
    const struct                   \
    {                              \
        unsigned char len;         \
        char content[sizeof(str)]; \
    }(var) = {sizeof(str) - 1, (str)}

static const char *TAG = "ESP_ZB_SEN66_SENSOR";

#define ESP_ZB_ZCL_CLUSTER_ID_PM10_MEASUREMENT 0x042dU
#define ESP_ZB_ZCL_ATTR_PM10_MEASUREMENT_MEASURED_VALUE_ID 0x0000

#define ESP_ZB_ZCL_CLUSTER_ID_VOC_MEASUREMENT 0x042EU
#define ESP_ZB_ZCL_ATTR_VOC_MEASUREMENT_MEASURED_VALUE_ID 0x0000

void reportAttribute(uint16_t clusterID, uint16_t attributeID, void *value, uint8_t value_length)
{
    esp_zb_zcl_report_attr_cmd_t cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = 0x0000,
            .dst_endpoint = HA_ESP_SENSOR_ENDPOINT,
            .src_endpoint = HA_ESP_SENSOR_ENDPOINT,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = clusterID,
        .attributeID = attributeID,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
    };
    esp_zb_zcl_attr_t *value_r = esp_zb_zcl_get_attribute(HA_ESP_SENSOR_ENDPOINT, clusterID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attributeID);
    memcpy(value_r->data_p, value, value_length);
    esp_zb_zcl_report_attr_cmd_req(&cmd);
}

void reportValue(uint16_t cluserID, uint16_t valueID, void *value)
{
    //esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(
        HA_ESP_SENSOR_ENDPOINT, 
        cluserID, 
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, 
        valueID, 
        value, 
        false
    );
    // esp_zb_lock_release();
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, ,
                        TAG, "Failed to start Zigbee bdb commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p     = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Deferred driver initialization %s", sen66_sensor_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in%s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : " non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
            }
        } else {
            ESP_LOGW(TAG, "%s failed with status: %s, retrying", esp_zb_zdo_signal_to_string(sig_type),
                     esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static esp_zb_cluster_list_t *custom_sensor_clusters_create()
{
    esp_zb_basic_cluster_cfg_t basic_cfg;
    esp_zb_identify_cluster_cfg_t identify_cfg;
    
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
    DEFINE_PSTRING(ManufacturerName, "Sensirion");
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)&ManufacturerName));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&identify_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
    
    esp_zb_temperature_meas_cluster_cfg_t temperature_meas_cfg = {
        .measured_value = 0xFFFF,
        .min_value = -1000,
        .max_value = 6000,
    };
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(
        cluster_list, 
        esp_zb_temperature_meas_cluster_create(&temperature_meas_cfg), 
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
    ));
    ESP_LOGI(TAG, "Temperature measurement cluster added");

    esp_zb_humidity_meas_cluster_cfg_t humidity_meas_cfg = {
        .measured_value = 0xFFFF,
        .min_value = 0,
        .max_value = 65535,
    };
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_humidity_meas_cluster(
        cluster_list, 
        esp_zb_humidity_meas_cluster_create(&humidity_meas_cfg), 
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
    ));
    ESP_LOGI(TAG, "Humidity measurement cluster added");

    esp_zb_pm2_5_measurement_cluster_cfg_t pm2_5_meas_cfg = {
        .measured_value = 0xFFFF,
        .min_measured_value = 0,
        .max_measured_value = 65535,
    };
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_pm2_5_measurement_cluster(
        cluster_list, 
        esp_zb_pm2_5_measurement_cluster_create(&pm2_5_meas_cfg), 
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
    ));
    ESP_LOGI(TAG, "PM2.5 measurement cluster added");
    
    esp_zb_carbon_dioxide_measurement_cluster_cfg_t co2_meas_cfg = {
        .measured_value = 0xFFFF,
        .min_measured_value = 0,
        .max_measured_value = 10000,
    };
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_carbon_dioxide_measurement_cluster(
        cluster_list,
        esp_zb_carbon_dioxide_measurement_cluster_create(&co2_meas_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
    ));
    ESP_LOGI(TAG, "Carbon dioxide measurement cluster added");

    return cluster_list;
}

static esp_zb_ep_list_t *custom_sensor_ep_create(uint8_t endpoint_id)
{
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = endpoint_id,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, custom_sensor_clusters_create(), endpoint_config);
    return ep_list;
}

// Helper function to configure reporting for a single cluster
static void configure_cluster_reporting(uint8_t endpoint, uint16_t cluster_id, uint16_t attr_id) {
    esp_zb_zcl_reporting_info_t reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = endpoint,
        .cluster_id = cluster_id,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,
        .u.send_info.max_interval = 0,
        .u.send_info.def_min_interval = 1,
        .u.send_info.def_max_interval = 0,
        .u.send_info.delta.u16 = 10,
        .attr_id = attr_id,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    esp_zb_zcl_update_reporting_info(&reporting_info);
}

static void init_zigbee(void)
{
    /* Initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_ep_list_t *esp_zb_sensor_ep = custom_sensor_ep_create(HA_ESP_SENSOR_ENDPOINT);

    /* Register the device */
    esp_zb_device_register(esp_zb_sensor_ep);

    /* Configure reporting for all clusters */
    configure_cluster_reporting(
        HA_ESP_SENSOR_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID
    );
    
    configure_cluster_reporting(
        HA_ESP_SENSOR_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
        ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID
    );

    configure_cluster_reporting(
        HA_ESP_SENSOR_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_PM2_5_MEASUREMENT,
        ESP_ZB_ZCL_ATTR_PM2_5_MEASUREMENT_MEASURED_VALUE_ID
    );

    configure_cluster_reporting(
        HA_ESP_SENSOR_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT,
        ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID
    );

    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
}

static void esp_zb_task(void *pvParameters)
{
    esp_zb_stack_main_loop();
}

static void read_temperature_task(void *pvParameters)
{
    uint16_t mass_concentration_pm1p0, mass_concentration_pm2p5;
    uint16_t mass_concentration_pm4p0, mass_concentration_pm10p0;
    int16_t ambient_humidity, ambient_temperature, voc_index;
    int16_t nox_index;
    uint16_t co2;
    int16_t error;

    if (sen66_sensor_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SEN66 sensor");
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

        error = sen66_read_measured_values_as_integers(
            &mass_concentration_pm1p0, &mass_concentration_pm2p5,
            &mass_concentration_pm4p0, &mass_concentration_pm10p0,
            &ambient_humidity, &ambient_temperature, &voc_index,
            &nox_index, &co2);

        esp_zb_lock_acquire(portMAX_DELAY);
        // Update Zigbee attribute with new temperature
        int16_t temp_value = ambient_temperature / 2;
        ESP_LOGI(TAG, "Temperature: %.1f °C | Z2M Value: %d", ambient_temperature / 200.0f, temp_value);
        reportValue(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &temp_value);
        
        // Update Zigbee attribute with new temperature
        ESP_LOGI(TAG, "Humidity: %.2f %%", ambient_humidity / 100.0f);
        reportValue(ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &ambient_humidity);
        
        float_t pm25_value = mass_concentration_pm2p5 / 10.0f;
        ESP_LOGI(TAG, "PM2.5: %.1f µg/m³", pm25_value);
        reportValue(ESP_ZB_ZCL_CLUSTER_ID_PM2_5_MEASUREMENT, ESP_ZB_ZCL_ATTR_PM2_5_MEASUREMENT_MEASURED_VALUE_ID, &pm25_value);
                          
        float_t co2_value = ((co2 > 10000) ? 10000.0f : (float_t) co2) / 1000000.0f;
        ESP_LOGI(TAG, "CO2: %.1f ppm", co2_value * 1000000.0f);
        reportValue(ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT, ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID, &co2_value);

        esp_zb_lock_release();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    init_zigbee();
    xTaskCreate(esp_zb_task, "Zigbee_main", 16384, NULL, 5, NULL);
    xTaskCreate(read_temperature_task, "temp_reader", 16384, NULL, 4, NULL);
}
