#include "esp_zb_sen66.h"

#include <math.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "nvs_flash.h"
#include "sen66_esp32.h"
#include "string.h"

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile sensor (End Device) source code.
#endif

#define DEFINE_PSTRING(var, str)   \
    const struct {                 \
        unsigned char len;         \
        char content[sizeof(str)]; \
    }(var) = {sizeof(str) - 1, (str)}

static const char *TAG = "ESP_ZB_SEN66_SENSOR";

void reportAttribute(uint16_t clusterID, uint16_t attributeID, void *value, uint8_t value_length) {
    esp_zb_zcl_report_attr_cmd_t cmd = {
        .zcl_basic_cmd =
            {
                .dst_addr_u.addr_short = 0x0000,
                .dst_endpoint = HA_ESP_SENSOR_ENDPOINT,
                .src_endpoint = HA_ESP_SENSOR_ENDPOINT,
            },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = clusterID,
        .attributeID = attributeID,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
    };
    esp_zb_zcl_attr_t *value_r =
        esp_zb_zcl_get_attribute(HA_ESP_SENSOR_ENDPOINT, clusterID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attributeID);
    memcpy(value_r->data_p, value, value_length);
    esp_zb_zcl_report_attr_cmd_req(&cmd);
}

void reportValue(uint16_t clusterID, uint16_t valueID, void *value, bool round) {
    float_t rounded_value;

    if (round) {
        rounded_value = *(float_t *)value;
        rounded_value = roundf(rounded_value * 100) / 100;  // Round to 2 decimal places
    }

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(HA_ESP_SENSOR_ENDPOINT, clusterID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, valueID,
                                 round ? &rounded_value : value, false);
    esp_zb_lock_release();
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
    esp_err_t err = esp_zb_bdb_start_top_level_commissioning(mode_mask);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to start Zigbee bdb commissioning: %s", esp_err_to_name(err));
        // Retry after a longer delay
        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, mode_mask, 5000);
    }
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    static bool commissioning_in_progress = false;

    switch (sig_type) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            ESP_LOGI(TAG, "Initialize Zigbee stack");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
            break;

        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (err_status == ESP_OK) {
                ESP_LOGI(TAG, "Device started up in%s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : " non");
                if (!commissioning_in_progress) {
                    commissioning_in_progress = true;
                    ESP_LOGI(TAG, "Start network steering");
                    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                }
            } else {
                ESP_LOGW(TAG, "Failed to initialize Zigbee stack, rebooting");
                esp_restart();
            }
            break;

        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (err_status == ESP_OK) {
                commissioning_in_progress = false;
                esp_zb_ieee_addr_t extended_pan_id;
                esp_zb_get_extended_pan_id(extended_pan_id);
                ESP_LOGI(TAG, "Network steering successful");
            } else {
                if (!commissioning_in_progress) {
                    commissioning_in_progress = true;
                    ESP_LOGI(TAG, "Network steering failed, retrying in 1 second");
                    esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                           ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
                }
            }
            break;

        default:
            ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                     esp_err_to_name(err_status));
            break;
    }
}

static esp_zb_cluster_list_t *custom_sensor_clusters_create() {
    esp_zb_basic_cluster_cfg_t basic_cfg;
    esp_zb_identify_cluster_cfg_t identify_cfg;
    basic_cfg.power_source = ZCL_BASIC_POWER_SOURCE;

    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
    DEFINE_PSTRING(ManufacturerName, "Sensirion");
    DEFINE_PSTRING(ModelName, "SEN66");

    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
                                                  (void *)&ManufacturerName));
    ESP_ERROR_CHECK(
        esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)&ModelName));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(
        cluster_list, esp_zb_identify_cluster_create(&identify_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(
        cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));

    esp_zb_temperature_meas_cluster_cfg_t temperature_meas_cfg = {
        // .measured_value = 0,
        .min_value = -1000,
        .max_value = 6000,
    };
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(
        cluster_list, esp_zb_temperature_meas_cluster_create(&temperature_meas_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_LOGI(TAG, "Temperature measurement cluster added");

    esp_zb_humidity_meas_cluster_cfg_t humidity_meas_cfg = {
        // .measured_value = 0,
        .min_value = 0,
        .max_value = 65535,
    };
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_humidity_meas_cluster(
        cluster_list, esp_zb_humidity_meas_cluster_create(&humidity_meas_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_LOGI(TAG, "Humidity measurement cluster added");

    esp_zb_pm2_5_measurement_cluster_cfg_t pm2_5_meas_cfg = {
        // .measured_value = 0,
        .min_measured_value = 0,
        .max_measured_value = SEN66_PM2_5_MAX_VALUE,
    };
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_pm2_5_measurement_cluster(
        cluster_list, esp_zb_pm2_5_measurement_cluster_create(&pm2_5_meas_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_LOGI(TAG, "PM2.5 measurement cluster added");

    esp_zb_carbon_dioxide_measurement_cluster_cfg_t co2_meas_cfg = {
        // .measured_value = 0,
        .min_measured_value = 0,
        .max_measured_value = SEN66_CO2_MAX_VALUE,
    };
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_carbon_dioxide_measurement_cluster(
        cluster_list, esp_zb_carbon_dioxide_measurement_cluster_create(&co2_meas_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_LOGI(TAG, "Carbon dioxide measurement cluster added");

    // Custon Clusters
    float_t zero_value = 0.0;

    esp_zb_attribute_list_t *pm1_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_PM1_MEASUREMENT);
    esp_zb_custom_cluster_add_custom_attr(
        pm1_cluster, ESP_ZB_ZCL_ATTR_PM1_MEASUREMENT_MEASURED_VALUE_ID, ESP_ZB_ZCL_ATTR_TYPE_SINGLE,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &zero_value);
    esp_zb_cluster_list_add_custom_cluster(cluster_list, pm1_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    ESP_LOGI(TAG, "PM1 measurement cluster added (custom)");

    esp_zb_attribute_list_t *pm4_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_PM4_MEASUREMENT);
    esp_zb_custom_cluster_add_custom_attr(
        pm4_cluster, ESP_ZB_ZCL_ATTR_PM4_MEASUREMENT_MEASURED_VALUE_ID, ESP_ZB_ZCL_ATTR_TYPE_SINGLE,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &zero_value);
    esp_zb_cluster_list_add_custom_cluster(cluster_list, pm4_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    ESP_LOGI(TAG, "PM4 measurement cluster added (custom)");

    esp_zb_attribute_list_t *pm10_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_PM10_MEASUREMENT);
    esp_zb_custom_cluster_add_custom_attr(
        pm10_cluster, ESP_ZB_ZCL_ATTR_PM10_MEASUREMENT_MEASURED_VALUE_ID, ESP_ZB_ZCL_ATTR_TYPE_SINGLE,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &zero_value);
    esp_zb_cluster_list_add_custom_cluster(cluster_list, pm10_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    ESP_LOGI(TAG, "PM10 measurement cluster added (custom)");

    esp_zb_attribute_list_t *voc_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_VOC_MEASUREMENT);
    esp_zb_custom_cluster_add_custom_attr(
        voc_cluster, ESP_ZB_ZCL_ATTR_VOC_MEASUREMENT_MEASURED_VALUE_ID, ESP_ZB_ZCL_ATTR_TYPE_SINGLE,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &zero_value);
    esp_zb_cluster_list_add_custom_cluster(cluster_list, voc_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    ESP_LOGI(TAG, "VOC measurement cluster added (custom)");

    esp_zb_attribute_list_t *nox_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_NOX_MEASUREMENT);
    esp_zb_custom_cluster_add_custom_attr(
        nox_cluster, ESP_ZB_ZCL_ATTR_NOX_MEASUREMENT_MEASURED_VALUE_ID, ESP_ZB_ZCL_ATTR_TYPE_SINGLE,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &zero_value);
    esp_zb_cluster_list_add_custom_cluster(cluster_list, nox_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    ESP_LOGI(TAG, "NOx measurement cluster added (custom)");

    return cluster_list;
}

static esp_zb_ep_list_t *custom_sensor_ep_create(uint8_t endpoint_id) {
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {.endpoint = endpoint_id,
                                                .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
                                                .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
                                                .app_device_version = 0};
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

static void init_zigbee(void) {
    /* Initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_ep_list_t *esp_zb_sensor_ep = custom_sensor_ep_create(HA_ESP_SENSOR_ENDPOINT);

    esp_zb_device_register(esp_zb_sensor_ep);

    /* Configure reporting for all clusters */
    configure_cluster_reporting(HA_ESP_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                                ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID);

    configure_cluster_reporting(HA_ESP_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
                                ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID);

    configure_cluster_reporting(HA_ESP_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_PM2_5_MEASUREMENT,
                                ESP_ZB_ZCL_ATTR_PM2_5_MEASUREMENT_MEASURED_VALUE_ID);

    configure_cluster_reporting(HA_ESP_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT,
                                ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID);

    configure_cluster_reporting(HA_ESP_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_PM1_MEASUREMENT,
                                ESP_ZB_ZCL_ATTR_PM1_MEASUREMENT_MEASURED_VALUE_ID);

    configure_cluster_reporting(HA_ESP_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_PM4_MEASUREMENT,
                                ESP_ZB_ZCL_ATTR_PM4_MEASUREMENT_MEASURED_VALUE_ID);

    configure_cluster_reporting(HA_ESP_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_PM10_MEASUREMENT,
                                ESP_ZB_ZCL_ATTR_PM10_MEASUREMENT_MEASURED_VALUE_ID);

    configure_cluster_reporting(HA_ESP_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_VOC_MEASUREMENT,
                                ESP_ZB_ZCL_ATTR_VOC_MEASUREMENT_MEASURED_VALUE_ID);

    configure_cluster_reporting(HA_ESP_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_NOX_MEASUREMENT,
                                ESP_ZB_ZCL_ATTR_NOX_MEASUREMENT_MEASURED_VALUE_ID);

    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
}

static void esp_zb_task(void *pvParameters) { esp_zb_stack_main_loop(); }

static void read_temperature_task(void *pvParameters) {
    uint16_t mass_concentration_pm1p0, mass_concentration_pm2p5;
    uint16_t mass_concentration_pm4p0, mass_concentration_pm10p0;
    int16_t ambient_humidity, ambient_temperature, voc_index;
    int16_t nox_index;
    uint16_t co2_ppm;
    int16_t error;

    if (sen66_sensor_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SEN66 sensor");
    }

    while (1) {
        bool data_ready = false;
        uint8_t padding;

        // Check if data is ready
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
            &mass_concentration_pm1p0, &mass_concentration_pm2p5, &mass_concentration_pm4p0, &mass_concentration_pm10p0,
            &ambient_humidity, &ambient_temperature, &voc_index, &nox_index, &co2_ppm);

        // Temperature check
        float_t ambient_temperature_celsius = (float_t)ambient_temperature / 200.0f;
        if (ambient_temperature_celsius > SEN66_TEMP_MAX_VALUE) {
            ESP_LOGI(TAG, "Temperature: null (value too high)");
        } else {
            ESP_LOGI(TAG, "Temperature: %.1f °C", ambient_temperature_celsius);
            int16_t temp_zigbee = ambient_temperature_celsius * 100;
            reportValue(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &temp_zigbee,
                        false);
        }

        // Humidity check
        float_t ambient_humidity_percentage = (float_t)ambient_humidity / 100.0f;
        if (ambient_humidity_percentage > SEN66_HUMIDITY_MAX_VALUE) {
            ESP_LOGI(TAG, "Humidity: null (value too high)");
        } else {
            ESP_LOGI(TAG, "Humidity: %.2f %%", ambient_humidity_percentage);
            reportValue(ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
                        ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &ambient_humidity, false);
        }

        // PM2.5 check
        float_t pm2_5_ug_m3 = (float_t)mass_concentration_pm2p5 / 10.0f;
        if (pm2_5_ug_m3 > SEN66_PM2_5_MAX_VALUE) {
            ESP_LOGI(TAG, "PM2.5: null (value too high)");
        } else {
            ESP_LOGI(TAG, "PM2.5: %.1f µg/m³", pm2_5_ug_m3);
            reportValue(ESP_ZB_ZCL_CLUSTER_ID_PM2_5_MEASUREMENT, ESP_ZB_ZCL_ATTR_PM2_5_MEASUREMENT_MEASURED_VALUE_ID,
                        &pm2_5_ug_m3, false);
        }

        // CO2 check
        if (co2_ppm > SEN66_CO2_MAX_VALUE) {
            ESP_LOGI(TAG, "CO2: null (value too high)");
        } else {
            float_t co2_zigbee = (float_t)co2_ppm / 1000000.0f;
            ESP_LOGI(TAG, "CO2: %d ppm", co2_ppm);
            reportValue(ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT,
                        ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID, &co2_zigbee, false);
        }

        // PM1 check
        float_t pm1_ug_m3 = (float_t)mass_concentration_pm1p0 / 10.0f;
        if (pm1_ug_m3 > SEN66_PM1_MAX_VALUE) {
            ESP_LOGI(TAG, "PM1: null (value too high)");
        } else {
            ESP_LOGI(TAG, "PM1: %.1f µg/m³", pm1_ug_m3);
            reportValue(ESP_ZB_ZCL_CLUSTER_ID_PM1_MEASUREMENT, ESP_ZB_ZCL_ATTR_PM1_MEASUREMENT_MEASURED_VALUE_ID,
                        &pm1_ug_m3, true);
        }

        // PM4 check
        float_t pm4_ug_m3 = (float_t)mass_concentration_pm4p0 / 10.0f;
        if (pm4_ug_m3 > SEN66_PM4_MAX_VALUE) {
            ESP_LOGI(TAG, "PM4: null (value too high)");
        } else {
            ESP_LOGI(TAG, "PM4: %.1f µg/m³", pm4_ug_m3);
            reportValue(ESP_ZB_ZCL_CLUSTER_ID_PM4_MEASUREMENT, ESP_ZB_ZCL_ATTR_PM4_MEASUREMENT_MEASURED_VALUE_ID,
                        &pm4_ug_m3, true);
        }

        // PM10 check
        float_t pm10_ug_m3 = (float_t)mass_concentration_pm10p0 / 10.0f;
        if (pm10_ug_m3 > SEN66_PM10_MAX_VALUE) {
            ESP_LOGI(TAG, "PM10: null (value too high)");
        } else {
            ESP_LOGI(TAG, "PM10: %.1f µg/m³", pm10_ug_m3);
            reportValue(ESP_ZB_ZCL_CLUSTER_ID_PM10_MEASUREMENT, ESP_ZB_ZCL_ATTR_PM10_MEASUREMENT_MEASURED_VALUE_ID,
                        &pm10_ug_m3, true);
        }

        // VOC check
        float_t voc_index_value = (float_t)voc_index / 10.0f;
        if (voc_index_value > SEN66_VOC_MAX_VALUE) {
            ESP_LOGI(TAG, "VOC: null (value too high)");
        } else {
            ESP_LOGI(TAG, "VOC index: %.1f", voc_index_value);
            reportValue(ESP_ZB_ZCL_CLUSTER_ID_VOC_MEASUREMENT, ESP_ZB_ZCL_ATTR_VOC_MEASUREMENT_MEASURED_VALUE_ID,
                        &voc_index_value, true);
        }

        // NOx check
        float_t nox_index_value = (float_t)nox_index / 10.0f;
        if (nox_index_value > SEN66_NOX_MAX_VALUE) {
            ESP_LOGI(TAG, "NOx: null (value too high)");
        } else {
            ESP_LOGI(TAG, "NOx index: %.1f", nox_index_value);
            reportValue(ESP_ZB_ZCL_CLUSTER_ID_NOX_MEASUREMENT, ESP_ZB_ZCL_ATTR_NOX_MEASUREMENT_MEASURED_VALUE_ID,
                        &nox_index_value, true);
        }

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void app_main(void) {
    // Completely erase NVS memory and reinitialize it
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    // Wait for configuration to be applied
    vTaskDelay(pdMS_TO_TICKS(100));

    // Initialize Zigbee with delay
    init_zigbee();
    vTaskDelay(pdMS_TO_TICKS(500));

    // Create tasks with different priorities
    xTaskCreate(esp_zb_task, "Zigbee_main", 16384, NULL, 5, NULL);
    vTaskDelay(pdMS_TO_TICKS(1000));  // Wait for Zigbee task to start
    xTaskCreate(read_temperature_task, "temp_reader", 16384, NULL, 4, NULL);
}
