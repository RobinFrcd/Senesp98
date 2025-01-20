#ifndef MAIN_ESP_ZB_SEN66_H_
#define MAIN_ESP_ZB_SEN66_H_

#include "esp_zigbee_core.h"

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE false
#define ED_AGING_TIMEOUT ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE 3000 /* 3000 millisecond */
#define HA_ESP_SENSOR_ENDPOINT 10
#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK

/* Local sensor update interval (second) */
#define ESP_TEMP_SENSOR_UPDATE_INTERVAL (1)
#define ESP_TEMP_SENSOR_MIN_VALUE (-10)
#define ESP_TEMP_SENSOR_MAX_VALUE (80)

#define ZCL_BASIC_POWER_SOURCE 1

#define ESP_ZB_ZCL_CLUSTER_ID_PM1_MEASUREMENT 0xFC01
#define ESP_ZB_ZCL_ATTR_PM1_MEASUREMENT_MEASURED_VALUE_ID 0x0000

#define ESP_ZB_ZCL_CLUSTER_ID_PM4_MEASUREMENT 0xFC02
#define ESP_ZB_ZCL_ATTR_PM4_MEASUREMENT_MEASURED_VALUE_ID 0x0000

#define ESP_ZB_ZCL_CLUSTER_ID_PM10_MEASUREMENT 0xFC03
#define ESP_ZB_ZCL_ATTR_PM10_MEASUREMENT_MEASURED_VALUE_ID 0x0000

#define ESP_ZB_ZCL_CLUSTER_ID_NOX_MEASUREMENT 0xFC04
#define ESP_ZB_ZCL_ATTR_NOX_MEASUREMENT_MEASURED_VALUE_ID 0x0000

#define ESP_ZB_ZCL_CLUSTER_ID_VOC_MEASUREMENT 0xFC05
#define ESP_ZB_ZCL_ATTR_VOC_MEASUREMENT_MEASURED_VALUE_ID 0x0000

#define ESP_ZB_ZCL_MANUFACTURER_CODE_SENSIRION 0x1234

#define ESP_ZB_ZED_CONFIG()                               \
    {                                                     \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,             \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE, \
        .nwk_cfg.zed_cfg =                                \
            {                                             \
                .ed_timeout = ED_AGING_TIMEOUT,           \
                .keep_alive = ED_KEEP_ALIVE,              \
            },                                            \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()       \
    {                                       \
        .radio_mode = ZB_RADIO_MODE_NATIVE, \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                          \
    {                                                         \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE, \
    }

#endif  // MAIN_ESP_ZB_SEN66_H_
