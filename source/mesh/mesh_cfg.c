/******************************************************************************
* File Name:   mesh_cfg.c
*
* Description: This file contains btstack and mesh configuration.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2021-2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include "cy_retarget_io.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_sdp.h"
#include "wiced_memory.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_mesh_app.h"
#include "mesh_cfg.h"

/*******************************************************************************
* Macros
********************************************************************************/

/******************************************************************************
* Configurations
*******************************************************************************/
/* Bluetooth stack core configuration */

wiced_bt_cfg_ble_scan_settings_t wiced_bt_cfg_scan_settings =
{
    .scan_mode = BTM_BLE_SCAN_MODE_ACTIVE, /**< BLE scan mode ( BTM_BLE_SCAN_MODE_PASSIVE, BTM_BLE_SCAN_MODE_ACTIVE, or BTM_BLE_SCAN_MODE_NONE ) */

    /* Advertisement scan configuration */
    .high_duty_scan_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_INTERVAL, /**< High duty scan interval */
    .high_duty_scan_window = WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_WINDOW,   /**< High duty scan window */
    .high_duty_scan_duration = 5, /**< High duty scan duration in seconds ( 0 for infinite ) */

    .low_duty_scan_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_INTERVAL, /**< Low duty scan interval  */
    .low_duty_scan_window = WICED_BT_CFG_DEFAULT_LOW_DUTY_SCAN_WINDOW,    /**< Low duty scan window */
    .low_duty_scan_duration = 5,  /**< Low duty scan duration in seconds ( 0 for infinite ) */

    /* Connection scan configuration */
    .high_duty_conn_scan_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_CONN_SCAN_INTERVAL, /**< High duty cycle connection scan interval */
    .high_duty_conn_scan_window = WICED_BT_CFG_DEFAULT_HIGH_DUTY_CONN_SCAN_WINDOW,   /**< High duty cycle connection scan window */
    .high_duty_conn_duration = 30,      /**< High duty cycle connection duration in seconds ( 0 for infinite ) */

    .low_duty_conn_scan_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_CONN_SCAN_INTERVAL, /**< Low duty cycle connection scan interval */
    .low_duty_conn_scan_window = WICED_BT_CFG_DEFAULT_LOW_DUTY_CONN_SCAN_WINDOW,    /**< Low duty cycle connection scan window */
    .low_duty_conn_duration = 30,       /**< Low duty cycle connection duration in seconds ( 0 for infinite ) */

    /* Connection configuration */
    .conn_min_interval = WICED_BT_CFG_DEFAULT_CONN_MIN_INTERVAL,               /**< Minimum connection interval */
    .conn_max_interval = WICED_BT_CFG_DEFAULT_CONN_MAX_INTERVAL,               /**< Maximum connection interval */
    .conn_latency = WICED_BT_CFG_DEFAULT_CONN_LATENCY,                         /**< Connection latency */
    .conn_supervision_timeout = WICED_BT_CFG_DEFAULT_CONN_SUPERVISION_TIMEOUT, /**< Connection link supervision timeout */
};

const wiced_bt_cfg_ble_advert_settings_t wiced_bt_cfg_adv_settings =
{
    .channel_map = BTM_BLE_ADVERT_CHNL_37 | /**< Advertising channel map ( mask of BTM_BLE_ADVERT_CHNL_37, BTM_BLE_ADVERT_CHNL_38, BTM_BLE_ADVERT_CHNL_39 ) */
    BTM_BLE_ADVERT_CHNL_38 |
    BTM_BLE_ADVERT_CHNL_39,

    .high_duty_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MIN_INTERVAL, /**< High duty undirected connectable minimum advertising interval */
    .high_duty_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MAX_INTERVAL, /**< High duty undirected connectable maximum advertising interval */
    .high_duty_duration = 120,                                                  /**< High duty undirected connectable advertising duration in seconds ( 0 for infinite ) */

    .low_duty_min_interval = 1024, /**< Low duty undirected connectable minimum advertising interval */
    .low_duty_max_interval = 1024, /**< Low duty undirected connectable maximum advertising interval */
    .low_duty_duration = 60,       /**< Low duty undirected connectable advertising duration in seconds ( 0 for infinite ) */

    .high_duty_directed_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MIN_INTERVAL, /**< High duty directed connectable minimum advertising interval */
    .high_duty_directed_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MAX_INTERVAL, /**< High duty directed connectable maximum advertising interval */

    .low_duty_directed_min_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MIN_INTERVAL, /**< Low duty directed connectable minimum advertising interval */
    .low_duty_directed_max_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MAX_INTERVAL, /**< Low duty directed connectable maximum advertising interval */
    .low_duty_directed_duration = 30,                                                          /**< Low duty directed connectable advertising duration in seconds ( 0 for infinite ) */

    .high_duty_nonconn_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MIN_INTERVAL, /**< High duty non-connectable minimum advertising interval */
    .high_duty_nonconn_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MAX_INTERVAL, /**< High duty non-connectable maximum advertising interval */
    .high_duty_nonconn_duration = 30,                                                          /**< High duty non-connectable advertising duration in seconds ( 0 for infinite ) */

    .low_duty_nonconn_min_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MIN_INTERVAL, /**< Low duty non-connectable minimum advertising interval */
    .low_duty_nonconn_max_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MAX_INTERVAL, /**< Low duty non-connectable maximum advertising interval */
    .low_duty_nonconn_duration = 0                                                           /**< Low duty non-connectable advertising duration in seconds ( 0 for infinite ) */
};

wiced_bt_cfg_ble_t wiced_bt_cfg_ble = {
    .ble_max_simultaneous_links = 1,
    .ble_max_rx_pdu_size = 365,

    .p_ble_scan_cfg = &wiced_bt_cfg_scan_settings, /**< */
    .p_ble_advert_cfg = &wiced_bt_cfg_adv_settings,
    .appearance = APPEARANCE_GENERIC_TAG,  /**< GATT appearance (see gatt_appearance_e) */

    .host_addr_resolution_db_size = 5, /**< LE Address Resolution DB settings - effective only for pre 4.2 controller*/
    .rpa_refresh_timeout = WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_NEVER_CHANGE, /**< Interval of  random address refreshing - secs */
};

wiced_bt_cfg_gatt_t wiced_bt_cfg_gatt = {
    .max_db_service_modules = 0,   /**< Maximum number of service modules in the DB*/
    .max_eatt_bearers = 0,         /**< Maximum number of allowed gatt bearers */
};

wiced_bt_cfg_settings_t wiced_bt_cfg_settings =
{
    .device_name = (uint8_t *)MESH_DEVICE_NAME,     /**< Local device name ( NULL terminated ) */
    .p_ble_cfg = &wiced_bt_cfg_ble,
    .p_gatt_cfg = &wiced_bt_cfg_gatt,

};


/* mesh configuration */
wiced_bt_mesh_core_config_model_t   mesh_element1_models[] =
{
    WICED_BT_MESH_DEVICE,
    WICED_BT_MESH_MODEL_ONOFF_SERVER,
};

wiced_bt_mesh_core_config_model_t   mesh_element2_models[] =
{
    WICED_BT_MESH_MODEL_LEVEL_CLIENT
};

wiced_bt_mesh_core_config_element_t mesh_elements[] =
{
    {
        .location = MESH_ELEM_LOC_MAIN,                                 // location description as defined in the GATT Bluetooth Namespace Descriptors section of the Bluetooth SIG Assigned Numbers
        .default_transition_time = MESH_DEFAULT_TRANSITION_TIME_IN_MS,  // Default transition time for models of the element in milliseconds
        .onpowerup_state = WICED_BT_MESH_ON_POWER_UP_STATE_RESTORE,     // Default element behavior on power up
        .default_level = 0,                                             // Default value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_min = 1,                                                 // Minimum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_max = 0xffff,                                            // Maximum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .move_rollover = 0,                                             // If true when level gets to range_max during move operation, it switches to min, otherwise move stops.
        .properties_num = 0,                                            // Number of properties in the array models
        .properties = NULL,                                             // Array of properties in the element.
        .sensors_num = 0,                                               // Number of sensors in the sensor array
        .sensors = NULL,                                                // Array of sensors of that element
        .models_num = (sizeof(mesh_element1_models) / sizeof(wiced_bt_mesh_core_config_model_t)),    // Number of models in the array models
        .models = mesh_element1_models,                                 // Array of models located in that element. Model data is defined by structure wiced_bt_mesh_core_config_model_t
    },

    {
        .location = MESH_ELEM_LOC_MAIN,                                 // location description as defined in the GATT Bluetooth Namespace Descriptors section of the Bluetooth SIG Assigned Numbers
        .default_transition_time = MESH_DEFAULT_TRANSITION_TIME_IN_MS,  // Default transition time for models of the element in milliseconds
        .onpowerup_state = WICED_BT_MESH_ON_POWER_UP_STATE_RESTORE,     // Default element behavior on power up
        .default_level = 0,                                             // Default value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_min = 1,                                                 // Minimum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_max = 0xffff,                                            // Maximum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .move_rollover = 0,                                             // If true when level gets to range_max during move operation, it switches to min, otherwise move stops.
        .properties_num = 0,                                            // Number of properties in the array models
        .properties = NULL,                                             // Array of properties in the element.
        .sensors_num = 0,                                               // Number of sensors in the sensor array
        .sensors = NULL,                                                // Array of sensors of that element
        .models_num = (sizeof(mesh_element2_models) / sizeof(wiced_bt_mesh_core_config_model_t)),                              // Number of models in the array models
        .models = mesh_element2_models,                                 // Array of models located in that element. Model data is defined by structure wiced_bt_mesh_core_config_model_t
    },

};

wiced_bt_mesh_core_config_t  mesh_config =
{
    .company_id         = MESH_COMPANY_ID_CYPRESS,                  // Company identifier assigned by the Bluetooth SIG
    .product_id         = MESH_PID,                                 // Vendor-assigned product identifier
    .vendor_id          = MESH_VID,                                 // Vendor-assigned product version identifier
    .replay_cache_size  = MESH_CACHE_REPLAY_SIZE,                   // Number of replay protection entries, i.e. maximum number of mesh devices that can send application messages to this device.
#if defined(LOW_POWER_NODE) && (LOW_POWER_NODE == 1)
    .features           = WICED_BT_MESH_CORE_FEATURE_BIT_LOW_POWER, // A bit field indicating the device features. In Low Power mode no Relay, no Proxy and no Friend
    .friend_cfg         =                                           // Empty Configuration of the Friend Feature
    {
        .receive_window = 0,                                        // Receive Window value in milliseconds supported by the Friend node.
        .cache_buf_len  = 0,                                        // Length of the buffer for the cache
        .max_lpn_num    = 0                                         // Max number of Low Power Nodes with established friendship. Must be > 0 if Friend feature is supported.
    },
    .low_power          =                                           // Configuration of the Low Power Feature
    {
        .rssi_factor           = 2,                                 // contribution of the RSSI measured by the Friend node used in Friend Offer Delay calculations.
        .receive_window_factor = 2,                                 // contribution of the supported Receive Window used in Friend Offer Delay calculations.
        .min_cache_size_log    = 3,                                 // minimum number of messages that the Friend node can store in its Friend Cache.
        .receive_delay         = 100,                               // Receive delay in 1 ms units to be requested by the Low Power node.
        .poll_timeout          = 200                                // Poll timeout in 100ms units to be requested by the Low Power node.
    },
#else
    .features = WICED_BT_MESH_CORE_FEATURE_BIT_FRIEND | WICED_BT_MESH_CORE_FEATURE_BIT_RELAY | WICED_BT_MESH_CORE_FEATURE_BIT_GATT_PROXY_SERVER,   // Supports Friend, Relay and GATT Proxy
    .friend_cfg         =                                           // Configuration of the Friend Feature(Receive Window in Ms, messages cache)
    {
        .receive_window        = 20,
        .cache_buf_len         = 300,                               // Length of the buffer for the cache
        .max_lpn_num           = 4                                  // Max number of Low Power Nodes with established friendship. Must be > 0 if Friend feature is supported.
    },
    .low_power          =                                           // Configuration of the Low Power Feature
    {
        .rssi_factor           = 0,                                 // contribution of the RSSI measured by the Friend node used in Friend Offer Delay calculations.
        .receive_window_factor = 0,                                 // contribution of the supported Receive Window used in Friend Offer Delay calculations.
        .min_cache_size_log    = 0,                                 // minimum number of messages that the Friend node can store in its Friend Cache.
        .receive_delay         = 0,                                 // Receive delay in 1 ms units to be requested by the Low Power node.
        .poll_timeout          = 0                                  // Poll timeout in 100ms units to be requested by the Low Power node.
    },
#endif
    .gatt_client_only          = WICED_FALSE,                       // Can connect to mesh over GATT or ADV
    .elements_num  = (uint8_t)(sizeof(mesh_elements) / sizeof(mesh_elements[0])),   // number of elements on this device
    .elements      = mesh_elements                                  // Array of elements for this device
};

/* [] END OF FILE */
