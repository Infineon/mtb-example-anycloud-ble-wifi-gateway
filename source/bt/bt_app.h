/******************************************************************************
* File Name:   bt_app.h
*
* Description: This file is the public interface of bt_app.c
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2021, Cypress Semiconductor Corporation (an Infineon company) or
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
 * Include guard
 ******************************************************************************/
#ifndef BT_APP_H_
#define BT_APP_H_

/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include "stdint.h"


/******************************************************************************
* Macros
*******************************************************************************/
#define BT_APP_TRACE_ENABLE             0u  /*enable or disable the traces*/

// Definitions for handles used in the GATT database
enum
{
    MESH_HANDLE_GATT_SERVICE = 0x01,

    MESH_HANDLE_GAP_SERVICE = 0x14,

        MESH_HANDLE_GAP_SERVICE_CHARACTERISTIC_DEV_NAME,
        MESH_HANDLE_GAP_SERVICE_CHARACTERISTIC_DEV_NAME_VAL,

        MESH_HANDLE_GAP_SERVICE_CHARACTERISTIC_APPEARANCE,
        MESH_HANDLE_GAP_SERVICE_CHARACTERISTIC_APPEARANCE_VAL,

    HANDLE_MESH_SERVICE_PROVISIONING = 0x28,
        HANDLE_CHAR_MESH_PROVISIONING_DATA_IN,
        HANDLE_CHAR_MESH_PROVISIONING_DATA_IN_VALUE,
        HANDLE_CHAR_MESH_PROVISIONING_DATA_OUT,
        HANDLE_CHAR_MESH_PROVISIONING_DATA_OUT_VALUE,
        HANDLE_DESCR_MESH_PROVISIONING_DATA_CLIENT_CONFIG,

    HANDLE_MESH_SERVICE_PROXY = 0x30,
        HANDLE_CHAR_MESH_PROXY_DATA_IN,
        HANDLE_CHAR_MESH_PROXY_DATA_IN_VALUE,
        HANDLE_CHAR_MESH_PROXY_DATA_OUT,
        HANDLE_CHAR_MESH_PROXY_DATA_OUT_VALUE,
        HANDLE_DESCR_MESH_PROXY_DATA_CLIENT_CONFIG,

    MESH_HANDLE_DEV_INFO_SERVICE = 0x40,
        MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_MFR_NAME,
        MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_MFR_NAME_VAL,

        MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_MODEL_NUM,
        MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_MODEL_NUM_VAL,

        MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_SYSTEM_ID,
        MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_SYSTEM_ID_VAL,
};

/*******************************************************************************
 * Function prototypes
 ******************************************************************************/
void bt_app_gatt_init(void);
void bt_app_gatt_db_init(wiced_bool_t is_authenticated);
void bt_app_gatt_proxy_send_cb(uint32_t conn_id, uint32_t ref_data,
                                    const uint8_t *packet, uint32_t packet_len);
wiced_result_t bt_app_management_callback(wiced_bt_management_evt_t event,
                                  wiced_bt_management_evt_data_t *p_event_data);
wiced_bool_t bt_app_gatt_is_connected(void);
uint8_t *bt_app_alloc_buffer(int len);
void bt_app_free_buffer(uint8_t *p_buf);

#endif /* BT_APP_H_ */
