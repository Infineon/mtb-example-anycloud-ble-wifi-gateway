/******************************************************************************
* File Name:   mesh_app.c
*
* Description: This file contains mesh user application and callbacks.
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
#include "cybsp_bt_config.h"
#include "wiced_bt_types.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_mesh_core.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_mesh_provision.h"
#include "wiced_bt_mesh_app.h"
#include "wiced_memory.h"
#include "bt_app.h"
#include "bt_utils.h"
#include "mesh_app.h"
#include "mesh_application.h"
#include "mesh_platform_utils.h"

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
extern void              mesh_app_init_callback();
void                     mesh_application_init(void);
static wiced_bool_t      mesh_publication_callback(uint8_t elem_idx, uint16_t company_id,
                                                    uint16_t model_id, uint16_t period);
static void              mesh_factory_reset_callback(void);
static void              mesh_print_uuid(uint8_t *uuid);

uint8_t mesh_mfr_name[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MANUFACTURER_NAME] = { 'I', 'n', 'f', 'i', 'n', 'e', 'o', 'n', 0 };
uint8_t mesh_model_num[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MODEL_NUMBER]     = { '1', '2', '3', '4', 0, 0, 0, 0 };
uint8_t mesh_system_id[8]                                                  = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71 };

wiced_bool_t mesh_config_client = WICED_FALSE;

/*Private key for provisioning the node*/
uint8_t pb_priv_key[WICED_BT_MESH_PROVISION_PRIV_KEY_LEN] = { 0x52, 0x9A, 0xA0, 0x67, 0x0D, 0x72, 0xCD, 0x64,
                                                              0x97, 0x50, 0x2E, 0xD4, 0x73, 0x50, 0x2B, 0x03, 
                                                              0x7E, 0x88, 0x03, 0xB5, 0xC6, 0x08, 0x29, 0xA5,
                                                              0xA3, 0xCA, 0xA2, 0x19, 0x50, 0x55, 0x30, 0xBA };

/*
 * Mesh application library will call into application functions if provided by the application.
 */
wiced_bt_mesh_app_func_table_t wiced_bt_mesh_app_func_table =
{
    mesh_app_init_callback,         // application initialization
    NULL,                           // Default SDK platform button processing
    NULL,                           // GATT connection status
    NULL,                           // attention processing
    NULL,                           // notify period set
    NULL,                           // WICED HCI command
    NULL,                           // LPN sleep
    mesh_factory_reset_callback        // factory reset
};


/*******************************************************************************
* Function Name: mesh_publication_callback
********************************************************************************
* Summary: Application implements that function to get notified on the periodic 
*          publication event. If period is set to 0, the publication needs to stop.
*          Period between 1 and 0xFFFE indicate that Periodic Publication for the
*          specified model is being changed.  Value 0xFFFF indicates that the 
*          value needs to be published now.
*
* Parameters:
*  elem_idx : Element id index
*  company_id : company id
*  model_id : Model id
*  period : Publish period
* 
* Return:
*  wiced_boot_t : result
*
*******************************************************************************/
wiced_bool_t mesh_publication_callback(uint8_t elem_idx, uint16_t company_id, uint16_t model_id, uint16_t period)
{
    wiced_bt_mesh_core_received_msg_handler_t   p_message_handler = NULL;
    wiced_bt_mesh_event_t                       *p_event;
    uint8_t                                     idx_model;

    printf("mesh_publication_callback: idx:%d company_id:%x model_id:%x period:%04x\n", elem_idx, company_id, model_id, period);

    if (period != 0xFFFF)
    {
        // Check if application wants to process periodic notifications
        if (wiced_bt_mesh_app_func_table.p_mesh_app_notify_period_set != NULL)
        {
            // core reports, period in 100ms. Convert to milliseconds to send to the app.
            return (wiced_bt_mesh_app_func_table.p_mesh_app_notify_period_set(elem_idx, company_id, model_id, period * 100));
        }
        // Let core handle periodic publications
        return WICED_FALSE;
    }
    if (elem_idx < mesh_config.elements_num)
    {
        // get message handler of specific model
        for (idx_model = 0; idx_model < mesh_config.elements[elem_idx].models_num; idx_model++)
        {
            if (company_id == mesh_config.elements[elem_idx].models[idx_model].company_id
                && model_id == mesh_config.elements[elem_idx].models[idx_model].model_id)
            {
                p_message_handler = (wiced_bt_mesh_core_received_msg_handler_t)mesh_config.elements[elem_idx].models[idx_model].p_message_handler;
                break;
            }
        }
    }
    if (p_message_handler)
    {
        // create event, with 0 as destinaion address.
        if (NULL != (p_event = wiced_bt_mesh_create_event(elem_idx, company_id, model_id, 0, 0)))
        {
            if (p_message_handler(p_event, NULL, 0))
                return WICED_TRUE;
            wiced_bt_mesh_release_event(p_event);
        }
    }
    else
    {
        printf("mesh_publication_callback:ignored\r\n");
    }
    return WICED_FALSE;
}


/*******************************************************************************
* Function Name: mesh_application_init
********************************************************************************
* Summary: Application initilize the mesh core with required capablities 
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void mesh_application_init(void)
{
    wiced_result_t            result;
    wiced_bt_mesh_core_init_t init = { 0u };
    uint16_t                  nvm_idx_cfg_data_before_init;

    // Currently we can support up to 4 network keys.
    wiced_bt_mesh_core_net_key_max_num = 4u;
    wiced_bt_mesh_core_app_key_max_num = 8u;
    wiced_bt_mesh_scene_max_num = 10u;
    wiced_bt_mesh_scheduler_events_max_num = 16u; // PTS test uses index 15 (MMDL/SR/SCHS/BV-01-C )

    /* Enable or disable Mesh traces from makefile*/
#if(WICED_BT_MESH_TRACE_ENABLE)
    wiced_bt_mesh_models_set_trace_level(WICED_BT_MESH_CORE_TRACE_INFO);
    wiced_bt_mesh_core_set_trace_level(WICED_BT_MESH_CORE_TRACE_FID_ALL, WICED_BT_MESH_CORE_TRACE_DEBUG);
#endif
    // setup NVRAM IDs which will be used by core and models
    mesh_application_nvram_id_init();

    bt_app_gatt_init();

    //remember if we are provisioner (config client)
    mesh_config_client = mesh_application_get_element_count(WICED_BT_MESH_CORE_MODEL_ID_CONFIG_CLNT) > 0 ? WICED_TRUE : WICED_FALSE;

    // initialize core
    // each node shall be assigned a 128-bit UUID known as the Device UUID.
    // Device manufacturers shall follow the standard UUID format and generation
    // procedure to ensure the uniqueness of each Device UUID.

   if (mesh_nvram_access(WICED_FALSE, MESH_UUID_LOCAL_FLASH_ID, init.device_uuid,
                                                                   MESH_UUID_LOCAL_LENGTH, &result) != MESH_UUID_LOCAL_LENGTH)
    {
         // Not programmed at factory, generate UUID
         mesh_application_gen_uuid(init.device_uuid);

        // Save UUID in the NVRAM for future use
        if (MESH_UUID_LOCAL_LENGTH != mesh_nvram_access(WICED_TRUE, MESH_UUID_LOCAL_FLASH_ID, init.device_uuid,
                                                MESH_UUID_LOCAL_LENGTH, &result))
        {
            printf("Mesh failed to save UUID with error:%x\r\n", result);
        }
    }

    /* First 6 bytes and last 6 are random. Use them for BT addresses */
    memcpy(init.non_provisioned_bda, init.device_uuid, 6u);
    memcpy(init.provisioned_bda, &init.device_uuid[10u], 6u);
    /* Valid static random address should have 2 most significant bits set to 1 */
    init.non_provisioned_bda[0] |= 0xc0;
    init.provisioned_bda[0] |= 0xc0;

    printf("Mesh device UUID:");
    mesh_print_uuid(init.device_uuid);

    /* Assign config pointer to that variable at the startup. remote_provision_server uses it*/
    p_wiced_bt_mesh_cfg_settings = &wiced_bt_cfg_settings;

    init.p_config_data = &mesh_config;
    init.callback = mesh_message_handler_callback;
    init.pub_callback = mesh_publication_callback;
    init.proxy_send_callback = bt_app_gatt_proxy_send_cb;
    init.nvram_access_callback = mesh_nvram_access;
    init.attention_cb = NULL;
    init.state_changed_cb = mesh_state_changed_cb;
    init.scan_callback = mesh_start_stop_scan_callback;


    mesh_config.features |= WICED_BT_MESH_CORE_FEATURE_BIT_PB_GATT;
    /* Remember wiced_bt_mesh_core_nvm_idx_cfg_data. Core initialization can change it
     if detects it is changed after OTA FW upgrade */
    nvm_idx_cfg_data_before_init = wiced_bt_mesh_core_nvm_idx_cfg_data;
    result = wiced_bt_mesh_core_init(&init);
    if (result != WICED_BT_SUCCESS && result != WICED_BT_PENDING)
    {
        printf("Mesh core initialization failed with error code:0x%x\r\n", result);
        return;
    }
    mesh_node_authenticated = (result == WICED_BT_SUCCESS) ? WICED_TRUE : WICED_FALSE;
    printf("Mesh core initialization done. provision status %d\r\n",mesh_node_authenticated);
    /* If node is provisioned and wiced_bt_mesh_core_nvm_idx_cfg_data is changed after
    OTA FW upgrade then update mesh_nvm_idx_seq too */
    if (mesh_node_authenticated && (nvm_idx_cfg_data_before_init != wiced_bt_mesh_core_nvm_idx_cfg_data))
        mesh_nvm_idx_seq += wiced_bt_mesh_core_nvm_idx_cfg_data - nvm_idx_cfg_data_before_init;

     bt_app_gatt_db_init(mesh_node_authenticated);

     printf("Mesh GATT initialization done!\r\n");

    // Initialize own SEQ and RPL
    if (!mesh_application_seq_init())
        mesh_application_factory_reset();

    wiced_bt_mesh_app_provision_server_init(pb_priv_key, NULL);

    if (wiced_bt_mesh_app_func_table.p_mesh_app_init != NULL)
    {
        wiced_bt_mesh_app_func_table.p_mesh_app_init(mesh_node_authenticated);
    }
    // Now start mesh picking up tx power set by app in wiced_bt_mesh_core_adv_tx_power
    wiced_bt_mesh_core_start();
    printf("Mesh core application started!\r\n");

}


/*******************************************************************************
* Function Name: mesh_app_adv_config
********************************************************************************
* Summary: Application provided function to read/write information from/into
*            flash memory.
*
* Parameters:
*  *device_name : Name of the mesh node
*  appearance : Appearance of the mesh node
*
* Return:
*  wiced_boot_t : result
*
*******************************************************************************/
wiced_bool_t mesh_app_adv_config(uint8_t *device_name, uint16_t appearance)
{
    cy_rslt_t result;
    wiced_bt_ble_advert_elem_t  adv_elem[3];
    uint8_t                     buf[2];
    uint8_t                     num_elem = 0;

    if(NULL == device_name)
        return WICED_FALSE;

    /* Adv Data is fixed. Spec allows to put URI, Name, Appearance and Tx Power
    in the Scan Response Data. */

    wiced_bt_cfg_settings.device_name = (uint8_t *)device_name;
    wiced_bt_cfg_ble.appearance = (wiced_bt_gatt_appearance_t)appearance;

    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len = (uint16_t)strlen((const char*)wiced_bt_cfg_settings.device_name);
    adv_elem[num_elem].p_data = wiced_bt_cfg_settings.device_name;
    num_elem++;

    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_APPEARANCE;
    adv_elem[num_elem].len = 2;
    buf[0] = (uint8_t)wiced_bt_cfg_settings.p_ble_cfg->appearance;
    buf[1] = (uint8_t)(wiced_bt_cfg_settings.p_ble_cfg->appearance >> 8);
    adv_elem[num_elem].p_data = buf;
    num_elem++;

    result = wiced_bt_ble_set_raw_scan_response_data(num_elem, adv_elem);

    if(WICED_SUCCESS == result)
    {
        printf("Advertising in the name \"%s\"\n",wiced_bt_cfg_settings.device_name);
    }
    else
    {
        printf("Failed to set scan response data \n");
    }

    return result;
}


/*******************************************************************************
* Function Name: mesh_factory_reset_callback
********************************************************************************
* Summary: Callback for mesh factory reset
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void mesh_factory_reset_callback(void)
{
    /*Reset the flash on factory reset*/
    flash_memory_reset();
}


/*******************************************************************************
* Function Name: mesh_nvram_access
********************************************************************************
* Summary: Application provided function to read/write information from/into
*            flash memory.
*
* Parameters:
*  write : read/write flag
*  inx : id variable
*  node_info : node info
*  len : length of data
*  *p_result : flash memory operation result
*
* Return:
*  uint32_t : length of write/read data
*
*******************************************************************************/
uint32_t mesh_nvram_access(wiced_bool_t write, int inx, uint8_t* node_info,
                                     uint16_t len, wiced_result_t *p_result)
{
    uint32_t len_res = len;

    if (!write)
        len_res = (uint32_t)flash_memory_read(inx, len, (uint8_t*)node_info);
    else if (len != 0)
        len_res = flash_memory_write(inx, len, (uint8_t*)node_info);
    else
        *p_result = (wiced_result_t)flash_memory_delete(inx);

    *p_result = WICED_SUCCESS;
    return len_res;
}


/*******************************************************************************
* Function Name: mesh_print_uuid
********************************************************************************
* Summary: This is the utility function that prints the uuid of the mesh node
*
* Parameters:
*  *uuid : UUID of mesh node
*
* Return:
*  None
*
*******************************************************************************/
static void mesh_print_uuid(uint8_t *uuid)
{
    for(uint8_t i=0;i<MESH_UUID_LOCAL_LENGTH-1;i++)
    {
        printf("%2X:",uuid[i]);
    }
    printf("%2X\r\n",uuid[MESH_UUID_LOCAL_LENGTH-1]);
}

/* [] END OF FILE */
