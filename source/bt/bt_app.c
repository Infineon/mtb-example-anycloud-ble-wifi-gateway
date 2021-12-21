/******************************************************************************
* File Name:   bt_app.c
*
* Description: This file contains bluetooth stack management and GATT operation.
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
 * Header file includes
 ******************************************************************************/
#include "cy_retarget_io.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_mesh_provision.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_mesh_app.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_mesh_core.h"
#include "wiced_bt_trace.h"
#include "wiced_memory.h"
#include "mesh_application.h"
#include "mesh_app.h"
#include "bt_app.h"

/******************************************************************************
* Macros
*******************************************************************************/
#define BT_GATT_MAX_WRITE_SIZE                512u    /* GATT maximum write size */
#define BT_MESH_CORE_PROXY_ADV_INTERVAL        600u    /* Default 500ms to 800ms*/

/*******************************************************************************
 *          Function Prototypes
 *******************************************************************************/
       wiced_bt_gatt_status_t bt_app_gatt_callback(wiced_bt_gatt_evt_t event, 
                                                    wiced_bt_gatt_event_data_t *p_data);
static wiced_bt_gatt_status_t bt_app_gatt_write_handler(uint16_t conn_id, 
                                                    wiced_bt_gatt_opcode_t opcode, 
                                                    wiced_bt_gatt_write_req_t * p_data);
static wiced_bt_gatt_status_t bt_app_gatt_prep_write_handler(uint16_t conn_id, 
                                                    wiced_bt_gatt_opcode_t opcode, 
                                                    wiced_bt_gatt_write_req_t * p_data);
static wiced_bt_gatt_status_t bt_app_gatt_write_exec_handler(uint16_t conn_id, 
                                                    wiced_bt_gatt_opcode_t opcode,
                                                    wiced_bt_gatt_execute_write_req_t *p_data);

static wiced_bt_gatt_status_t bt_app_gatt_read_handler( uint16_t conn_id,
                                                    wiced_bt_gatt_opcode_t opcode,
                                                    wiced_bt_gatt_read_t *p_read_req,
                                                    uint16_t req_len);
static wiced_bt_gatt_status_t bt_app_gatt_req_read_by_type_handler(uint16_t conn_id,
                                                    wiced_bt_gatt_opcode_t opcode,
                                                    wiced_bt_gatt_read_by_type_t *p_read_req,
                                                    uint16_t req_len);
static wiced_bt_gatt_status_t bt_app_gatt_req_read_multi_handler(uint16_t conn_id,
                                                    wiced_bt_gatt_opcode_t opcode,
                                                    wiced_bt_gatt_read_multiple_req_t *p_read_req,
                                                    uint16_t req_len);

/*******************************************************************************
 *           Variables Definition
 ******************************************************************************/
/*
 * This is the GATT database for the WICED Mesh applications.
 * The database defines services, characteristics and
 * descriptors supported by the application.  Each attribute in the database
 * has a handle, (characteristic has two, one for characteristic itself,
 * another for the value).  The handles are used by the peer to access
 * attributes, and can be used locally by application, for example to retrieve
 * data written by the peer.  Definition of characteristics and descriptors
 * has GATT Properties (read, write, notify...) but also has permissions which
 * identify if peer application is allowed to read or write into it.
 * Handles do not need to be sequential, but need to be in order.
 *
 * Mesh applications have 2 GATT databases. One is shown while device is not
 * provisioned, this one contains Provisioning GATT service.  After device is
 * provisioned, it has GATT Proxy service.
 */
uint8_t gatt_db_unprovisioned[]=
{
    // Declare mandatory GATT service
    PRIMARY_SERVICE_UUID16(MESH_HANDLE_GATT_SERVICE, UUID_SERVICE_GATT),

    // Handle MESH_HANDLE_GAP_SERVICE (0x14): GAP service
    // Device Name and Appearance are mandatory characteristics.
    PRIMARY_SERVICE_UUID16(MESH_HANDLE_GAP_SERVICE, UUID_SERVICE_GAP),

    // Declare mandatory GAP service characteristic: Dev Name
    CHARACTERISTIC_UUID16(MESH_HANDLE_GAP_SERVICE_CHARACTERISTIC_DEV_NAME,
                          MESH_HANDLE_GAP_SERVICE_CHARACTERISTIC_DEV_NAME_VAL,
                          UUID_CHARACTERISTIC_DEVICE_NAME,
                          GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE),

    // Declare mandatory GAP service characteristic: Appearance
    CHARACTERISTIC_UUID16(MESH_HANDLE_GAP_SERVICE_CHARACTERISTIC_APPEARANCE,
                          MESH_HANDLE_GAP_SERVICE_CHARACTERISTIC_APPEARANCE_VAL,
                          UUID_CHARACTERISTIC_APPEARANCE,
                          GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE),

    // Mesh Provisionning Service.
    // This is the mesh application proprietary service. It has
    // characteristics which allows a device to provision a node.
    PRIMARY_SERVICE_UUID16(HANDLE_MESH_SERVICE_PROVISIONING, WICED_BT_MESH_CORE_UUID_SERVICE_PROVISIONING),

    // Handle HANDLE_CHAR_MESH_PROVISIONING_DATA_IN: characteristic Mesh Provisioning Data In
    // Handle HANDLE_CHAR_MESH_PROVISIONING_DATA_IN_VAL: characteristic Mesh Provisioning Data In Value
    // Characteristic is _WRITABLE and it allows writes.
    CHARACTERISTIC_UUID16_WRITABLE(HANDLE_CHAR_MESH_PROVISIONING_DATA_IN,
                                    HANDLE_CHAR_MESH_PROVISIONING_DATA_IN_VALUE,
                                    WICED_BT_MESH_CORE_UUID_CHARACTERISTIC_PROVISIONING_DATA_IN,
                                    GATTDB_CHAR_PROP_WRITE_NO_RESPONSE,
                                    GATTDB_PERM_WRITE_CMD | GATTDB_PERM_VARIABLE_LENGTH),

    // Handle HANDLE_CHAR_MESH_PROVISIONING_DATA_OUT: characteristic Mesh Provisioning Data Out
    // Handle HANDLE_CHAR_MESH_PROVISIONING_DATA_OUT_VAL: characteristic Mesh Provisioning Data Out Value
    // Characteristic can be notified to send provisioning PDU.
    CHARACTERISTIC_UUID16(HANDLE_CHAR_MESH_PROVISIONING_DATA_OUT,
                                    HANDLE_CHAR_MESH_PROVISIONING_DATA_OUT_VALUE,
                                    WICED_BT_MESH_CORE_UUID_CHARACTERISTIC_PROVISIONING_DATA_OUT,
                                    GATTDB_CHAR_PROP_NOTIFY,
                                    GATTDB_PERM_NONE),

    // Handle HANDLE_DESCR_MESH_PROVISIONING_DATA_CLIENT_CONFIG: Characteristic Client Configuration Descriptor.
    // This is standard GATT characteristic descriptor.  2 byte value 0 means that
    // message to the client is disabled.  Peer can write value 1 to enable
    // notifications.  Not _WRITABLE in the macro.  This
    // means that attribute can be written by the peer.
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HANDLE_DESCR_MESH_PROVISIONING_DATA_CLIENT_CONFIG,
                                     UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                     GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ),

    // Handle MESH_HANDLE_DEV_INFO_SERVICE (0x4D): Device Info service
    // Device Information service helps peer to identify manufacture or vendor of the
    // device.  It is required for some types of the devices, for example HID, medical,
    // and optional for others.  There are a bunch of characteristics available.
    PRIMARY_SERVICE_UUID16(MESH_HANDLE_DEV_INFO_SERVICE, UUID_SERVICE_DEVICE_INFORMATION),

    // Handle MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_MFR_NAME:
    //     characteristic Manufacturer Name
    // Handle MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_MFR_NAME_VAL:
    //     characteristic value
    CHARACTERISTIC_UUID16(MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_MFR_NAME,
                          MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_MFR_NAME_VAL,
                          UUID_CHARACTERISTIC_MANUFACTURER_NAME_STRING,
                          GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE),

    // Handle MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_MODEL_NUM:
    //     characteristic Model Number
    // Handle MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_MODEL_NUM_VAL:
    //     characteristic value
    CHARACTERISTIC_UUID16(MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_MODEL_NUM,
                          MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_MODEL_NUM_VAL,
                          UUID_CHARACTERISTIC_MODEL_NUMBER_STRING,
                          GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE),

    // Handle MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_SYSTEM_ID: characteristic System ID
    // Handle MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_SYSTEM_ID_VAL: characteristic value
    CHARACTERISTIC_UUID16(MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_SYSTEM_ID,
                          MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_SYSTEM_ID_VAL,
                          UUID_CHARACTERISTIC_SYSTEM_ID,
                          GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE),

};


uint8_t gatt_db_provisioned[]=
{
    // Declare mandatory GATT service
    PRIMARY_SERVICE_UUID16(MESH_HANDLE_GATT_SERVICE, UUID_SERVICE_GATT),

    // Handle MESH_HANDLE_GAP_SERVICE (0x14): GAP service
    // Device Name and Appearance are mandatory characteristics.
    PRIMARY_SERVICE_UUID16(MESH_HANDLE_GAP_SERVICE, UUID_SERVICE_GAP),

    // Declare mandatory GAP service characteristic: Dev Name
    CHARACTERISTIC_UUID16(MESH_HANDLE_GAP_SERVICE_CHARACTERISTIC_DEV_NAME,
                          MESH_HANDLE_GAP_SERVICE_CHARACTERISTIC_DEV_NAME_VAL,
                          UUID_CHARACTERISTIC_DEVICE_NAME,
                          GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE),

    // Declare mandatory GAP service characteristic: Appearance
    CHARACTERISTIC_UUID16(MESH_HANDLE_GAP_SERVICE_CHARACTERISTIC_APPEARANCE,
                          MESH_HANDLE_GAP_SERVICE_CHARACTERISTIC_APPEARANCE_VAL,
                          UUID_CHARACTERISTIC_APPEARANCE,
                          GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE),

    PRIMARY_SERVICE_UUID16(HANDLE_MESH_SERVICE_PROXY, WICED_BT_MESH_CORE_UUID_SERVICE_PROXY),

    // Handle HANDLE_CHAR_MESH_PROXY_DATA_IN: characteristic Mesh Proxy In
    // Handle HANDLE_CHAR_MESH_PROXY_DATA_IN_VALUE: characteristic Mesh Proxy In Value
    CHARACTERISTIC_UUID16_WRITABLE(HANDLE_CHAR_MESH_PROXY_DATA_IN,
                                    HANDLE_CHAR_MESH_PROXY_DATA_IN_VALUE,
                                    WICED_BT_MESH_CORE_UUID_CHARACTERISTIC_PROXY_DATA_IN,
                                    GATTDB_CHAR_PROP_WRITE_NO_RESPONSE,
                                    GATTDB_PERM_WRITE_CMD | GATTDB_PERM_VARIABLE_LENGTH),

    // Handle HANDLE_CHAR_MESH_PROXY_DATA_OUT: characteristic Mesh Proxy Out
    // Handle HANDLE_CHAR_MESH_PROXY_DATA_OUT_VALUE: characteristic Mesh Proxy Out Value
    CHARACTERISTIC_UUID16_WRITABLE(HANDLE_CHAR_MESH_PROXY_DATA_OUT,
                                    HANDLE_CHAR_MESH_PROXY_DATA_OUT_VALUE,
                                    WICED_BT_MESH_CORE_UUID_CHARACTERISTIC_PROXY_DATA_OUT,
                                    GATTDB_CHAR_PROP_NOTIFY,
                                    GATTDB_PERM_WRITE_CMD | GATTDB_PERM_VARIABLE_LENGTH),

    // Handle HANDLE_DESCR_MESH_PROXY_DATA_CLIENT_CONFIG: Characteristic Client Configuration Descriptor.
    // This is standard GATT characteristic descriptor.  2 byte value 0 means that
    // message to the client is disabled.  Peer can write value 1 to enable
    // notifications.  Not _WRITABLE in the macro.  This
    // means that attribute can be written by the peer.
    CHAR_DESCRIPTOR_UUID16_WRITABLE (HANDLE_DESCR_MESH_PROXY_DATA_CLIENT_CONFIG,
                                     UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                     GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ),


    // Handle MESH_HANDLE_DEV_INFO_SERVICE (0x4D): Device Info service
    // Device Information service helps peer to identify manufacture or vendor of the
    // device.  It is required for some types of the devices, for example HID, medical,
    // and optional for others.  There are a bunch of characteristics available.
    PRIMARY_SERVICE_UUID16(MESH_HANDLE_DEV_INFO_SERVICE, UUID_SERVICE_DEVICE_INFORMATION),

    // Handle MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_MFR_NAME:
    //     characteristic Manufacturer Name
    // Handle MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_MFR_NAME_VAL:
    //     characteristic value
    CHARACTERISTIC_UUID16(MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_MFR_NAME,
                          MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_MFR_NAME_VAL,
                          UUID_CHARACTERISTIC_MANUFACTURER_NAME_STRING,
                          GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE),

    // Handle MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_MODEL_NUM:
    //     characteristic Model Number
    // Handle MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_MODEL_NUM_VAL:
    //     characteristic value
    CHARACTERISTIC_UUID16(MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_MODEL_NUM,
                          MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_MODEL_NUM_VAL,
                          UUID_CHARACTERISTIC_MODEL_NUMBER_STRING,
                          GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE),

    // Handle MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_SYSTEM_ID: characteristic System ID
    // Handle MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_SYSTEM_ID_VAL: characteristic value
    CHARACTERISTIC_UUID16(MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_SYSTEM_ID,
                          MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_SYSTEM_ID_VAL,
                          UUID_CHARACTERISTIC_SYSTEM_ID,
                          GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE),

};

const uint32_t gatt_db_unprovisioned_size = sizeof(gatt_db_unprovisioned);
const uint32_t gatt_db_provisioned_size = sizeof(gatt_db_provisioned);

/* Structure for GATT attribute */
typedef struct
{
    uint16_t handle;
    uint16_t attr_len;
    void     *p_attr;
} attribute_t;

/* Structure for GATT prepare write command process */
typedef struct
{
    uint8_t                 *p_write_buffer;
    uint16_t                write_length;
    uint16_t                write_handle;
} bt_app_gatt_prep_write_t;

/* GATT proxy configuration */
uint8_t mesh_prov_data[20u]          = { 0u };
uint8_t mesh_proxy_data[20u]         = { 0u };
uint8_t mesh_prov_client_config[2]  = { BIT16_TO_8(GATT_CLIENT_CONFIG_NOTIFICATION) };
uint8_t mesh_proxy_client_config[2] = { 0u };

uint16_t conn_id = 0u;
uint16_t conn_mtu = 0u;
bt_app_gatt_prep_write_t bt_app_gatt_prep_write;

/* GATT attribute table */
attribute_t bt_app_attributes[] =
{
    { MESH_HANDLE_GAP_SERVICE_CHARACTERISTIC_DEV_NAME_VAL, 0, NULL },
    { MESH_HANDLE_GAP_SERVICE_CHARACTERISTIC_APPEARANCE_VAL, 0, NULL },
    { HANDLE_CHAR_MESH_PROVISIONING_DATA_IN_VALUE, 20, mesh_prov_data },
    { HANDLE_DESCR_MESH_PROVISIONING_DATA_CLIENT_CONFIG, sizeof(mesh_prov_client_config), mesh_prov_client_config },
    { HANDLE_CHAR_MESH_PROXY_DATA_IN_VALUE, 20, mesh_proxy_data },
    { HANDLE_DESCR_MESH_PROXY_DATA_CLIENT_CONFIG, sizeof(mesh_proxy_client_config), mesh_proxy_client_config },
    { MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_MFR_NAME_VAL, sizeof(mesh_mfr_name), mesh_mfr_name },
    { MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_MODEL_NUM_VAL, sizeof(mesh_model_num), mesh_model_num },
    { MESH_HANDLE_DEV_INFO_SERVICE_CHARACTERISTIC_SYSTEM_ID_VAL, sizeof(mesh_system_id), mesh_system_id },

};

/*******************************************************************************
 *               Function Definitions
 ******************************************************************************/

/*******************************************************************************
* Function Name: bt_app_management_callback
********************************************************************************
* Summary:
* This function handles the BT stack events.
*
* Parameters:
*  wiced_bt_management_evt_t event: event
*  wiced_bt_management_evt_data_t *p_event_data : Data corresponding to the event
*
* Return:
*  wiced_result_t : status
*
*******************************************************************************/
wiced_result_t bt_app_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_bt_ble_advert_mode_t          *p_mode;
    wiced_result_t                      result = WICED_BT_SUCCESS;
#if(BT_APP_TRACE_ENABLE)
    printf("Bluetooth management callback event: %x\n", event);
#endif
    switch (event)
    {
        /* Bluetooth  stack enabled */
    case BTM_ENABLED_EVT:
          mesh_application_init();
        break;

    case BTM_DISABLED_EVT:
        break;

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        p_mode = &p_event_data->ble_advert_state_changed;
#if(BT_APP_TRACE_ENABLE)
        printf("Bluetooth advertisement state changed:%d\r\n", *p_mode);
#endif
        if (*p_mode == BTM_BLE_ADVERT_OFF)
        {
            // On failed attempt to connect FW stops all connectable adverts.
            // If we disconnected then notify core to restart them
            if (!bt_app_gatt_is_connected())
            {
                /* Sending connection status with conn_id = 0 and mtu =0 */
                wiced_bt_mesh_core_connection_status(0, WICED_FALSE, 0, GATT_BLE_DEFAULT_MTU_SIZE);
            }
        }
        break;

    case BTM_BLE_SCAN_STATE_CHANGED_EVT:
#if(BT_APP_TRACE_ENABLE)
        printf("Bluetooth scan state changed:%d\n", p_event_data->ble_scan_state_changed);
#endif
        break;

    case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        result = WICED_BT_ERROR;
        break;

    default:
        break;
    }

    return result;
}


/*******************************************************************************
* Function Name: bt_app_gatt_init
********************************************************************************
* Summary:
* This function initilize the GATT callbacks.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void bt_app_gatt_init(void)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    /* Register with BT stack to receive GATT callback */
    status = wiced_bt_gatt_register(bt_app_gatt_callback);
    printf("GATT event handler registration status: 0x%x\r\n", status);

    /* Disable pairing for this application */
    wiced_bt_set_pairable_mode(WICED_FALSE, WICED_FALSE);

    memset(&bt_app_gatt_prep_write, 0, sizeof(bt_app_gatt_prep_write_t));

}


/*******************************************************************************
* Function Name: bt_app_gatt_db_init
********************************************************************************
* Summary:
* This function initilize the GATT DB for provison /unprovison node.
*
* Parameters:
*  is_authenticated : authentication status
*
* Return:
*  None
*
*******************************************************************************/
void bt_app_gatt_db_init(wiced_bool_t is_authenticated)
{
    wiced_bt_gatt_status_t gatt_status;

    if (!is_authenticated)
    {
        gatt_status = wiced_bt_gatt_db_init(gatt_db_unprovisioned, sizeof(gatt_db_unprovisioned), NULL);
        printf("Bluetooth GATT db init for mesh unprovisioned node status:%x\n", gatt_status);
    }
    // If device is provisioned we will always show the Proxy service.  If device really
    // does not support Proxy feature it will not send connectable adverts, so it will not
    // cause any problems.  If Proxy feature is supported and turned on, or if device is
    // making itself connectable after provisioning over PB-GATT or when Identification is
    // turned on, the Proxy service is required.  In the latter 2 cases, core will make sure that
    // the device does not relay packets received over GATT.
    else
    {
        gatt_status = wiced_bt_gatt_db_init(gatt_db_provisioned, sizeof(gatt_db_provisioned), NULL);
        printf("Bluetooth GATT db init for mesh provisioned node status:%x\n", gatt_status);
    }
    
    UNUSED_VARIABLE(gatt_status);
}


/*******************************************************************************
* Function Name: bt_app_gatt_request_callback
********************************************************************************
* Summary:
* Process GATT request callback from the peer.
*
* Parameters:
*  *p_data : GATT attribute request type data.
*
* Return:
*  wiced_bt_gatt_status_t : BT GATT status
*
*******************************************************************************/
wiced_bt_gatt_status_t bt_app_gatt_request_callback(wiced_bt_gatt_attribute_request_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

#if(BT_APP_TRACE_ENABLE)
    printf("bt_app_gatt_request_callback. conn id:%d\r\n", p_data->conn_id);
#endif
    switch (p_data->opcode)
    {
    case GATT_REQ_READ:
    case GATT_REQ_READ_BLOB:
        result = bt_app_gatt_read_handler(conn_id, p_data->opcode, &p_data->data.read_req, p_data->len_requested);
        break;
    case GATT_REQ_READ_BY_TYPE:
        result = bt_app_gatt_req_read_by_type_handler(conn_id, p_data->opcode, &p_data->data.read_by_type, p_data->len_requested);
        break;
    case GATT_REQ_READ_MULTI:
    case GATT_REQ_READ_MULTI_VAR_LENGTH:
        result = bt_app_gatt_req_read_multi_handler(conn_id, p_data->opcode, &p_data->data.read_multiple_req, p_data->len_requested);
        break;

    case GATT_REQ_WRITE:
    case GATT_CMD_WRITE:
    case GATT_CMD_SIGNED_WRITE:
        result = bt_app_gatt_write_handler(p_data->conn_id, p_data->opcode, &p_data->data.write_req);
        break;

    case GATT_REQ_PREPARE_WRITE:
        result = bt_app_gatt_prep_write_handler(p_data->conn_id, p_data->opcode, &p_data->data.write_req);
        break;

    case GATT_REQ_EXECUTE_WRITE:
        result = bt_app_gatt_write_exec_handler(p_data->conn_id, p_data->opcode, &p_data->data.exec_write_req);
        break;

    case GATT_REQ_MTU:
#if(BT_APP_TRACE_ENABLE)
        printf("bt_app_gatt_request_callback:GATT_REQ_MTU:%x\n", p_data->data.remote_mtu);
#endif
        if (p_data->data.remote_mtu < GATT_BLE_DEFAULT_MTU_SIZE) {
            p_data->data.remote_mtu = GATT_BLE_DEFAULT_MTU_SIZE;
        }
        // We will use that MTU in wiced_bt_mesh_core_connection_status() at notifications enable
        conn_mtu = MIN(p_data->data.remote_mtu, wiced_bt_cfg_settings.p_ble_cfg->ble_max_rx_pdu_size);
        wiced_bt_mesh_core_set_gatt_mtu(conn_mtu);
        result = wiced_bt_gatt_server_send_mtu_rsp(conn_id, p_data->data.remote_mtu, conn_mtu);
        break;

    case GATT_HANDLE_VALUE_CONF:
        result = WICED_BT_GATT_INVALID_HANDLE;
        break;
    default:
        break;
    }
    return result;
}


/*******************************************************************************
* Function Name: bt_app_gatt_callback
********************************************************************************
* Summary:
* Process GATT event callback from the peer.
*
* Parameters:
*  event : GATT event type.
*  *p_data : GATT event data.
*
* Return:
*  wiced_bt_gatt_status_t : BT GATT status
*
*******************************************************************************/
wiced_bt_gatt_status_t bt_app_gatt_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

#if(BT_APP_TRACE_ENABLE)
    printf("bt_app_gatt_callback. event:%x p_data:%x\n",  event, (uint32_t)p_data);
#endif

    switch (event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        printf("Bluetooth GATT connection status:%d conn_id:%x reason:%d role:%d\n",
                                            p_data->connection_status.connected,
                                            p_data->connection_status.conn_id,
                                            p_data->connection_status.reason,
                                            p_data->connection_status.link_role);

        result = WICED_BT_GATT_SUCCESS;
        /* If we are in Slave role, this is a connection for Proxy or for Provisioning server.
           Otherwise, we are a provisioner and we established connection to a new device or to
           a proxy device */
        if (p_data->connection_status.link_role == HCI_ROLE_PERIPHERAL)
        {
            if (wiced_bt_mesh_app_func_table.p_mesh_app_gatt_conn_status)
            {
                wiced_bt_mesh_app_func_table.p_mesh_app_gatt_conn_status(&p_data->connection_status);
            }

            if (p_data->connection_status.connected)
            {

                /* On connection up core stops proxy adverts.
                On next disconnection it will start adverts with interval 600 */
                wiced_bt_mesh_core_proxy_adv_interval = BT_MESH_CORE_PROXY_ADV_INTERVAL;

                conn_id = p_data->connection_status.conn_id;
                /* When connection is established befor GATTS_REQ_TYPE_MTU, use the default MTU 23 bytes
                   which makes max message length 20
                   Otherwise don't change MTU */
                if(conn_mtu == 0)
                    conn_mtu = GATT_BLE_DEFAULT_MTU_SIZE;
            }
            else
            {
                conn_id = 0;
                conn_mtu = 0;
                // On disconnect ref_data is disconnection reason.
                wiced_bt_mesh_core_connection_status(0, WICED_FALSE, p_data->connection_status.reason,
                                                                            GATT_BLE_DEFAULT_MTU_SIZE);
            }
        }
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        result = bt_app_gatt_request_callback(&p_data->attribute_request);
        break;

    case GATT_GET_RESPONSE_BUFFER_EVT:
    {
        wiced_bt_gatt_buffer_request_t *p_buf_req = &p_data->buffer_request;

        p_buf_req->buffer.p_app_ctxt =   (void *)bt_app_free_buffer;
        p_buf_req->buffer.p_app_rsp_buffer = (uint8_t *) bt_app_alloc_buffer(p_buf_req->len_requested);
#if(BT_APP_TRACE_ENABLE)
    printf("bt_app_gatt_callback. get buffer of len %d allocated %x ", p_buf_req->len_requested,
                                                        p_buf_req->buffer.p_app_rsp_buffer);
#endif
        result = WICED_BT_GATT_SUCCESS;
    }
    break;
    case GATT_APP_BUFFER_TRANSMITTED_EVT:
    {
        mesh_app_free_t pfn_free = (mesh_app_free_t)p_data->buffer_xmitted.p_app_ctxt;
        
        if (pfn_free)
            pfn_free(p_data->buffer_xmitted.p_app_data);
        result = WICED_BT_GATT_SUCCESS;
    }
    break;

    default:
    break;

    }
    return result;
}


/*******************************************************************************
* Function Name: bt_app_gatt_proxy_send_cb
********************************************************************************
* Summary:
* Callback function to send a packet over GATT proxy connection.  If 
* gatt_char_handle parameter is not zero, this device performs as a GATT client 
* and should use GATT Write Command to send the packet out. Otherwise, the device 
* is a GATT server and should use GATT Notification
*
* Parameters:
*  conn_id : connection id.
*  ref_data : ref_data.
*  *packet : packet data.
*  packet_len : packet data length.
*
* Return:
*  None
*
*******************************************************************************/
void bt_app_gatt_proxy_send_cb(uint32_t conn_id, uint32_t ref_data, const uint8_t *packet, uint32_t packet_len)
{

    uint8_t  *p_data = bt_app_alloc_buffer(packet_len);

    if(NULL == p_data)
    {
#if(BT_APP_TRACE_ENABLE)
        printf("bt_app_gatt_proxy_send_cb: no memory to write %d \r\n", packet_len);
#endif
        return;
    }
    else
    {
        memcpy(p_data, packet, packet_len);

        if (ref_data)
        {
            wiced_bt_gatt_write_hdr_t value;

            value.handle = (uint16_t)ref_data;
            value.offset = 0;
            value.len = packet_len;
            value.auth_req = 0;

            wiced_bt_gatt_client_send_write(conn_id, GATT_CMD_WRITE, &value, p_data,  (void *)bt_app_free_buffer);
        }
        else if (mesh_proxy_client_config[0] & GATT_CLIENT_CONFIG_NOTIFICATION)
        {
            wiced_bt_gatt_server_send_notification(conn_id, HANDLE_CHAR_MESH_PROXY_DATA_OUT_VALUE,
                                                        packet_len, p_data,  (void *)bt_app_free_buffer);
#if(BT_APP_TRACE_ENABLE)
            printf("bt_app_gatt_proxy_send_cb: send notification.\r\n");
#endif
        }
        else
        {
            bt_app_free_buffer(p_data);
        }
    }
}


/*******************************************************************************
* Function Name: bt_app_gatt_find_attribute
********************************************************************************
* Summary:
* This function to find attribute description by handle.
*
* Parameters:
*  handle : connection handle
*
* Return:
*  attribute_t* : attribute values
*
*******************************************************************************/
static attribute_t * bt_app_gatt_find_attribute(uint16_t handle)
{
    int i;
    for (i = 0; i < sizeof(bt_app_attributes) / sizeof(bt_app_attributes[0]); i++)
    {
        if (bt_app_attributes[i].handle == handle)
        {
            return (&bt_app_attributes[i]);
        }
    }
    printf("bt_app_find_attribute: attribute not found:%x\n", handle);
    return NULL;
}


/*******************************************************************************
* Function Name: bt_app_gatt_write_handler
********************************************************************************
* Summary:
* Handle a command packet received from the Bluetooth host.
*
* Parameters:
*  conn_id : connection id.
*  opcode : GATT request type opcode.
*  *p_data : GATT write data.
*
* Return:
*  wiced_bt_gatt_status_t : BT GATT status
*
*******************************************************************************/
static wiced_bt_gatt_status_t bt_app_gatt_write_handler(uint16_t conn_id, wiced_bt_gatt_opcode_t opcode, wiced_bt_gatt_write_req_t * p_data)
{
    wiced_bt_gatt_status_t result    = WICED_BT_GATT_SUCCESS;
    uint8_t                *attr     = p_data->p_val;
    uint16_t                len      = p_data->val_len;
    attribute_t            *pAttr;

        switch (p_data->handle)
        {
        case HANDLE_CHAR_MESH_PROVISIONING_DATA_IN_VALUE:
            if (mesh_node_authenticated)
            {
#if(BT_APP_TRACE_ENABLE)
                printf("bt_app_gatt_write_handler: mesh is started already. ignoring...\r\n");
#endif
                result = WICED_BT_GATT_WRONG_STATE;
                break;
            }
            // Pass received packet to pb_gat transport layer
            wiced_bt_mesh_core_provision_gatt_packet(0u, conn_id, attr, len);
            break;

        case HANDLE_CHAR_MESH_PROXY_DATA_IN_VALUE:
            // handle it the same way as mesh packet received via advert report
            if (!mesh_node_authenticated)
            {
#if(BT_APP_TRACE_ENABLE)
                printf("bt_app_gatt_write_handler: mesh isn't started. ignoring...\n");
#endif
                result = WICED_BT_GATT_WRONG_STATE;

                break;
            }
            wiced_bt_mesh_core_proxy_packet(attr, len);
            break;

        case HANDLE_DESCR_MESH_PROVISIONING_DATA_CLIENT_CONFIG:
        case HANDLE_DESCR_MESH_PROXY_DATA_CLIENT_CONFIG:
            pAttr = bt_app_gatt_find_attribute(p_data->handle);
            if (pAttr)
            {
                memset(pAttr->p_attr, 0u, pAttr->attr_len);
                memcpy(pAttr->p_attr, attr, len <= pAttr->attr_len ? len : pAttr->attr_len);
            }
            /* Use conn_mtu value to call wiced_bt_mesh_core_connection_status() just once
               on first write to HANDLE_DESCR_MESH_PROXY_DATA_CLIENT_CONFIG */
            if (conn_mtu != 0u)
            {
                wiced_bt_mesh_core_connection_status(conn_id, WICED_FALSE, 0u, conn_mtu);
                conn_mtu = 0u;
            }

            break;

        default:
#if(BT_APP_TRACE_ENABLE)
                printf("bt_app_gatt_write_handler: invalid handle. ignoring packet...\n");
#endif
            result = WICED_BT_GATT_INVALID_HANDLE;
            break;
        }

    if (!opcode) {
        /** this has been called on execute write */
        return result;
    }

    if (result == WICED_BT_GATT_SUCCESS) {
        wiced_bt_gatt_server_send_write_rsp(conn_id, opcode, p_data->handle);
    }
    else {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_data->handle, result);
    }

    return result;
}


/*******************************************************************************
* Function Name: bt_app_gatt_prep_write_handler
********************************************************************************
* Summary:
* Handle GATT Prepare Write command.
*
* Parameters:
*  conn_id : connection id.
*  opcode : GATT request type opcode.
*  *p_data : GATT write data.
*
* Return:
*  wiced_bt_gatt_status_t : BT GATT status
*
*******************************************************************************/
static wiced_bt_gatt_status_t bt_app_gatt_prep_write_handler(uint16_t conn_id, wiced_bt_gatt_opcode_t opcode, wiced_bt_gatt_write_req_t * p_write_req)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;

    if (!bt_app_gatt_prep_write.p_write_buffer)
    {
        bt_app_gatt_prep_write.p_write_buffer = bt_app_alloc_buffer(BT_GATT_MAX_WRITE_SIZE);
        if (!bt_app_gatt_prep_write.p_write_buffer) {
            result = WICED_BT_GATT_PREPARE_Q_FULL;
            wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_write_req->handle, result);
            return result;
        }
    }

    if (p_write_req->offset + p_write_req->val_len <= BT_GATT_MAX_WRITE_SIZE)
    {
        bt_app_gatt_prep_write.write_handle = p_write_req->handle;
        memcpy(bt_app_gatt_prep_write.p_write_buffer + p_write_req->offset, p_write_req->p_val, p_write_req->val_len);
        bt_app_gatt_prep_write.write_length = p_write_req->offset + p_write_req->val_len;
    }
    else
    {
        result = WICED_BT_GATT_INSUF_RESOURCE;
    }

    if (result == WICED_BT_GATT_SUCCESS) {
        /* send a null context here, since the p_write_buffer is to be freed on exe write, or disconnect */
        wiced_bt_gatt_server_send_prepare_write_rsp(conn_id, opcode, p_write_req->handle, p_write_req->offset, p_write_req->val_len,
            bt_app_gatt_prep_write.p_write_buffer + p_write_req->offset, NULL);
    }
    else {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_write_req->handle, result);
    }

    return result;
}


/*******************************************************************************
* Function Name: bt_app_gatt_write_exec_handler
********************************************************************************
* Summary:
* Handle GATT Write Execute command.
*
* Parameters:
*  conn_id : connection id.
*  opcode : opcode.
*  *p_data : GATT write data.
*
* Return:
*  wiced_bt_gatt_status_t : BT GATT status
*
*******************************************************************************/ 
static wiced_bt_gatt_status_t bt_app_gatt_write_exec_handler(uint16_t conn_id, wiced_bt_gatt_opcode_t opcode, wiced_bt_gatt_execute_write_req_t *p_req)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;
    wiced_bt_gatt_write_req_t write_req = {0};

    if (p_req->exec_write == GATT_PREPARE_WRITE_CANCEL) {
        bt_app_free_buffer(bt_app_gatt_prep_write.p_write_buffer);
        bt_app_gatt_prep_write.p_write_buffer = NULL;

        wiced_bt_gatt_server_send_execute_write_rsp(conn_id, opcode);

        return result;
    }

    if (bt_app_gatt_prep_write.p_write_buffer)
    {
        memset(&write_req, 0, sizeof(wiced_bt_gatt_write_req_t));
        write_req.handle  = bt_app_gatt_prep_write.write_handle;
        write_req.p_val   = bt_app_gatt_prep_write.p_write_buffer;
        write_req.val_len = bt_app_gatt_prep_write.write_length;

        result = bt_app_gatt_write_handler(conn_id, 0, &write_req);

        bt_app_free_buffer(bt_app_gatt_prep_write.p_write_buffer);
        bt_app_gatt_prep_write.p_write_buffer = NULL;
    }
    else
    {
        result = WICED_BT_GATT_INSUF_RESOURCE;
    }

    if (result == WICED_BT_GATT_SUCCESS) {
        wiced_bt_gatt_server_send_execute_write_rsp(conn_id, opcode);
    }
    else {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, write_req.handle, result);
    }

    return result;
}


/*******************************************************************************
* Function Name: bt_app_gatt_read_handler
********************************************************************************
* Summary:
* This function handles Read Requests received from the client device.
*
* Parameters:
*  conn_id : Connection ID
*  opcode : Bluetooth GATT request type opcode
*  p_read_req : Pointer to read request containing the handle to read
*  req_len : length of data requested
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e
*                          in wiced_bt_gatt.h
*
*******************************************************************************/
static wiced_bt_gatt_status_t bt_app_gatt_read_handler( uint16_t conn_id,
                                                         wiced_bt_gatt_opcode_t opcode,
                                                         wiced_bt_gatt_read_t *p_read_req,
                                                         uint16_t req_len)
{
    attribute_t  *puAttribute = NULL;
    int          attr_len_to_copy;
    uint8_t     *from;
    int          to_send;

    puAttribute = bt_app_gatt_find_attribute(p_read_req->handle);
    if ( NULL == puAttribute )
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle,
                                            WICED_BT_GATT_INVALID_HANDLE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }
    attr_len_to_copy = puAttribute->attr_len;
    if (p_read_req->offset >= puAttribute->attr_len)
     {
         wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle,
                                             WICED_BT_GATT_INVALID_OFFSET);
         return WICED_BT_GATT_INVALID_OFFSET;
     }
    to_send = MIN(req_len, attr_len_to_copy - p_read_req->offset);
    from = ((uint8_t *)puAttribute->p_attr) + p_read_req->offset;

    return wiced_bt_gatt_server_send_read_handle_rsp(conn_id, opcode, to_send, from, NULL); /* No need for context, as buff not allocated */;

}


/*******************************************************************************
* Function Name: bt_app_gatt_req_read_by_type_handler
********************************************************************************
 * Summary
 * Process read-by-type request from peer device
 *
 * Parameters:
 *  conn_id : connection id
 *  opcode : GATT request type opcode
 *  p_read_req : pointer to read request containing the handle to read
 *  req_len : length of data requested
 * 
 * Return:
 *  wiced_bt_gatt_status_t  BT GATT status
 ******************************************************************************/
static wiced_bt_gatt_status_t bt_app_gatt_req_read_by_type_handler(uint16_t conn_id,
                                                                    wiced_bt_gatt_opcode_t opcode,
                                                                    wiced_bt_gatt_read_by_type_t *p_read_req,
                                                                    uint16_t req_len)
{
    attribute_t *puAttribute = NULL;
    uint16_t attr_handle = p_read_req->s_handle;
    uint8_t *p_rsp = bt_app_alloc_buffer(req_len);
    uint8_t pair_len = 0;
    int used_len = 0;
    if ( NULL == p_rsp )
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, attr_handle, WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INSUF_RESOURCE;
    }
    /* Read by type returns all attributes of the specified type, between the start and end handles */
    while (WICED_TRUE)
    {
        attr_handle = wiced_bt_gatt_find_handle_by_type(attr_handle, p_read_req->e_handle,
                                                        &p_read_req->uuid);

        if (0 == attr_handle)
            break;

        puAttribute = bt_app_gatt_find_attribute(attr_handle);
        if ( NULL == puAttribute )
        {
            wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->s_handle,
                                                WICED_BT_GATT_ERR_UNLIKELY);
            bt_app_free_buffer(p_rsp);
            return WICED_BT_GATT_INVALID_HANDLE;
        }

        {
            int filled = wiced_bt_gatt_put_read_by_type_rsp_in_stream(p_rsp + used_len,
                                                                      req_len - used_len, &pair_len,
                                                                      attr_handle, puAttribute->attr_len,
                                                                      puAttribute->p_attr);
            if (0 == filled)
            {
                break;
            }
            used_len += filled;
        }
        /* Increment starting handle for next search to one past current */
        attr_handle++;
    }
    if ( !used_len )
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->s_handle,
                                            WICED_BT_GATT_INVALID_HANDLE);
        bt_app_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }
    /* Send the response */
    wiced_bt_gatt_server_send_read_by_type_rsp(conn_id, opcode, pair_len, used_len, p_rsp,
                                               (void *)bt_app_free_buffer);
    return WICED_BT_GATT_SUCCESS;
}


/*******************************************************************************
* Function Name: bt_app_gatt_req_read_multi_handler
********************************************************************************
 * Summary
 * Process write read multi request from peer device
 *
 * Parameters:
 *  conn_id : connection id
 *  opcode : GATT request type opcode
 *  p_read_req : pointer to read request containing the handle to read
 *  req_len : length of data requested
 * 
 * Return:
 *  wiced_bt_gatt_status_t  BT GATT status
 ******************************************************************************/
static wiced_bt_gatt_status_t bt_app_gatt_req_read_multi_handler(uint16_t conn_id,
                                                                  wiced_bt_gatt_opcode_t opcode,
                                                                  wiced_bt_gatt_read_multiple_req_t *p_read_req,
                                                                  uint16_t req_len)
{
    attribute_t *puAttribute = NULL;
    uint8_t *p_rsp = bt_app_alloc_buffer(req_len);
    int used_len = 0;
    int count;
    uint16_t handle = wiced_bt_gatt_get_handle_from_stream(p_read_req->p_handle_stream, 0);
    if ( NULL ==p_rsp )
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, handle, WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }
    /* Read by type returns all attributes of the specified type, between the start and end handles */
    for (count = 0; count < p_read_req->num_handles; count++)
    {
        handle = wiced_bt_gatt_get_handle_from_stream(p_read_req->p_handle_stream, count);
        puAttribute = bt_app_gatt_find_attribute(handle);
        if ( NULL == puAttribute )
        {
            wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, *p_read_req->p_handle_stream,
                                                WICED_BT_GATT_ERR_UNLIKELY);
            bt_app_free_buffer(p_rsp);
            return WICED_BT_GATT_ERR_UNLIKELY;
        }
        else
        {
            int filled = wiced_bt_gatt_put_read_multi_rsp_in_stream(opcode, p_rsp + used_len,
                                                                    req_len - used_len,
                                                                    puAttribute->handle,
                                                                    puAttribute->attr_len,
                                                                    puAttribute->p_attr);
            if (!filled)
            {
                break;
            }
            used_len += filled;
        }
    }
    if ( !used_len )
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, *p_read_req->p_handle_stream,
                                            WICED_BT_GATT_INVALID_HANDLE);
        bt_app_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }
   /* Send the response */
    wiced_bt_gatt_server_send_read_multiple_rsp(conn_id, opcode, used_len, p_rsp,
                                                (void *)bt_app_free_buffer);
    return WICED_BT_GATT_SUCCESS;
}


/*******************************************************************************
* Function Name: bt_app_gatt_is_connected
********************************************************************************
* Summary:
* Returns the GATT connection status.
*
* Parameters:
*  None
*
* Return:
*  wiced_bool_t : GATT connection status
*
*******************************************************************************/
wiced_bool_t bt_app_gatt_is_connected(void)
{
    return conn_id != 0;
}


/*******************************************************************************
* Function Name: bt_app_free_buffer
********************************************************************************
* Summary:
* Free the allocated memory
*
* Parameters:
*  *p_buf : pointer to the buffer.
*
* Return:
*  None
*
*******************************************************************************/
void bt_app_free_buffer(uint8_t *p_buf)
{
    wiced_bt_free_buffer(p_buf);
#if(BT_APP_TRACE_ENABLE)
    printf("bt_app_free_buffer 0x%x", p_buf);
#endif
    return;
}

/*******************************************************************************
* Function Name: bt_app_alloc_buffer
********************************************************************************
* Summary:
* Allocated the memory
*
* Parameters:
*  len : lenth of buffer.
*
* Return:
*  uint8_t : buffer
*
*******************************************************************************/
uint8_t *bt_app_alloc_buffer(int len)
{
    uint8_t *p_buf = wiced_bt_get_buffer(len);
#if(BT_APP_TRACE_ENABLE)
    printf("bt_app_alloc_buffer %d 0x%x", len, p_buf);
#endif
    return p_buf;
}

/* [] END OF FILE */
