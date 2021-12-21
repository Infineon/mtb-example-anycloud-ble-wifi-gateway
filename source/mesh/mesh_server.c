/******************************************************************************
* File Name:   mesh_server.c
*
* Description: This file contains mesh server initialization & callback messages.
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
#include "cybsp_bt_config.h"
#include "wiced_bt_types.h"
#include "wiced_bt_stack.h"

#include "wiced_bt_types.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_mesh_app.h"
#include "wiced_bt_mesh_core.h"
#include "wiced_bt_mesh_models.h"
#include "mesh_server.h"
#include "mesh_cfg.h"
#include "mesh_app.h"
#include "mqtt_app.h"

/******************************************************************************
* Macros
*******************************************************************************/
#define TRANSITION_INTERVAL             (100u)

/******************************************************************************
* Function Prototypes
*******************************************************************************/
static void mesh_onoff_server_message_handler(uint8_t element_idx,
                                                 uint16_t event, void *p_data);
static void mesh_onoff_server_process_status(uint8_t element_idx,
                                         wiced_bt_mesh_onoff_status_data_t *p_data);
/******************************************************************************
 * Variables Definitions
 ******************************************************************************/

/* Structure to keep the state of onoff server. */
typedef struct
{
    uint8_t  present_state;
    uint8_t  target_state;
} mesh_onoff_server_t;

/* Application state ON/OFF */
mesh_onoff_server_t app_state;

/******************************************************************************
 * Function Name: mesh_server_models_init
 ******************************************************************************
 * Summary:
 *  mesh server model initialization.
 *
 * Parameters:
 *  wiced_bool_t is_provisioned : provisioned status
 *
 * Return:
 *  void
 *
 ******************************************************************************/
void mesh_server_models_init(wiced_bool_t is_provisioned)
{

    memset (&app_state, 0, sizeof(app_state));
    wiced_bt_mesh_model_onoff_server_init(MESH_ONOFF_SERVER_ELEMENT_INDEX, 
            mesh_onoff_server_message_handler, TRANSITION_INTERVAL, is_provisioned);

}

/******************************************************************************
 * Function Name: mesh_onoff_server_message_handler
 ******************************************************************************
 * Summary:
 *  onoff server mesh handler.
 *
 * Parameters:
 *  uint8_t element_idx : element id
 *  uint16_t event : event type
 *  uint16_t *p_data : p_data
 * Return:
 *  void
 *
 ******************************************************************************/
void mesh_onoff_server_message_handler(uint8_t element_idx, uint16_t event, void *p_data)
{
    switch (event)
    {
    /* Event for ON/OFF state chnage notification */
    case WICED_BT_MESH_ONOFF_STATUS:
        mesh_onoff_server_process_status(element_idx, (wiced_bt_mesh_onoff_status_data_t *)p_data);
        break;
    /* Event for setting ON/OFF state */
    case WICED_BT_MESH_ONOFF_SET:
        break;

    default:
        printf("Mesh onoff server: unknown event \r\n");
    }
}


/******************************************************************************
 * Function Name: mesh_onoff_server_process_status
 ******************************************************************************
 * Summary:
 * This function is called when command to change state is received over mesh.
 *
 * Parameters:
 *  uint8_t element_idx : element id
 *  wiced_bt_mesh_onoff_status_data_t p_status : onoff status
 * Return:
 *  void
 *
 ******************************************************************************/
void mesh_onoff_server_process_status(uint8_t element_idx, wiced_bt_mesh_onoff_status_data_t *p_status)
{

    printf("Mesh onoff server set status: present:%d target:%d remaining:%ld\r\n",
                                                        p_status->present_onoff,
                                                        p_status->target_onoff,
                                                        p_status->remaining_time);
    app_state.present_state = p_status->present_onoff;
    app_state.target_state = p_status->target_onoff;
    node_data.switch_level[1] = p_status->target_onoff;

}

/* [] END OF FILE */
