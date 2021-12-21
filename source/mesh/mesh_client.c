/******************************************************************************
* File Name:   mesh_client.c
*
* Description: This file contains mesh client initialization & callback messages.
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

#include "wiced_bt_mesh_app.h"
#include "wiced_bt_mesh_core.h"
#include "wiced_bt_mesh_models.h"
#include "mesh_client.h"
#include "mesh_app.h"
#include "mesh_cfg.h"
#include "mqtt_app.h"

/******************************************************************************
* Macros
*******************************************************************************/
#define LIGHT_LEVEL_RANGE         9u /* Light level range for Light level model*/
/******************************************************************************
* Function Prototypes
*******************************************************************************/
static void mesh_client_level_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event,
                                     wiced_bt_mesh_level_status_data_t *p_data);

/******************************************************************************
 * Variables Definitions
 ******************************************************************************/

/******************************************************************************
 * Function Name: mesh_client_models_init
 ******************************************************************************
 * Summary:
 *  mesh client model initialization.
 *
 * Parameters:
 *  wiced_bool_t is_provisioned : provisioned status
 *
 * Return:
 *  void
 *
 ******************************************************************************/
void mesh_client_models_init(wiced_bool_t is_provisioned)
{
    wiced_bt_mesh_model_level_client_init(MESH_LEVEL_CLIENT_ELEMENT_INDEX, 
                            mesh_client_level_message_handler, is_provisioned);
}

/******************************************************************************
 * Function Name: mesh_client_level_message_handler
 ******************************************************************************
 * Summary:
 *  Process event received from the Level Server.
 *
 * Parameters:
 *  uint16_t event : event type
 *  wiced_bt_mesh_event_t *p_even : event pointer
 *  wiced_bt_mesh_level_status_data_t *p_data : event data
 * Return:
 *  void
 *
 ******************************************************************************/
void mesh_client_level_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event,
                                         wiced_bt_mesh_level_status_data_t *p_data)
{

    switch (event)
    {
    case WICED_BT_MESH_TX_COMPLETE:
        printf("Mesh client level tx complete status:%d\n", p_event->status.tx_flag);
        break;

    case WICED_BT_MESH_LEVEL_STATUS:
        printf("Mesh client level present:%d target:%d remaining time:%ld\n",
                                                            p_data->present_level,
                                                            p_data->target_level,
                                                            p_data->remaining_time);
        break;

    default:
        printf("Mesh client level:unknown event\n");
        break;
    }
    wiced_bt_mesh_release_event(p_event);
}

/******************************************************************************
 * Function Name: mesh_client_set_level
 ******************************************************************************
 * Summary:
 *  Set the light level.
 *
 * Parameters:
 *  uint8_t element_idx : element id
 *  uint8_t level : light level
 * Return:
 *  void
 *
 ******************************************************************************/
void mesh_client_set_level(uint8_t element_idx,uint8_t level)
{
    wiced_bt_mesh_level_set_level_t set_data;
    wiced_result_t result = WICED_SUCCESS;
    wiced_bool_t is_final = WICED_TRUE;

    /* Pre defined light levels range from 0% to 100%*/
    uint16_t light_level_step[LIGHT_LEVEL_RANGE] =
    {
        0x8000, 0xa000, 0xC000, 0xE000, 0x0000, 0x2000, 0x4000, 0x6000, 0x7FFF,
    };

    /* Validate the light level range */
    if(level < LIGHT_LEVEL_RANGE)
    {
        set_data.level = light_level_step[level & 0x0F];
        set_data.transition_time = 100u;
        set_data.delay = 0u;

        node_data.light_level[1] = level;
        printf("Mesh client set level:%d \n", set_data.level);
        result = wiced_bt_mesh_model_level_client_set(element_idx, is_final, &set_data);
        if(WICED_SUCCESS != result)
        {
            printf("Mesh client set level failed with error:%x \r\n", result);
        }
    }
    else
    {
         printf("Mesh client set level failed, range from 0 to 8 \r\n");
    }
}

/* [] END OF FILE */
