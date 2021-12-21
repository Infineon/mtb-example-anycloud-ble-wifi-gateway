/******************************************************************************
* File Name:   mesh_cfg.h
*
* Description: This file is the public interface of mesh_cfg.c
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
#ifndef MESH_CFG_H_
#define MESH_CFG_H_

#include "stdint.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_mesh_cfg.h"

/******************************************************************************
 *                             Macros
 ******************************************************************************/
#define MESH_PID                                0x3122
#define MESH_VID                                0x0002
#define MESH_CACHE_REPLAY_SIZE                  0x0008
#define MESH_COMPANY_ID                         0x0009

#define MESH_DEVICE_NAME                       "IoT Gateway"
#define MESH_DEVICE_APPERANCE                   APPEARANCE_GENERIC_TAG

#define MESH_ONOFF_SERVER_ELEMENT_INDEX         (0u)
#define MESH_LEVEL_CLIENT_ELEMENT_INDEX         (1u)

#define MESH_SUPPORT_PB_GATT                    /* To support GATT provisioning */

extern wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern wiced_bt_cfg_ble_t wiced_bt_cfg_ble;
extern wiced_bt_cfg_ble_scan_settings_t wiced_bt_cfg_scan_settings;
extern wiced_bt_mesh_core_config_t mesh_config;

#endif /* MESH_CFG_H_ */
