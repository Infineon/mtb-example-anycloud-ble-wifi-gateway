/******************************************************************************
* File Name:   mqtt_publisher.c
*
* Description: This file contains the task that initializes the timer ISR
*              and publishes MQTT messages on the topic 'shadow device' to
*              send sensor data to cloud. The file also contains the timer
*              timer ISR that notifies the publisher task
*              about new data to be published.
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
#include "cyhal.h"
#include "cybsp.h"
#include "FreeRTOS.h"

/* Task header files */
#include "mqtt_publisher.h"
#include "mqtt_app.h"

/* Configuration file for MQTT client */
#include "mqtt_client_config.h"

/* Middleware libraries */
#include "cy_mqtt_api.h"
#include "cy_retarget_io.h"

/******************************************************************************
* Macros
******************************************************************************/
/* Interrupt priority for User Button Input. */
#define USER_BTN_INTR_PRIORITY          (5u)

/* The maximum number of times each PUBLISH in this example will be retried. */
#define PUBLISH_RETRY_LIMIT             (10u)

/* A PUBLISH message is retried if no response is received within this
 * time (in milliseconds).
 */
#define PUBLISH_RETRY_MS                (1000u)

/* Queue length of a message queue that is used to communicate with the 
 * publisher task.
 */
#define PUBLISHER_TASK_QUEUE_LENGTH     (3u)
/******************************************************************************
* Global Variables
*******************************************************************************/
/* FreeRTOS task handle for this task. */
TaskHandle_t publisher_task_handle;

/* Handle of the queue holding the commands for the publisher task */
QueueHandle_t publisher_task_q;

/* TimerHandle for the cloud timer */
TimerHandle_t publish_timer_handle;

/* Structure to store publish message information. */
cy_mqtt_publish_info_t publish_info =
{
    .qos = (cy_mqtt_qos_t) MQTT_MESSAGES_QOS,
    .topic = MQTT_PUB_TOPIC,
    .topic_len = (sizeof(MQTT_PUB_TOPIC) - 1),
    .retain = false,
    .dup = false
};
/* Node data to hold node status information */
node_data_t node_data;

/* MQTT publisher topic buffer*/
static uint8_t pub_data[255u]="\0";

/* AWS shadow data strings and lengths */
static const char _pub_open_string[21u]="{\"state\":{\"desired\":{";
static uint8_t _pub_open_string_len = 21u;

static const char _pub_close_string[3u]="}}}";

static const char *_values_string[3]={"\"node_status\":",",\"light_level\":",",\"switch_level\":"};
static uint8_t _values_string_len[3u] = { 14u, 15u, 16u};

static const char *_node_string[2u] = { "\"non provisioned\"", "\"provisioned\""};
static uint8_t _node_string_len[2u] = { 17u, 13u};

static uint8_t _light_string[5u] = "\"000\"";
static uint8_t _light_string_len = 5u;

static uint8_t _switch_string[5u] = "\"000\"";
static uint8_t _switch_string_len =5u;


/******************************************************************************
* Function Prototypes
*******************************************************************************/
/******************************************************************************
 * Function Name: publisher_task
 ******************************************************************************
 * Summary:
 *  Task that handles initialization of the user button GPIO, configuration of
 *  ISR, and publishing of MQTT messages to control the device that is actuated
 *  by the subscriber task.
 *
 * Parameters:
 *  void *pvParameters : Task parameter defined during task creation (unused)
 *
 * Return:
 *  void
 *
 ******************************************************************************/
void publisher_task(void *pvParameters)
{
    /* Status variable */
    cy_rslt_t result = CY_RSLT_SUCCESS;

    uint8_t pub_index=0u;

    /* Variable to receive new device state from the user button ISR. */
    uint32_t publish_device_state;

    /* Command to the MQTT client task */
    mqtt_task_cmd_t mqtt_task_cmd;
    /* To avoid compiler warnings */
    (void)pvParameters;

    printf("Press the user button (SW2) to publish the data to cloud.\r\n");

    for(;;)
    {
        /* Wait for notification from the User Button ISR. */
        xTaskNotifyWait(0, 0, &publish_device_state, portMAX_DELAY);

        pub_index = 0u;
        memcpy(&pub_data[pub_index],_pub_open_string,_pub_open_string_len);
        pub_index += _pub_open_string_len;
        memcpy(&pub_data[pub_index],_values_string[0],_values_string_len[0]);
        pub_index += _values_string_len[0];
        memcpy(&pub_data[pub_index],_node_string[node_data.node_status[1]],
                                    _node_string_len[node_data.node_status[1]]);
        pub_index += _node_string_len[node_data.node_status[1]];
        memcpy(&pub_data[pub_index],_values_string[1],_values_string_len[1]);
        pub_index += _values_string_len[1];

        _light_string[1] = (char) node_data.light_level[1]/100 + '0';
        _light_string[2] = (char) (node_data.light_level[1]/10)%10 + '0';
        _light_string[3] = (char) node_data.light_level[1]%10 + '0';

        memcpy(&pub_data[pub_index],_light_string,_light_string_len);
        pub_index+=_light_string_len;
        memcpy(&pub_data[pub_index],_values_string[2],_values_string_len[2]);
        pub_index+=_values_string_len[2];

        _switch_string[1] = (char) node_data.switch_level[1]/100 + '0';
        _switch_string[2] = (char) (node_data.switch_level[1]/10)%10 + '0';
        _switch_string[3] = (char) node_data.switch_level[1]%10 + '0';

        memcpy(&pub_data[pub_index],_switch_string,_switch_string_len);
        pub_index+=_switch_string_len;

        memcpy(&pub_data[pub_index],_pub_close_string,3u);

        /* Assign the publish message payload */
        publish_info.payload = (const char*) pub_data;

        publish_info.payload_len = strlen(publish_info.payload);

        printf("Publishing '%s' on the topic '%s'\n\n",
             (char *)publish_info.payload,
             publish_info.topic);

       result = cy_mqtt_publish(mqtt_connection, &publish_info);

       if (result != CY_RSLT_SUCCESS)
       {
          printf("Publisher: MQTT publish failed with error 0x%0X.\n\n", (int)result);

          /* Communicate the publish failure with the the MQTT
           * client task.
           */
          mqtt_task_cmd = HANDLE_MQTT_PUBLISH_FAILURE;
          xQueueSend(mqtt_task_q, &mqtt_task_cmd, portMAX_DELAY);
       }
       else
       {
           /* Copying the node current state  */
           node_data.node_status[0u] = node_data.node_status[1u];
           node_data.light_level[0u] = node_data.light_level[1u];
           node_data.switch_level[0u] = node_data.switch_level[1u];
       }

    }
}


/* [] END OF FILE */
