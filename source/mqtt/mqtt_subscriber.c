/******************************************************************************
* File Name:   mqtt_subscriber.c
*
* Description: This file contains the task that initializes the subscribes to
*                the topic 'MQTT_TOPIC', to control the sensor data notifications
*                to the cloud.
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
#include "string.h"
#include "FreeRTOS.h"

/* Task header files */
#include "mqtt_subscriber.h"
#include "mqtt_publisher.h"
#include "mqtt_app.h"

/* Configuration file for MQTT client */
#include "mqtt_client_config.h"

#include "mesh_cfg.h"
#include "mesh_client.h"

/* Middleware libraries */
#include "cy_mqtt_api.h"
#include "cy_retarget_io.h"

/******************************************************************************
* Macros
******************************************************************************/
/* Maximum number of retries for MQTT subscribe operation */
#define MAX_SUBSCRIBE_RETRIES                   (3u)

/* Time interval in milliseconds between MQTT subscribe retries. */
#define MQTT_SUBSCRIBE_RETRY_INTERVAL_MS        (1000)
/* The number of MQTT topics to be subscribed to. */
#define SUBSCRIPTION_COUNT          (1)


/******************************************************************************
* Global Variables
*******************************************************************************/
/* Task handle for this task. */
TaskHandle_t subscriber_task_handle;

/* Configure the subscription information structure. */
cy_mqtt_subscribe_info_t subscribe_info =
{
    .qos = (cy_mqtt_qos_t) MQTT_MESSAGES_QOS,
    .topic = MQTT_SUB_TOPIC,
    .topic_len = (sizeof(MQTT_SUB_TOPIC) - 1)
};


/******************************************************************************
* Function Prototypes
*******************************************************************************/
static void subscribe_to_topic(void);

/******************************************************************************
 * Function Name: subscriber_task
 ******************************************************************************
 * Summary:
 *  Task that sets up the user LED GPIO, subscribes to topic - 'MQTT_TOPIC',
 *  and controls the user LED based on the received task notification.
 *
 * Parameters:
 *  void *pvParameters : Task parameter defined during task creation (unused)
 *
 * Return:
 *  void
 *
 ******************************************************************************/
void subscriber_task(void *pvParameters)
{
    /* Light level variable */
    uint32_t light_level = 0;

    /* To avoid compiler warnings */
    (void)pvParameters;

     /* Subscribe with the configured parameters. */
    subscribe_to_topic();

    while (true)
    {
        /* Block till a notification is received from the subscriber callback. */
        xTaskNotifyWait(0, 0, (uint32_t*) &light_level, portMAX_DELAY);

        mesh_client_set_level(MESH_LEVEL_CLIENT_ELEMENT_INDEX, (uint8_t) light_level);

    }
}


/******************************************************************************
 * Function Name: subscribe_to_topic
 ******************************************************************************
 * Summary:
 *  Function that subscribes to the MQTT topic specified by the macro
 *  'MQTT_SUB_TOPIC'. This operation is retried a maximum of
 *  'MAX_SUBSCRIBE_RETRIES' times with interval of
 *  'MQTT_SUBSCRIBE_RETRY_INTERVAL_MS' milliseconds.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 ******************************************************************************/
static void subscribe_to_topic(void)
{
    /* Status variable */
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Command to the MQTT client task */
    mqtt_task_cmd_t mqtt_task_cmd;

    /* Subscribe with the configured parameters. */
    for (uint32_t retry_count = 0; retry_count < MAX_SUBSCRIBE_RETRIES; retry_count++)
    {
        result = cy_mqtt_subscribe(mqtt_connection, &subscribe_info, SUBSCRIPTION_COUNT);
        if (result == CY_RSLT_SUCCESS)
        {
            printf("MQTT client subscribed to the topic '%.*s' successfully.\r\n",
                    subscribe_info.topic_len, subscribe_info.topic);
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(MQTT_SUBSCRIBE_RETRY_INTERVAL_MS));
    }

    if (result != CY_RSLT_SUCCESS)
    {
        printf("MQTT Subscribe failed with error 0x%0X after %d retries...\r\n",
               (int)result, MAX_SUBSCRIBE_RETRIES);

        /* Notify the MQTT client task about the subscription failure */
        mqtt_task_cmd = HANDLE_MQTT_SUBSCRIBE_FAILURE;
        xQueueSend(mqtt_task_q, &mqtt_task_cmd, portMAX_DELAY);

    }
}

/******************************************************************************
 * Function Name: mqtt_subscription_callback
 ******************************************************************************
 * Summary:
 *  Callback to handle incoming MQTT messages. This callback prints the 
 *  contents of the incoming message and informs the subscriber task, via a 
 *  message queue, to turn on / turn off the device based on the received 
 *  message.
 *
 * Parameters:
 *  cy_mqtt_publish_info_t *received_msg_info : Information structure of the 
 *                                              received MQTT message
 *
 * Return:
 *  void
 *
 ******************************************************************************/
void mqtt_subscription_callback(cy_mqtt_publish_info_t *received_msg_info)
{
    /* Received MQTT message */
    const char *received_msg = received_msg_info->payload;
    int received_msg_len = received_msg_info->payload_len;
    /* Variable to hold led brightness level. */
    uint32_t received_level = 0u;

    printf("Subsciber: incoming MQTT message received:\n"
           "    Publish topic name: %.*s\n"
           "    Publish QoS: %d\n"
           "    Publish payload: %.*s\r\n",
           received_msg_info->topic_len, received_msg_info->topic,
           (int) received_msg_info->qos,
           (int) received_msg_info->payload_len, (const char *)received_msg_info->payload);



    /* Assign the cloud notify state depending on the received MQTT message. */
    if ((strlen(MQTT_DEVICE_GET_NODE_MESSAGE) == received_msg_len) &&
        (strncmp(MQTT_DEVICE_GET_NODE_MESSAGE, received_msg, received_msg_len) == 0u))
    {
        /* Notify the publisher task about the new data to be published. */
        xTaskNotify(publisher_task_handle, 1u, eSetValueWithoutOverwrite);
    }
    else if ((strlen(MQTT_DEVICE_SET_LIGHT_MESSAGE) <= received_msg_len) &&
            (strncmp(MQTT_DEVICE_SET_LIGHT_MESSAGE, received_msg, 8u) == 0u))
    {

           if(11u < received_msg_len)
           {
               /* Check the received values are digits */
               if((47u < received_msg[11u]) && (received_msg[11u] < 58u))
               {
                   received_level = received_msg[11u] - '0';
               }

               if((47u < received_msg[10u]) && (received_msg[10u] < 58u))
               {
                   received_level += (received_msg[10u] - '0') * (10u);
               }
           }
           else
           {
               if((47u < received_msg[10u]) && (received_msg[10u] < 58u))
               {
                   received_level = received_msg[10u] - '0';
               }
           }
           printf("Subscriber: received light brightness level: %d \r\n", (int)received_level);

           /* Notify the subscriber task about the received level. */
           xTaskNotify(subscriber_task_handle, received_level, eSetValueWithoutOverwrite);

    }
    else
    {
        printf("Subscriber: received MQTT message not in valid format!\n");
        return;
    }

}

/******************************************************************************
 * Function Name: mqtt_unsubscribe
 ******************************************************************************
 * Summary:
 *  Function that unsubscribes from the topic specified by the macro
 *  'MQTT_TOPIC'. This operation is called during cleanup by the MQTT client
 *  task.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 ******************************************************************************/
void mqtt_unsubscribe(void)
{
    cy_rslt_t result = cy_mqtt_unsubscribe(mqtt_connection, 
                                           (cy_mqtt_unsubscribe_info_t *) &subscribe_info, 
                                           SUBSCRIPTION_COUNT);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("MQTT unsubscribe operation failed with error 0x%0X!\r\n", (int)result);
    }
}

/* [] END OF FILE */
