/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code of IoT Gateway Example for ModusToolbox.
*
* Related Document: See README.md
*
********************************************************************************
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
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "wiced_bt_types.h"
#include "wiced_memory.h"
#include "wiced_bt_stack.h"

#include "bt_app.h"
#include "bt_utils.h"
#include "mesh_app.h"
#include "mesh_server.h"
#include "mesh_client.h"
#include "mesh_application.h"
#include "mqtt_app.h"

#include <FreeRTOS.h>
#include <task.h>

/*******************************************************************************
* Macros
*******************************************************************************/
/* PWM Frequency for LED Blink rate at 2Hz */
#define PWM_FREQUENCY                   (2u)
/* PWM Duty-cycle (0 - 100)*/
#define LED_BLINK                       (50.0f)
#define LED_OFF                         (0.0f)
#define LED_ON                          (100.0f)

/*Button press and hold interval time for node reset */
#define BUTTON_LONG_PRESS_INTERVAL      (5000u)
/* Interrupt priority for the GPIO connected to the user button */
#define BUTTON_INTERRUPT_PRIORITY       (7u)
#define BUTTON_TASK_PRIORITY            (configMAX_PRIORITIES - 1)
#define BUTTON_TASK_STACK_SIZE          (configMINIMAL_STACK_SIZE * 2)

#define MESH_HEAP_SIZE                  (10240u)

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
void mesh_app_init_callback(wiced_bool_t is_provisioned);
static void led_set_state(uint8_t value);
static void button_task(void *pvParameters);
static void gpio_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);

/*******************************************************************************
* Global Variables
******************************************************************************/
uint64_t button_pushed_time = 0u;
uint32_t button_previous_value = 1u;
uint32_t button_pushed_duration = 0u;

wiced_bt_heap_t* p_mesh_heap = NULL;
/* PWM object */
cyhal_pwm_t pwm_led_control;

/* FreeRTOS task handle for button task.*/
TaskHandle_t  button_task_handle;

/******************************************************************************
 * Function Name: main
 ******************************************************************************
 * Summary:
 *  System entrance point. This function initializes retarget IO, PWM, sets up
 *  the BTSTACK, MQTT client task, and then starts the RTOS scheduler.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  int
 *
 ******************************************************************************/
int main()
{
    cy_rslt_t result;
    BaseType_t rtos_result;

    /* Initialize the board support package. */
    result = cybsp_init();
    CY_ASSERT(CY_RSLT_SUCCESS == result);

    /* To avoid compiler warnings. */
    (void) result;

    /* Enable global interrupts. */
    __enable_irq();
    
    /* Configure platform specific settings for the BT device */
    cybt_platform_config_init(&cybsp_bt_platform_cfg);


    /* Initialize retarget-io to use the debug UART port. */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                        CY_RETARGET_IO_BAUDRATE);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence to clear screen. */
    printf("\x1b[2J\x1b[;H");
    printf("===============================================================\n");
    printf("CE230858 - Bluetooth IoT Gateway\n");
    printf("===============================================================\n\n");

    /* Initialize the PWM */
    result = cyhal_pwm_init(&pwm_led_control, CYBSP_USER_LED, NULL);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("PWM init failed with error code: %lu\r\n", (unsigned long) result);
    }

    /* Start the PWM */
    result = cyhal_pwm_start(&pwm_led_control);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("PWM start failed with error code: %lu\r\n", (unsigned long) result);

    }

    led_set_state(LED_ON);

    if( CY_RSLT_SUCCESS == flash_memory_init())
    {
        printf("Flash memory initialized! \r\n");
    }

    /* Register call back and configuration with stack */
    result = wiced_bt_stack_init(bt_app_management_callback, &wiced_bt_cfg_settings);

    /* Check if stack initialization was successful */
    if( WICED_BT_SUCCESS == result)
    {
        printf("Bluetooth stack initialization successful!\r\n");
    }
    else
    {
        printf("Bluetooth stack initialization failed!\r\n");
        CY_ASSERT(0u);
    }

    p_mesh_heap = wiced_bt_create_heap("mesh_app", NULL, MESH_HEAP_SIZE, NULL, WICED_TRUE);

    if(NULL == p_mesh_heap)
    {
        printf("Heap memory allocation failed! \r\n");
        CY_ASSERT(0u);
    }


    /* Create Button Task for processing button presses */
     rtos_result = xTaskCreate(button_task,"Button Task", BUTTON_TASK_STACK_SIZE,
                               NULL, BUTTON_TASK_PRIORITY, &button_task_handle);
     if( pdPASS != rtos_result)
     {
         printf("Failed to create button task. \n");
         CY_ASSERT(0u);
     }

     /* Create the MQTT Client task. */
     rtos_result =  xTaskCreate(mqtt_client_task, "MQTT Client Task", MQTT_CLIENT_TASK_STACK_SIZE,
                                                        NULL, MQTT_CLIENT_TASK_PRIORITY, NULL);
     if( pdPASS != rtos_result)
     {
        printf("Failed to create MQTT task. \n");
        CY_ASSERT(0u);
     }

     /* Start the FreeRTOS scheduler. */
     vTaskStartScheduler();

     /* Should never get here. */
     CY_ASSERT(0u);
}

/*******************************************************************************
* Function Name: mesh_app_init_callback
********************************************************************************
* Summary:
*   Mesh application init callback.
*
* Parameters:
*  is_provisioned : provison status
*
* Return:
*  None
*
*******************************************************************************/
void mesh_app_init_callback(wiced_bool_t is_provisioned)
{

    printf("Mesh provision status:%d\r\n" , is_provisioned);
    /* Adv Data is fixed. Spec allows to put URI, Name, Appearance and Tx Power
     in the Scan Response Data. */
    if (!is_provisioned)
    {
        (void) mesh_app_adv_config((uint8_t*)MESH_DEVICE_NAME, MESH_DEVICE_APPERANCE);
    }

    node_data.node_status[1u] = is_provisioned;

    /* Set the PWM output frequency and duty cycle */
    if(!is_provisioned){

        led_set_state(LED_BLINK);
    }
    else
    {
        led_set_state(LED_OFF);
    }

    mesh_server_models_init(is_provisioned);
    mesh_client_models_init(is_provisioned);

    printf("Mesh model initialization done!\r\n");

}

/*******************************************************************************
* Function Name: led_set_state
********************************************************************************
*
* Summary:
*   Set the led brightness over PWM
*
* Parameters:
*   value: PWM duty cycle value
*
* Return:
*   None
*
*******************************************************************************/
void led_set_state(uint8_t value)
{
    cy_rslt_t result;

    result = cyhal_pwm_set_duty_cycle(&pwm_led_control, value, PWM_FREQUENCY);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("PWM set duty cycle failed with error code: %lu\r\n", (unsigned long) result);
        CY_ASSERT(false);
    }
}

/*******************************************************************************
* Function Name: button_task
********************************************************************************
*
* Summary:
*   This task initialize the button interrupt and handler
*
* Parameters:
*   void *pvParameters: Not used
*
* Return:
*   None
*
*******************************************************************************/
void button_task(void *pvParameters)
{
     cy_rslt_t result;

    cyhal_gpio_callback_data_t gpio_cb_data =
    {
        .callback = gpio_interrupt_handler,
        .callback_arg = NULL,
        .pin = NC,
        .next = NULL
    };
    /* Initialize the user button */
    result = cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT,
                    CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);

    if (CY_RSLT_SUCCESS != result)
    {
         printf("GPIO initialization failed! \r\n");
    }
    /* Configure GPIO interrupt. */

    cyhal_gpio_register_callback(CYBSP_USER_BTN, &gpio_cb_data);
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_BOTH,
                                BUTTON_INTERRUPT_PRIORITY, true);

    for(;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (button_pushed_duration < BUTTON_LONG_PRESS_INTERVAL)
        {
            printf("User button (SW2) pressed!\r\n");
            if(cloud_connected && wifi_connected)
            {
                /* Notify the publisher task about the new data to be published. */
                xTaskNotify(publisher_task_handle, 1u, eSetValueWithoutOverwrite);
            }

        }
        else
        {
            printf("User button (SW2) long pressed: mesh core factory reset\r\n");
            // More than 5 seconds means factory reset
            mesh_application_factory_reset();
        }
    }
}


/*******************************************************************************
* Function Name: gpio_interrupt_handler
********************************************************************************
* Summary:
*   GPIO interrupt handler.
*
* Parameters:
*  *handler_arg : Not used
*  event : Not used
*
*  Return:
*   None
*
*******************************************************************************/
static void gpio_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
    uint32_t value =  cyhal_gpio_read(CYBSP_USER_BTN);
    uint32_t current_time =  xTaskGetTickCountFromISR();

    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;


    if (value == button_previous_value)
    {
        return;
    }
    button_previous_value = value;


    if (value == 0u)
    {
        button_pushed_time = current_time;
        return;
    }

    // button is released
    button_pushed_duration = current_time - button_pushed_time;

    vTaskNotifyGiveFromISR(button_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

}

/* [] END OF FILE */