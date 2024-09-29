//
// Copyleft  F.Chaxel 2017
//
/**** From STM32 *****************/
/*                          77 STM32 specific? 
* #include <stdbool.h>  
* #include <stdint.h>
*********************************/
// #include "hardware.h"  //also STM32 specific? 
#include "bacnet/basic/sys/mstimer.h"
#include "bacnet/basic/sys/mstimer.h"
#include "rs485.h"
#include "led.h"
#include "bacnet.h"
/*******************************/
// #define BACDL_MSTP

#include "esp_log.h"
// #include "esp_wifi.h"
// #include "esp_event_loop.h"
#include "nvs_flash.h"
#include "esp_log.h"

// #include "driver/gpio.h"

// #include "lwip/sockets.h"
// #include "lwip/netdb.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"

char *BACnet_Version = "1.0";
#define PRINT_ENABLED

int app_main(void)
{

    //gpio_set_direction(32, GPIO_MODE_OUTPUT);
    //gpio_set_level(32, 1);
    struct mstimer Blink_Timer;

    mstimer_init();
    led_init();
    bacnet_init();
    mstimer_set(&Blink_Timer, 5000);
    for (;;) {
        if (mstimer_expired(&Blink_Timer)) {
            mstimer_reset(&Blink_Timer);
            led_ld3_toggle();
        }
        led_task();
        bacnet_task();
        vTaskDelay(1);
    }
}

// /* Entry point */
// void app_main()
// {
//     // Cannot run BACnet code here, the default stack size is to small : 4096
//     // byte
//     xTaskCreate(BACnetTask, /* Function to implement the task */
//         "BACnetTask", /* Name of the task */
//         10000, /* Stack size in words */
//         NULL, /* Task input parameter */
//         20, /* Priority of the task */
//         NULL); /* Task handle. */
// }