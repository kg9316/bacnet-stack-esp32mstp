/**************************************************************************
 *
 * Copyright (C) 2011 Steve Karg <skarg@users.sourceforge.net>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Module Description:
 * Generate a periodic timer tick for use by generic timers in the code.
 *
 *************************************************************************/
#include <stdbool.h>
#include <stdint.h>
// #include "hardware.h"
#include "bacnet/basic/sys/mstimer.h"
#include "bacnet/basic/sys/debug.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "esp_timer.h"


/* counter for the various timers */
static volatile unsigned long Millisecond_Counter;
static volatile struct mstimer_callback_data_t *Callback_Head;


/**
 * Handles the interrupt from the timer
 */
void SysTick_Handler(void* arg)
{
    struct mstimer_callback_data_t *cb;

    /* increment the tick count */
    Millisecond_Counter++;
    cb = (struct mstimer_callback_data_t *)Callback_Head;
    while (cb) {
        if (mstimer_expired(&cb->timer)) {
            cb->callback();
            if (mstimer_interval(&cb->timer) > 0) {
                mstimer_reset(&cb->timer);
            }
        }
        cb = cb->next;
    }
}

/**
 * Returns the continuous milliseconds count, which rolls over
 *
 * @return the current milliseconds count
 */
unsigned long mstimer_now(void)
{
    return Millisecond_Counter;
}

void mstimer_callback(struct mstimer_callback_data_t *new_cb,
    mstimer_callback_function callback,
    unsigned long milliseconds)
{
    struct mstimer_callback_data_t *cb;

    if (new_cb) {
        new_cb->callback = callback;
        mstimer_set(&new_cb->timer, milliseconds);
    }
    if (Callback_Head) {
        cb = (struct mstimer_callback_data_t *)Callback_Head;
        while (cb) {
            if (!cb->next) {
                cb->next = new_cb;
                break;
            } else {
                cb = cb->next;
            }
        }
    } else {
        Callback_Head = new_cb;
    }
}



/**
 * Timer setup for 1 millisecond timer
 */
void mstimer_init(void)
{


 const esp_timer_create_args_t periodic_timer_args = {
            .callback = &SysTick_Handler,
            /* name is optional, but may help identify the timer when debugging */
            .name = "periodic"
    };

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    /* The timer has been created but is not running yet */

     ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1000));
}

