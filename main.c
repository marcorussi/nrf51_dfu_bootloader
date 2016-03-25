/*
 * The MIT License (MIT)
 *
 * Copyright (c) [2015] [Marco Russi]
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

#include "dfu_transport.h"
#include "bootloader.h"
#include "bootloader_util.h"
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "ble.h"
#include "nrf.h"
#include "ble_hci.h"
#include "app_scheduler.h"
#include "app_timer_appsh.h"
#include "nrf_error.h"
#include "softdevice_handler_appsh.h"
#include "pstorage_platform.h"
#include "nrf_mbr.h"
#include "nrf_log.h"


/* ATTENTION: CHECK "BOOTLOADER_REGION_START" AND "BOOTLOADER_SETTINGS_ADDRESS" DEFINES VALUES IN DFU_TYPES.H FILE */

/* Include the service_changed characteristic. For DFU this should normally be the case. */
#define IS_SRVC_CHANGED_CHARACT_PRESENT 		1			

/* Value of the RTC1 PRESCALER register. */
#define APP_TIMER_PRESCALER             		0	
/* Size of timer operation queues. */		
#define APP_TIMER_OP_QUEUE_SIZE         		4                                                       

/* Maximum size of scheduler events. */
#define SCHED_MAX_EVENT_DATA_SIZE       		MAX(APP_TIMER_SCHED_EVT_SIZE, 0)                       

/* Maximum number of events in the scheduler queue. */
#define SCHED_QUEUE_SIZE                		20  

/* Low frequency clock source to be used by the SoftDevice */
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}                                                    


/* Callback function for asserts in the SoftDevice.
   This function will be called in case of an assert in the SoftDevice. */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/* Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
   This function is called from the scheduler in the main loop after a BLE stack event has been received. */
static void sys_evt_dispatch(uint32_t event)
{
    pstorage_sys_event_handler(event);
}


/* Function for initializing the BLE stack.
   Initializes the SoftDevice and the BLE event interrupt. */
static void ble_stack_init(void)
{
	uint32_t         err_code;
    sd_mbr_command_t com = {SD_MBR_COMMAND_INIT_SD, };
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    err_code = sd_mbr_command(&com);
    APP_ERROR_CHECK(err_code);
    
    err_code = sd_softdevice_vector_table_base_set(BOOTLOADER_REGION_START);
    APP_ERROR_CHECK(err_code);
   
    SOFTDEVICE_HANDLER_APPSH_INIT(&clock_lf_cfg, true);

    /* Enable BLE stack. */
    ble_enable_params_t ble_enable_params;
    /* Only one connection as a central is used when performing dfu. */
    err_code = softdevice_enable_get_default_config(1, 1, &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/* Function for event scheduler initialization. */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


/* Function for bootloader main entry. */
int main(void)
{
    uint32_t err_code;
    bool     dfu_start = false;
    bool     app_reset = (NRF_POWER->GPREGRET == BOOTLOADER_DFU_START);

    if (app_reset)
    {
        NRF_POWER->GPREGRET = 0;
    }

    /* This check ensures that the defined fields in the bootloader corresponds with actual setting in the nRF51 chip. */
    APP_ERROR_CHECK_BOOL(*((uint32_t *)NRF_UICR_BOOT_START_ADDRESS) == BOOTLOADER_REGION_START);
    APP_ERROR_CHECK_BOOL(NRF_FICR->CODEPAGESIZE == CODE_PAGE_SIZE);

    /* Initialize timer module, making it use the scheduler. */
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, true);

    (void)bootloader_init();

    if (bootloader_dfu_sd_in_progress())
    {
        err_code = bootloader_dfu_sd_update_continue();
        APP_ERROR_CHECK(err_code);

		ble_stack_init();
        scheduler_init();

        err_code = bootloader_dfu_sd_update_finalize();
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        /* if stack is present then continue initialization of bootloader. */
		ble_stack_init();
        scheduler_init();
    }

	/* get DFU start request status */
    dfu_start = app_reset;
    
    if (dfu_start || (!bootloader_app_is_valid(DFU_BANK_0_REGION_START)))
    {
        /* initiate an update of the firmware. */
        err_code = bootloader_dfu_start();
        APP_ERROR_CHECK(err_code);
    }

    if (bootloader_app_is_valid(DFU_BANK_0_REGION_START) && !bootloader_dfu_sd_in_progress())
    {
        /* Select a bank region to use as application region. 
		   Only applications running from DFU_BANK_0_REGION_START is supported. */
        bootloader_app_start(DFU_BANK_0_REGION_START);
    }
    
	/* perform a reset */
    NVIC_SystemReset();
}
