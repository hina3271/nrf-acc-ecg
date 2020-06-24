/**
 * Copyright (c) 2015 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "nrf.h"
#include "bsp.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif


#include "ADS1292.h"
#include "ADS1292_main.h"
#include <intrinsics.h>
#include <string.h>

#define SYSUNIV_BUSIFG	SYSUNIV_SYSBUSIV


/*******************************************************************************************************
 * 
 * bring it
 * 
 * *****************************************************************************************************/
// Function declarations
extern unsigned char ADS1x9xRegVal[16];
extern unsigned char ADS1292R_Register_Settings_yonghun[16];

int32_t data[1024];
int pointer = 0;
int ma = 0;
uint32_t data_count = 0;
int32_t ecg_data;
//LPFilter filter;
volatile bool new_data = false;
int32_t adc_value = 0;
uint32_t ecg_value = 0;
int32_t res_value = 0;

//#define ENABLE_LOOPBACK_TEST  /**< if defined, then this example will be a loopback test, which means that TX should be connected to RX to get data loopback. */

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}


#ifdef ENABLE_LOOPBACK_TEST
/* Use flow control in loopback test. */
#define UART_HWFC APP_UART_FLOW_CONTROL_ENABLED

/** @brief Function for setting the @ref ERROR_PIN high, and then enter an infinite loop.
 */
static void show_error(void)
{

    bsp_board_leds_on();
    while (true)
    {
        // Do nothing.
    }
}


/** @brief Function for testing UART loop back.
 *  @details Transmitts one character at a time to check if the data received from the loopback is same as the transmitted data.
 *  @note  @ref TX_PIN_NUMBER must be connected to @ref RX_PIN_NUMBER)
 */
static void uart_loopback_test()
{
    uint8_t * tx_data = (uint8_t *)("\r\nLOOPBACK_TEST\r\n");
    uint8_t   rx_data;

    // Start sending one byte and see if you get the same
    for (uint32_t i = 0; i < MAX_TEST_DATA_BYTES; i++)
    {
        uint32_t err_code;
        while (app_uart_put(tx_data[i]) != NRF_SUCCESS);

        nrf_delay_ms(10);
        err_code = app_uart_get(&rx_data);

        if ((rx_data != tx_data[i]) || (err_code != NRF_SUCCESS))
        {
            show_error();
        }
    }
    return;
}
#else
/* When UART is used for communication with the host do not use flow control.*/
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED
#endif


void drdy_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) 
{
    new_data = true;
}


int main(void)
{
    uint8_t value;
    uint32_t count = 0;
    //int32_t adc_value = 0;
    uint32_t error_code;


    /*uart*/
    uint32_t err_code;
    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          UART_HWFC,
          false,
#if defined (UART_PRESENT)
          NRF_UART_BAUDRATE_115200   
#else
          NRF_UARTE_BAUDRATE_115200
#endif
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);


    APP_ERROR_CHECK(err_code);

    printf("\r\nUART example started.\r\n");


    /*ADS1292 pin define and drdy set up*/
    volatile unsigned short i, j;

    Pin_Define(); 

    nrf_gpio_pin_write(ADS_RST_PIN, 0);
    nrf_gpio_pin_write(ADS_START_PIN, 0);

    error_code = nrf_drv_gpiote_init();
    if (error_code != NRF_SUCCESS )
    {
        printf("\r\nDriver init failed!\n\r");
    }

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config.pull = NRF_GPIO_PIN_PULLDOWN;

    error_code = nrf_drv_gpiote_in_init(DRDY_PIN, &in_config, drdy_handler);
    if (error_code != NRF_SUCCESS )
    {
        printf("\r\nInit failed!\n\r");
    }


    /*ADS1292 power set up*/
    ADS1x9x_PowerOn_Init();          


    /*ASDS1292 write and readregister*/
    ADS1x9x_Enable_Start();				// Enable START (command)
    nrf_delay_us(30000);

    Set_ADS1x9x_Chip_Enable();					// CS = 0
    nrf_delay_us(300);

    ADS1x9x_Default_Reg_Init();             
    nrf_delay_us(300);

    ADS1x9x_Read_All_Regs(ADS1292R_Register_Settings_yonghun);
    nrf_delay_us(300);


    /*ASDS1292 read value by RDATAC*/
    ADS1x9x_Enable_Start();				// Enable START (command)
    nrf_delay_us(30000);
    
    Set_ADS1x9x_Chip_Enable();

    //Start_Read_Data_Continuous();			//RDATAC command  // for read data continuous 

    //Channel_offset_Command();       //erase this code 2020 06/23

    //while (nrf_gpio_pin_read(DRDY_PIN) == 0);
    nrf_drv_gpiote_in_event_enable(DRDY_PIN, true);
    count = 0;

    printf("\r\nRunning...\n\r");

    unsigned char *data_ptr;     //input by yonghun 2020 06/22 //
    uint8_t data_buf[9];

    while(1)
    {
        
        while (new_data == true)    
        {
            //new_data = false;
            Read_One_Data(); 
            /*
            data_ptr=ADS1292R_ReadData();
            for(int i = 0; i < 9; i++)
            {
                data_buf[i] = *(data_ptr + i); 
            }
            for(int i = 0; i < 9; i++)
            {
                printf("   %d   ", data_buf[i]); 
            }
            printf("\r\n");
            nrf_delay_ms(500);  
            */

            //new_data = false;
            //Read_One_Data();                          //RDATA command  // for read data once
            
            adc_value = GetData();
            printf("%x     ", adc_value);
            ecg_value = ECG_GetData();
            printf("     %d\n\r", ecg_value);
            nrf_delay_ms(20);
            
            new_data = false;
            
        }
        
    }
}
            