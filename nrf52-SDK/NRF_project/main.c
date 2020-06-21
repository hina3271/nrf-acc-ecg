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
#include "acc-ecg-board.h"
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

void Init_StartUp(void);
void Init_TimerA1(void);
unsigned char retInString(char* string);

// Function declarations

extern unsigned char ADS1x9xRegVal[16];


unsigned char LeadStatus = 0x0F;
// Global flags set by events
volatile unsigned char bCDCDataReceived_event = false;// Indicates data has been received without an open rcv operation
                 
#define MAX_STR_LENGTH 64
unsigned int SlowToggle_Period = 20000-1;
unsigned int FastToggle_Period = 2000-1;
unsigned short Two_5millisec_Period = 60000;

unsigned int EcgPtr =0;
unsigned char regval, Live_Streaming_flag = false;
extern void XT1_Stop(void);

unsigned char ECGTxPacket[64],ECGTxCount,ECGTxPacketRdy ;
unsigned char ECGRxPacket[64],ECGRxCount, dumy ;

struct ADS1x9x_state ECG_Recoder_state;
extern unsigned short Respiration_Rate;
unsigned short timeCtr =0;

union ECG_REC_Data_Packet {
	unsigned char ucECG_data_rec[32];
	short sECG_data_rec[16];
};
unsigned char ECG_Proc_data_cnt = 0;
union ECG_REC_Data_Packet ECG_REC_Proc_Data_Packet;


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


/* When UART is used for communication with the host do not use flow control.*/
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED
#endif

int32_t data[1024];
int pointer = 0;
int ma = 0;
uint32_t data_count = 0;
int32_t ecg_data;
LPFilter filter;
volatile bool new_data = false;

void drdy_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) 
{
    new_data = true;
}


int main(void)
{
    uint8_t value;
    uint32_t count = 0;
    int32_t adc_value = 0;
    int32_t filter_out;
    uint32_t raw_value;
 
    uint32_t error_code;


    /* Configure board. */
    bsp_board_init(BSP_INIT_LEDS);

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
          NRF_UART_BAUDRATE_115200   

      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);


    APP_ERROR_CHECK(err_code);

    printf("\r\nUART example started.\r\n");

    volatile unsigned short i, j;

    //Init_StartUp();                 //initialize device   //no use 2020 06/16 yonghun
    //Init_TimerA1();                 //no use 2020 06/16 yonghun
    //XT1_Stop();                     //no use 2020 06/16 yonghun


    Pin_Define();       //put by yh,  2020 06/16 yonghun
    nrf_gpio_pin_write(ADS_RST_PIN, 0);
    nrf_gpio_pin_write(ADS_START_PIN, 0);

    error_code = nrf_drv_gpiote_init();
    if (error_code != NRF_SUCCESS )
    {
        printf("\r\nDriver init failed!\n\r");
    }
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config.pull = NRF_GPIO_PIN_PULLDOWN;

    //nrf_gpio_cfg_input(DRDY_PIN, NRF_GPIO_PIN_PULLDOWN);
    error_code = nrf_drv_gpiote_in_init(DRDY_PIN, &in_config, drdy_handler);
    if (error_code != NRF_SUCCESS )
    {
        printf("\r\nInit failed!\n\r");
    }

    ADS1x9x_PowerOn_Init();           //a little change -> it works later change, 2020 06/16 by yh





    //Start_Read_Data_Continuous();      //RDATAC command

    //put setting register here


    /*write and readregister*/
    ADS1x9x_Enable_Start();				// Enable START (command)
    nrf_delay_us(30000);

    Set_ADS1x9x_Chip_Enable();					// CS = 0
    nrf_delay_us(300);

    ADS1x9x_Default_Reg_Init();               //perfect function   by yh   2020 06/16
    nrf_delay_us(300);



    //ADS1x9x_Read_ID_Regs(0x00);
    //printf("\r\n000000000000\r\n");

    ADS1x9x_Read_All_Regs(ADS1x9xRegVal);//read register success 2020 06/18 by yh


    /*read function*/
    //ADS1x9x_Enable_Start();				// Enable START (SET START to high)
    //Soft_Start_ADS1x9x();
    //nrf_delay_us(30000);

    //Set_ADS1x9x_Chip_Enable();					// CS = 0
    //nrf_delay_us(30000);

    //Start_Read_Data_Continuous();			//RDATAC command      //erase 06/17
    //nrf_delay_ms(2000);
    //Read_One_Data();

    //Channel_offset_Command();
    printf("\r\nRunning...\n\r");


    //Enable_ADS1x9x_DRDY_Interrupt();		// Enable DRDY interrupt
    //ADS1x9x_Enable_Start();				// Enable START (SET START to high)

    //////////////////////////
    printf("\n\rasdfasdf\n\r");

    ADS1x9x_Enable_Start();				// Enable START (command)
    nrf_delay_us(30000);
    
    Set_ADS1x9x_Chip_Enable();

    Start_Read_Data_Continuous();			//RDATAC command      //erase 06/17

    Channel_offset_Command();

    while (nrf_gpio_pin_read(DRDY_PIN) == 0);
    nrf_drv_gpiote_in_event_enable(DRDY_PIN, true);
    count = 0;



    while(1)
    {
        while (new_data == true)     //we must check this drdy code(interrupt)//when fall down -> it works
        {
            //new_data = false;
            //Read_One_Data();
            nrf_delay_us(500);
            adc_value = GetData();
            printf("%x\n\r", adc_value);
            nrf_delay_ms(500);   //change this one -> change value
            new_data = false;
        }
    }
}
            