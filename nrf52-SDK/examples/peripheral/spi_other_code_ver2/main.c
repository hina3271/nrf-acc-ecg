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
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "LPFilter.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
//#include "app_error.h"
//#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif



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





#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

#define TEST_STRING "Nordic"
static uint8_t       m_tx_buf[16];// = {0x40, 0x1, 0x0, 0x0};           /**< TX buffer. */
static uint8_t       m_rx_buf[16];    /**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */

#define DRDY_PIN 17
#define RESET_PIN 15
#define START_PIN 16
//#define CLKSEL_PIN 26 //not yet

typedef enum 
{
    // System Commands
    ADS_CMND_WAKEUP    = 0x02,   // Wake-up from standby mode
    ADS_CMND_STANDBY   = 0x04,   // Enter standby mode
    ADS_CMND_RESET_CMD = 0x06,   // Reset the device registers
    ADS_CMND_START     = 0x08,   // Start/restart (synchronize) conversions
    ADS_CMND_STOP      = 0x0A,   // Stop conversion
    ADS_CMND_OFFSETCAL = 0x1A,   // Channel offset calibration - needs to be sent every time there is a change to the PGA gain
    // Data Read Commands
    ADS_CMND_RDATAC    = 0x10,   // Enable Read Data Continuous mode.
                                 // - This mode is the default mode at power-up.
    ADS_CMND_SDATAC    = 0x11,   // Stop Read Data Continuously mode
    ADS_CMND_RDATA     = 0x12,   // Read data by command; supports multiple read back.
    // Register Read/Write Commands
    ADS_CMND_RREG      = 0x20,   // Read n nnnn registers starting at address r rrrr
                                 //  - first byte 001r rrrr (2xh)(2) - second byte 000n nnnn(2)
    ADS_CMND_WREG      = 0x40    // Write n nnnn registers starting at address r rrrr
                                 //  - first byte 010r rrrr (2xh)(2) - second byte 000n nnnn(2)
} ADS1x9xCommand_t;

typedef struct 
{
    uint8_t ADS1292_ID;
    uint8_t ADS1292_CONFIG1;       // 0x0001u ADS1x9x_REG_CONFIG1
    uint8_t ADS1292_CONFIG2;       // 0x0002u ADS1x9x_REG_CONFIG2
    uint8_t ADS1292_LOFF;          // 0x0003u ADS1x9x_REG_LOFF
    uint8_t ADS1292_CH1SET;        // 0x0004u ADS1x9x_REG_CH1SET
    uint8_t ADS1292_CH2SET;        // 0x0005u ADS1x9x_REG_CH2SET
    uint8_t ADS1292_RLD_SENS;      // 0x0006u ADS1x9x_REG_RLD_SENS
    uint8_t ADS1292_LOFF_SENS;     // 0x0007u ADS1x9x_REG_LOFF_SENS
    uint8_t ADS1292_LOFF_STAT;     // 0x0008u ADS1x9x_REG_LOFF_STAT
    uint8_t ADS1292_RESP1;         // 0x0009u ADS1x9x_REG_RESP1
    uint8_t ADS1292_RESP2;         // 0x000Au ADS1x9x_REG_RESP2
    uint8_t ADS1292_GPIO;          // 0x000Bu ADS1x9x_REG_GPIO
} ADS1292WriteRegisters_t;

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
//    NRF_LOG_INFO("Transfer completed.");
//    NRF_LOG_INFO(" Received:");
//    NRF_LOG_HEXDUMP_INFO(m_rx_buf, 16);
    
}

void WriteRegister(uint8_t address, uint8_t value) 
{
    memset(m_tx_buf, 0, 16);
    memset(m_rx_buf, 0, 16);
    spi_xfer_done = false;
    m_tx_buf[0] = 0x40 + address;
    m_tx_buf[1] = 1;
    m_tx_buf[2] = value;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 16, m_rx_buf, 3));

    while (!spi_xfer_done) 
    {
        __WFE();
    }

    return;
}

uint8_t ReadRegister(uint8_t address) 
{
    memset(m_tx_buf, 0, 16);
    memset(m_rx_buf, 0, 16);
    spi_xfer_done = false;
    m_tx_buf[0] = 0x20 + address;
    m_tx_buf[1] = 1;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 16, m_rx_buf, 3));

    while (!spi_xfer_done) 
    {
        __WFE();
    }

    return m_rx_buf[2];
}

void DebugWriteRegister(uint8_t address, uint8_t value) 
{
    uint8_t debug_value;
    debug_value = ReadRegister(address);
    printf("Value %d: %x\n\r", address, debug_value);
    WriteRegister(address, value);
    debug_value = ReadRegister(address);
    printf("Value %d: %x\n\r", address, debug_value);
}

void SendCommand(uint8_t op_code)
{
    memset(m_tx_buf, 0, 16);
    memset(m_rx_buf, 0, 16);
    spi_xfer_done = false;
    m_tx_buf[0] = op_code;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 16, m_rx_buf, 1));

    while (!spi_xfer_done) 
    {
        __WFE();
    }

    return;  
}

int32_t GetData() 
{
    int32_t raw_value;
    memset(m_tx_buf, 0, 16);
    memset(m_rx_buf, 0, 16);
    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 6, m_rx_buf, 6));

    while (!spi_xfer_done) 
    {
        __WFE();
    }

    raw_value = 0;
    if (m_rx_buf[3] & 0x80) 
    {
        raw_value = 0xff;
    }
    raw_value <<= 8;
    raw_value += m_rx_buf[3];
    raw_value <<= 8;
    raw_value += m_rx_buf[4];
    raw_value <<= 8;
    raw_value += m_rx_buf[5];
    return raw_value;
}

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
//  NRF_LOG_DEBUG("Handler");
//  NRF_LOG_FLUSH();
    /*
    int32_t adc_value;
    int32_t filter_out;
    //nrf_delay_us(20);

    adc_value = GetData();
    LPFilter_put(&filter, adc_value);
    filter_out = LPFilter_get(&filter);
    ma += filter_out;
    ma -= data[pointer];
    data[pointer] = filter_out;
    pointer++;
    pointer %= 1024;
  
    ecg_data = filter_out - (ma>>10);
    data_count++;
  */
}



int main(void)
{
    uint8_t value;
    uint32_t count = 0;
    int32_t adc_value;
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

    //printf("\r\nUART example started.\r\n");


    // Sets up the reset pin
    nrf_gpio_cfg_output(ADS_RST_PIN);
    nrf_gpio_pin_write(ADS_RST_PIN, 0);

    nrf_gpio_cfg_output(ADS_START_PIN);
    nrf_gpio_pin_write(ADS_START_PIN, 0);
  
    //nrf_gpio_cfg_output(ADS_CLKSEL_PIN);    //erase, yonghun, 6/8
    //nrf_gpio_pin_write(ADS_CLKSEL_PIN, 1);    //erase, yonghun, 6/8

    //nrf_gpio_cfg_input(DRDY_PIN, NRF_GPIO_PIN_PULLDOWN);

    error_code = nrf_drv_gpiote_init();
    if (error_code != NRF_SUCCESS )
    {
        //printf("\r\nDriver init failed!\n\r");
    }

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config.pull = NRF_GPIO_PIN_PULLDOWN;
    error_code = nrf_drv_gpiote_in_init(DRDY_PIN, &in_config, drdy_handler);
    if (error_code != NRF_SUCCESS )
    {
        //printf("\r\nInit failed!\n\r");
    }
  
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
   // spi_config.ss_pin = 7;
    spi_config.ss_pin = ADS_CSN_PIN;
    spi_config.miso_pin = ADS_MISO_PIN;
    spi_config.mosi_pin = ADS_MOSI_PIN;
    spi_config.sck_pin = ADS_SCK_PIN;
    spi_config.mode = NRF_DRV_SPI_MODE_1;//check this mode with ADS1292 datasheet, yonghun 2020 6/9
   // spi_config.frequency = NRF_DRV_SPI_FREQ_500K;
    spi_config.frequency = NRF_DRV_SPI_FREQ_250K;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));

    //nrf_gpio_pin_clear(ADS_RST_PIN);

    m_tx_buf[0] = 0x40;
    m_tx_buf[1] = 0x1;
    m_tx_buf[2] = 0;
    m_tx_buf[3] = 0;


    // Inits filter
    LPFilter_init(&filter);

    //printf("\r\nSPI example started.\n\r");

    nrf_delay_ms(200);
    //printf("\r\nDone Waiting.\n\r");

    nrf_gpio_pin_set(ADS_RST_PIN);
    nrf_delay_ms(1000);
    nrf_gpio_pin_clear(ADS_RST_PIN);
    nrf_delay_ms(10);
    nrf_gpio_pin_set(ADS_RST_PIN);
    nrf_delay_ms(1);


    // Stop Read Data Continously
    SendCommand(ADS_CMND_SDATAC);
    nrf_delay_ms(100);

    // Enables Internal Reference
    DebugWriteRegister(0, 0b00000000);
    nrf_delay_ms(20);

    // Sets sampling rate
    DebugWriteRegister(1, 1);
    nrf_delay_ms(20);

    // Enables Internal Reference
    DebugWriteRegister(2, 0b10100000);
    nrf_delay_ms(20);

    // Shorts inputs
  //  DebugWriteRegister(4, 0b00000001);
    // Normal Gain 6
    DebugWriteRegister(4, 0b00000000);
    // Normal Gain 12
  //  DebugWriteRegister(4, 0b01100000);
    nrf_delay_ms(20);

    // Disables Channel 2
  //  DebugWriteRegister(5, 0b10000000);
    nrf_delay_ms(20);
    uint8_t debug_value;
    debug_value = ReadRegister(0x09);
    printf("Value %d: %x\n\r", 0x09, debug_value);

    debug_value = ReadRegister(0x0A);
    printf("Value %d: %x\n\r", 0x0A, debug_value);

    debug_value = ReadRegister(0x0B);
    printf("Value %d: %x\n\r", 0x0B, debug_value);

    // Asserts start bit
    SendCommand(ADS_CMND_START);
  //  nrf_delay_ms(2000);

    // Start Read data Continouously
    SendCommand(ADS_CMND_RDATAC);
    nrf_delay_ms(2000);
    SendCommand(ADS_CMND_OFFSETCAL);
    //printf("\r\nRunning...\n\r");

    while (nrf_gpio_pin_read(DRDY_PIN) == 0);
    nrf_drv_gpiote_in_event_enable(DRDY_PIN, true);
    count = 0;

    
    //printf("\r\nSPI example started.\n\r");


    /* Toggle LEDs. */
    /*
    while (true)
    {
        for (int i = 0; i < LEDS_NUMBER; i++)
        {
            bsp_board_led_invert(i);
            nrf_delay_ms(500);
        }
    }
    */
    while (1) 
    {
        while (new_data == false);
        new_data = false;
    
        adc_value = GetData();
        
        LPFilter_put(&filter, adc_value);
        filter_out = LPFilter_get(&filter);
        ma += filter_out;
        ma -= data[pointer];
        data[pointer] = filter_out;
        pointer++;
        pointer %= 1024;
        
        ecg_data = filter_out - (ma >> 10);
        
        if( adc_value <= 3000000 && adc_value >= -3000000 )
        {
            printf("%d\n\r", adc_value);
            nrf_delay_ms(50);   //change this one -> change value
        }
        //value = ReadRegister(0);
        //NRF_LOG_INFO("Value: %x", value);
        //NRF_LOG_FLUSH();
    /*
        while (nrf_gpio_pin_read(DRDY_PIN) == 1);
        nrf_delay_us(20);
        GetData();
        raw_value = 0;
        if ( m_rx_buf[3] & 0x80) {
          raw_value = 0xff;
         }
        raw_value <<= 8;
        raw_value += m_rx_buf[3];
        raw_value <<= 8;
        raw_value += m_rx_buf[4];
        raw_value <<= 8;
        raw_value += m_rx_buf[5];
        adc_value = &raw_value;
    //   NRF_LOG_HEXDUMP_INFO(m_rx_buf, 6);
        LPFilter_put(&filter, *adc_value);
        filter_out = LPFilter_get(&filter);
        ma += filter_out;
        ma -= data[pointer];
        data[pointer] = filter_out;
        pointer++;
        pointer %= 1024;
        NRF_LOG_INFO("%d", filter_out - (ma>>10));
        NRF_LOG_FLUSH();
    */


     //   while( nrf_gpio_pin_read(DRDY_PIN) == 1 );
        //bsp_board_led_invert(BSP_BOARD_LED_0);
        //nrf_delay_ms(200);
    }
}
