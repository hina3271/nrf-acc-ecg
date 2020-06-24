#ifndef ACC_ECG_BOARD_H
#define ACC_ECG_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

 
/* SPI Communication Pin Configuration */
#define MISO_PIN    3
#define SCLK_PIN    4 
#define MOSI_PIN    5

/* ADS1292R Communication Pin Configuration */
#define ADS_DRDY_PIN    2
#define ADS_CS_PIN      6 
#define ADS_START_PIN   7
#define ADS_RST_PIN     8

/* BMI270 Communication Pin Configuration */
#define BMI_INT1_PIN    9
#define BMI_INT2_PIN    10 
#define BMI_CS_PIN      11

/* UART Communication Pin Configuration */
#define RX_PIN_NUMBER  27
#define TX_PIN_NUMBER  26
#define CTS_PIN_NUMBER 25
#define RTS_PIN_NUMBER 28


#ifdef __cplusplus
}
#endif

#endif 