#ifndef ARGOS_PROTO_V1_H
#define ARGOS_PROTO_V1_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

// LEDs definitions for PCA10040
#define LEDS_NUMBER    4

#define LED_START      19
#define LED_1          19
#define LED_2          20
#define LED_3          19
#define LED_4          20
#define LED_STOP       20

#define LEDS_ACTIVE_STATE 0

#define LEDS_INV_MASK  LEDS_MASK

#define LEDS_LIST { LED_3, LED_3, LED_4, LED_4 }

#define BSP_LED_0      LED_3
#define BSP_LED_1      LED_4



// BUTTONs definitions for PCA10040
#define BUTTONS_NUMBER 2

#define BUTTON_START   28
#define BUTTON_1       28
#define BUTTON_2       27
#define BUTTON_STOP    27
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0

#define BUTTONS_LIST { BUTTON_1, BUTTON_2 }

#define BSP_BUTTON_0   BUTTON_1
#define BSP_BUTTON_1   BUTTON_2




//ADS1292 define by yonghun, 2020 6/9
#define ADS_MISO_PIN   30
#define ADS_CSN_PIN    25//25 
#define ADS_MOSI_PIN   29 
#define ADS_SCK_PIN    26 

#define ADS_RST_PIN    22
#define ADS_DRDY_PIN   24//24
#define ADS_START_PIN  23

//#define ADS_CLKSEL_PIN 26

#define DRDY_PIN 24
#define RESET_PIN 22//22
#define START_PIN 23//23

//#define CLKSEL_PIN 26 //not yet, yonghun,  6/8
  

//UART pin number define
#define RX_PIN_NUMBER  8
#define TX_PIN_NUMBER  6
#define CTS_PIN_NUMBER 7
#define RTS_PIN_NUMBER 5
#define HWFC           true



#ifdef __cplusplus
}
#endif

#endif 
