#ifndef _ARGOS_LIB_ADS_H_
#define _ARGOS_LIB_ADS_H_

#include <stdint.h>

#define kAdsBufferLength  16

typedef struct {
  int miso;
  int mosi;
  int sck;
  int csn;
  int rst;
  int drdy;
  int start;
  int clksel;
} ads_pins_t;

typedef enum {
  // System Commands
  ADS_CMND_WAKEUP = 0x02,    // Wake-up from standby mode
  ADS_CMND_STANDBY = 0x04,   // Enter standby mode
  ADS_CMND_RESET_CMD = 0x06, // Reset the device registers
  ADS_CMND_START = 0x08,     // Start/restart (synchronize) conversions
  ADS_CMND_STOP = 0x0A,      // Stop conversion
  ADS_CMND_OFFSETCAL = 0x1A, // Channel offset calibration - needs to be sent every time there is a change to the PGA gain
  // Data Read Commands
  ADS_CMND_RDATAC = 0x10, // Enable Read Data Continuous mode.
  // - This mode is the default mode at power-up.
  ADS_CMND_SDATAC = 0x11, // Stop Read Data Continuously mode
  ADS_CMND_RDATA = 0x12,  // Read data by command; supports multiple read back.
  // Register Read/Write Commands
  ADS_CMND_RREG = 0x20, // Read n nnnn registers starting at address r rrrr
  //  - first byte 001r rrrr (2xh)(2) - second byte 000n nnnn(2)
  ADS_CMND_WREG = 0x40 // Write n nnnn registers starting at address r rrrr
  //  - first byte 010r rrrr (2xh)(2) - second byte 000n nnnn(2)
} ADS1x9xCommand_t;

bool AdsOverflow();

bool AdsInit(const nrf_drv_spi_t spi, const ads_pins_t ads_pins);

int32_t AdsGetData();
//int32_t _AdsGetData();

void AdsWriteRegister(uint8_t address, uint8_t value);
uint8_t AdsReadRegister(uint8_t address);
void AdsDebugWriteRegister(uint8_t address, uint8_t value);
void AdsSendCommand(ADS1x9xCommand_t command);

bool AdsNewData();
int32_t AdsGetDataCount();

#endif
