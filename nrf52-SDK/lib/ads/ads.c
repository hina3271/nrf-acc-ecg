#include <string.h>
#include <stdint.h>

#include "app_util_platform.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "lib/ads/ads.h"
#include "lib/circbuf/circbuf.h"

static nrf_drv_spi_t spi_;
static ads_pins_t pins_;
static uint8_t tx_buf_[kAdsBufferLength];
static uint8_t rx_buf_[kAdsBufferLength];

volatile bool spi_xfer_done;
volatile bool ads_overflow;
volatile bool ads_new_data;

void spi_event_handler(nrf_drv_spi_evt_t const *p_event,
    void *p_context) {
  spi_xfer_done = true;
}

bool AdsNewData(){
  return ads_new_data;
}

void ads_drdy_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  ads_new_data = true;
//  ecg_value = _AdsGetData();
//  if(CIRC_GBUF_PUSH(ADSBUFFER, ecg_value))
//    ads_overflow = true;

}

bool AdsOverflow() {
  if( ads_overflow ){
    ads_overflow = false;
    return true;
  }
  return false;
}

bool AdsInit(const nrf_drv_spi_t spi, const ads_pins_t ads_pins) {
  uint32_t error_code;

  spi_ = spi;
  pins_ = ads_pins;

  // Sets up the reset pin
  nrf_gpio_cfg_output(pins_.rst);
  nrf_gpio_pin_write(pins_.rst, 0);

  nrf_gpio_cfg_output(pins_.start);
  nrf_gpio_pin_write(pins_.start, 0);

  nrf_gpio_cfg_output(pins_.clksel);
  nrf_gpio_pin_write(pins_.clksel, 1);

  error_code = nrf_drv_gpiote_init();
  if (error_code != NRF_SUCCESS) {
    NRF_LOG_WARNING("Driver init failed!");
    return false;
  }

  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
  in_config.pull = NRF_GPIO_PIN_PULLDOWN;
  error_code = nrf_drv_gpiote_in_init(pins_.drdy, &in_config, ads_drdy_handler);
  if (error_code != NRF_SUCCESS) {
    NRF_LOG_WARNING("DRDY callback init failed!");
    return false;
  }

  nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
  spi_config.ss_pin = pins_.csn;
  spi_config.miso_pin = pins_.miso;
  spi_config.mosi_pin = pins_.mosi;
  spi_config.sck_pin = pins_.sck;
  spi_config.mode = NRF_DRV_SPI_MODE_1;
  spi_config.frequency = NRF_DRV_SPI_FREQ_500K;
  APP_ERROR_CHECK(nrf_drv_spi_init(&spi_, &spi_config, spi_event_handler, NULL));

  NRF_LOG_INFO("Configuring ADS");
  nrf_delay_ms(200);
  nrf_gpio_pin_set(pins_.rst);
  nrf_delay_ms(1000);
  nrf_gpio_pin_clear(pins_.rst);
  nrf_delay_ms(10);
  nrf_gpio_pin_set(pins_.rst);
  nrf_delay_ms(1);

  // Stop Read Data Continously
  AdsSendCommand(ADS_CMND_SDATAC);
  nrf_delay_ms(100);

  // Sets sampling rate
  AdsDebugWriteRegister(1, 1);
  nrf_delay_ms(20);

  // Enables Internal Reference
  AdsDebugWriteRegister(2, 0b10100000);
  nrf_delay_ms(20);

  // Shorts inputs
  //  AdsDebugWriteRegister(4, 0b00000001);
  // Normal Gain 6
  //AdsDebugWriteRegister(4, 0b00000000);
  // Normal Gain 12
  AdsDebugWriteRegister(4, 0b01100000);
  nrf_delay_ms(20);

  // Disables Channel 2
  //  AdsDebugWriteRegister(5, 0b10000000);
  nrf_delay_ms(20);

  // Asserts start bit
  AdsSendCommand(ADS_CMND_START);
  //  nrf_delay_ms(2000);

  // Start Read data Continouously
  AdsSendCommand(ADS_CMND_RDATAC);
  nrf_delay_ms(1000);
  AdsSendCommand(ADS_CMND_OFFSETCAL);

  while (nrf_gpio_pin_read(pins_.drdy) == 0);
  nrf_drv_gpiote_in_event_enable(pins_.drdy, true);

  NRF_LOG_INFO("ADS Configured");

  return true;
}

void AdsWriteRegister(uint8_t address, uint8_t value) {
  memset(tx_buf_, 0, kAdsBufferLength);
  memset(rx_buf_, 0, kAdsBufferLength);
  spi_xfer_done = false;
  tx_buf_[0] = 0x40 + address;
  tx_buf_[1] = 1;
  tx_buf_[2] = value;

  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_, tx_buf_, 16, rx_buf_, 3));

  while (!spi_xfer_done) {
    __WFE();
  }

  return;
}

uint8_t AdsReadRegister(uint8_t address) {
  memset(tx_buf_, 0, kAdsBufferLength);
  memset(rx_buf_, 0, kAdsBufferLength);
  spi_xfer_done = false;
  tx_buf_[0] = 0x20 + address;
  tx_buf_[1] = 1;

  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_, tx_buf_, 16, rx_buf_, 3));

  while (!spi_xfer_done) {
    __WFE();
  }

  return rx_buf_[2];
}

void AdsDebugWriteRegister(uint8_t address, uint8_t value) {
  uint8_t debug_value;
  debug_value = AdsReadRegister(address);
  NRF_LOG_INFO("Value %d: %x", address, debug_value);
  AdsWriteRegister(address, value);
  debug_value = AdsReadRegister(address);
  NRF_LOG_INFO("Value %d: %x", address, debug_value);
  NRF_LOG_FLUSH();
}

void AdsSendCommand(ADS1x9xCommand_t command) {
  memset(tx_buf_, 0, kAdsBufferLength);
  memset(rx_buf_, 0, kAdsBufferLength);
  spi_xfer_done = false;
  tx_buf_[0] = command;
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_, tx_buf_, 16, rx_buf_, 1));

  while (!spi_xfer_done) {
    __WFE();
  }

  return;
}

int32_t AdsGetData() {
  ads_new_data = false;
  int32_t raw_value;
  memset(tx_buf_, 0, kAdsBufferLength);
  memset(rx_buf_, 0, kAdsBufferLength);
  spi_xfer_done = false;
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_, tx_buf_, 16, rx_buf_, 6));

  while (!spi_xfer_done) {
    __WFE();
  }

  raw_value = 0;
  if (rx_buf_[3] & 0x80) {
    raw_value = 0xff;
  }
  raw_value <<= 8;
  raw_value += rx_buf_[3];
  raw_value <<= 8;
  raw_value += rx_buf_[4];
  raw_value <<= 8;
  raw_value += rx_buf_[5];
  return raw_value;
}
