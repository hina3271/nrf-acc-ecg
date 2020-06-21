
/// @file ads1292r.c
/// This file contains the API for ADS1292r device.
/// Detailed file description.

#include "ads1292r.h"

//private data structure containing the pin configuration of ADS1292r
static gpio_out_config_t ads1292r_pull_config;
//private data structure containing the spi configuration to communicate with ADS1292r
static spi_master_config_t spi_ads1292r;

/**
  * @brief  This function can be used to get the Bio-Impedence data through ADS1292r with 
  *         the help of SPI communication.	
  */
uint64_t ads1292r_get_bioimpedance (void)
{
  uint8_t datar[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  spi_read_byte (datar, 9);
  volatile uint64_t impedance = ((uint64_t)datar[3] << 48) | ((uint64_t)datar[4] << 40) | ((uint64_t)datar[5] << 32) | ((uint64_t)datar[6] << 16) | ((uint64_t)datar[7] << 8) | ((uint64_t)datar[8] << 0);

  return impedance;
}

/**
  * @brief  Various ADS1292r registers can be set using this function	
  * @param  rgstr : Register whose value to be set (see ads1292r.h for list of registers)
  *         num_of_regs: Number of registers whose values are going to be set
  *         val: The value to which the register is set to
  */
void ads1292r_set_register(uint8_t rgstr, uint8_t num_of_regs, uint8_t val)
{
  // Register Address
	uint8_t data_write = 0;
	spi_write_byte(&rgstr, 1);
  // Transmit Amount
	data_write = num_of_regs -1 ;
	spi_write_byte(&data_write, 1);
  // Register Data
	data_write = val;
	spi_write_byte(&data_write, 1);
}

/**
  * @brief  Various ADS1292r registers can be read using this function	
  * @param  rgstr : Register whose value to be set (see ads1292r.h for list of registers)
  *         num_of_regs: Number of registers whose values are going to be set
  */
uint8_t ads1292r_read_register (uint8_t rgstr, uint8_t num_of_regs)
{
  uint8_t data_len=0;
	uint8_t reg_read=0;
	
	spi_write_byte(&rgstr, 1);
	
	data_len = num_of_regs - 1;
	spi_write_byte(&data_len, 1);
	
	spi_read_byte(&reg_read, 1);
	
	return reg_read;
}

/**
  * @brief  Configure the pin for ADS1292r	
  */
void ads1292r_pin_config()
{
	ads1292r_pull_config.pin = ADS_PULL_CONFIG_PIN;
	ads1292r_pull_config.pull = NO_PULL;
	ads1292r_pull_config.drive = STANDARD_0_STANDARD_1;
	ads1292r_pull_config.sense = DISABLE_SENSING;
	ads1292r_pull_config.input = DISCONNECT_INPUT_BUFFER;
	ads1292r_pull_config.NRF_GPIO_MODULE = NRF_P0;
	
	app_gpio_hal_configure_pin_output(&ads1292r_pull_config);
	app_gpio_hal_clear_output_pin(ads1292r_pull_config.pin, ads1292r_pull_config.NRF_GPIO_MODULE);
}


/**
  * @brief  This function can be used to send commands to the register
  *         
  * @param  cmd: different commands like ADS_WAKEUP, ADS_ENTER_STANDBY, ADS_RESET_DEVICE, ADS_START_CONVERSION,
  *                    ADS_STOP_CONVERSION, ADS_OFFSETCAL
  */
void ads1292r_send_command(uint8_t cmd)
{
	spi_write_byte(&cmd, 1);
}

/**
  * @brief  Enables the ADS1292r
  */
void ads1292r_enable(void)
{
	spi_enable_ads1292r(&spi_ads1292r);
}

/**
  * @brief  Initialize the ADS1292r interface
  */
void ads1292r_interface_init(void)
{
	spi_init_ads1292r(&spi_ads1292r);
	app_gpio_hal_set_output_pin(ads1292r_pull_config.pin, ads1292r_pull_config.NRF_GPIO_MODULE);
}

/**
  * @brief  Read the id of ADS1292r
  */
uint8_t ads1292r_read_id()
{
	uint8_t ads1292r_id=0;
	ads1292r_id = ads1292r_read_register(ADS_READ_ID, 1);
	
	return ads1292r_id;
}

/**
  * @brief  Initialize ADS1292r
  */
void ads1292r_init()
{
	uint8_t reg_read = 0;
	uint8_t data_write =0;

	//Initialize the ADS1292R pin configuration
	ads1292r_pin_config();
	
	//Initialize the ADS1292R interface
	ads1292r_interface_init();
	
	//Enable the ADS1292R
	ads1292r_enable();
	
	//Without this delay, ADS1292R behaves crazy
	for (int i=0; i<500000; i++)
	{
	}
	
	//Reset ADS1292R
  ads1292r_send_command(ADS_RESET_DEVICE);
	
	//Without this delay, ADS1292R behaves crazy
	for (int i=0; i<500000; i++)
	{
	}
  // Send Stop Data
	//ads1292r_send_command(ADS_SDATAC);
	ads1292r_send_command(ADS_SDATAC);

  //// Read ID
  uint8_t ads1292r_id;
	ads1292r_id = ads1292r_read_register(ADS_READ_ID, 1);

	// Write Config Register 1
	data_write = 0;
	data_write = ( ADS_CONFIG1_OVERSAMPLING_RATIO_125SPS ||
                 ADS_CONFIG_1_CONVERSION_MODE_CONTINUOUS) ;
	ads1292r_set_register(ADS_CONFIG1_WRITE, 1, data_write);
	
	// Write Config Register 2
	data_write = 0;
	data_write = (ADS_CONFIG_2_BIT_7) ;
	ads1292r_set_register(ADS_CONFIG2_WRITE, 1, data_write);
  
	// Write Channel 1 Register 
	data_write = 0;
  data_write = (ADS_CHANNEL_1_PGA_GAIN_1) ;
	ads1292r_set_register(ADS_CHANNEL1_WRITE, 1, data_write);
	
	// Write Channel 2 Register 
	data_write = 0;
	data_write = (ADS_CHANNEL_2_PGA_GAIN_1) ;
	ads1292r_set_register(ADS_CHANNEL2_WRITE, 1, data_write);
	
	for(int i = 0; i<5000; i++)
  {
    i++;
  }
	
	NRF_LOG_INFO ("config1 register : %x", ads1292r_read_register(ADS_CONFIG1_READ, 1));
	NRF_LOG_INFO ("config2 register : %x", ads1292r_read_register(ADS_CONFIG2_READ, 1));
	NRF_LOG_INFO ("channel1 register: %x", ads1292r_read_register(ADS_CHANNEL1_READ, 1));
	NRF_LOG_INFO ("channel2 register: %x", ads1292r_read_register(ADS_CHANNEL2_READ, 1));
	
	// Start Conversion
	ads1292r_send_command(ADS_START_CONVERSION);
}



