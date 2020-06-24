
/// @file ads1292r.h
/// This file contains the API for ADS1292r device.
/// Detailed file description.

#ifndef ADS1292R_H
#define ADS1292R_H

#include "nrf52.h"
#include "nrf52_bitfields.h"
#include "spi.h"
#include "nrf_log.h"

//ADS1292R pull config pin
#define ADS_PULL_CONFIG_PIN                        13

//Read ID
#define ADS_READ_ID                                0x20

//Config 1 register
#define ADS_CONFIG1_READ                           0x21  
#define ADS_CONFIG1_WRITE                          0x41			

//Config 2 register
#define ADS_CONFIG2_READ                           0x22
#define ADS_CONFIG2_WRITE                          0x42

//Channel 1 register
#define ADS_CHANNEL1_READ                          0x24   
#define ADS_CHANNEL1_WRITE                         0x44

//Channel 2 register
#define ADS_CHANNEL2_READ                          0x25
#define ADS_CHANNEL2_WRITE                         0x45

//ADS commands
#define ADS_WAKEUP           				               0x02
#define ADS_ENTER_STANDBY                          0x04
#define ADS_RESET_DEVICE                           0x06
#define ADS_START_CONVERSION                       0x08
#define ADS_STOP_CONVERSION                        0x0A
#define ADS_OFFSETCAL                              0x1A
 
#define ADS_RDATAC          				               0x10
#define ADS_SDATAC          				               0x11
#define ADS_RDATA           				               0x12

/*Config 1 register*/

/*Channel oversampling ratio/Data Rate : For both channels*/
#define ADS_CONFIG1_OVERSAMPLING_RATIO_POS         0x00
#define ADS_CONFIG1_OVERSAMPLING_RATIO_LEN         0x03 //3 bits
#define ADS_CONFIG1_OVERSAMPLING_RATIO_125SPS      (0x00 << ADS_CONFIG1_OVERSAMPLING_RATIO_POS)
#define ADS_CONFIG1_OVERSAMPLING_RATIO_250SPS      (0x01 << ADS_CONFIG1_OVERSAMPLING_RATIO_POS)
#define ADS_CONFIG1_OVERSAMPLING_RATIO_500SPS      (0x02 << ADS_CONFIG1_OVERSAMPLING_RATIO_POS)
#define ADS_CONFIG1_OVERSAMPLING_RATIO_1KSPS       (0x03 << ADS_CONFIG1_OVERSAMPLING_RATIO_POS)
#define ADS_CONFIG1_OVERSAMPLING_RATIO_2KSPS       (0x04 << ADS_CONFIG1_OVERSAMPLING_RATIO_POS)
#define ADS_CONFIG1_OVERSAMPLING_RATIO_4KSPS       (0x05 << ADS_CONFIG1_OVERSAMPLING_RATIO_POS)
#define ADS_CONFIG1_OVERSAMPLING_RATIO_8KSPS       (0x06 << ADS_CONFIG1_OVERSAMPLING_RATIO_POS)

/*Single shot conversion*/
#define ADS_CONFIG_1_CONVERSION_MODE_POS           0x07
#define ADS_CONFIG_1_CONVERSION_MODE_CONTINUOUS    (0x00 << ADS_CONFIG_1_CONVERSION_MODE_POS)
#define ADS_CONFIG_1_CONVERSION_MODE_SINGLE        (0x01 << ADS_CONFIG_1_CONVERSION_MODE_POS)

/*Config 2 register*/

/*Test signal frequency*/
#define ADS_CONFIG_2_TEST_FREQ_POS                 0x00
#define ADS_CONFIG_2_TEST_FREQ_DC                  (0x00 << ADS_CONFIG_2_TEST_FREQ_POS)
#define ADS_CONFIG_2_TEST_FREQ_SQUARE_WAVE         (0x01 << ADS_CONFIG_2_TEST_FREQ_POS)

/*Test signal selection*/
#define ADS_CONFIG_2_INT_TEST_POS                  0x01
#define ADS_CONFIG_2_INT_TEST_OFF                  (0x00 << ADS_CONFIG_2_INT_TEST_POS)
#define ADS_CONFIG_2_INT_TEST_ON                   (0x01 << ADS_CONFIG_2_INT_TEST_POS)

/*CLK connection*/
#define ADS_CONFIG_2_CLK_EN_POS                    0x03
#define ADS_CONFIG_2_CLK_DISABLED                  (0x00 << ADS_CONFIG_2_CLK_EN_POS)
#define ADS_CONFIG_2_CLK_ENABLED                   (0x01 << ADS_CONFIG_2_CLK_EN_POS)

/*VREF_4V*/
#define ADS_CONFIG_2_VREF_4V_POS                   0x04
#define ADS_CONFIG_2_VREF_4V_DISABLE               (0x00 << ADS_CONFIG_2_VREF_4V_POS) //VREF - 2.42V
#define ADS_CONFIG_2_VREF_4V_ENABLED               (0x01 << ADS_CONFIG_2_VREF_4V_POS) //VREF - 4.033V

/*Reference buffer power down*/
#define ADS_CONFIG_2_PDB_REFBUF_POS                0x05
#define ADS_CONFIG_2_PDB_REFBUF_POWER_DOWN         (0x00 << ADS_CONFIG_2_PDB_REFBUF_POS)
#define ADS_CONFIG_2_PDB_REFBUF_ENABLED    		     (0x01 << ADS_CONFIG_2_PDB_REFBUF_POS) 

/*Lead-off comparator power-down*/
#define ADS_CONFIG_2_PDB_LOFF_COMP_POS             0x06
#define ADS_CONFIG_2_PDB_LOFF_COMP_DISABLED        (0x00 << ADS_CONFIG_2_PDB_LOFF_COMP_POS)
#define ADS_CONFIG_2_PDB_LOFF_COMP_ENABLED         (0x01 << ADS_CONFIG_2_PDB_LOFF_COMP_POS)

/*Bit 7 must be set to one*/
#define ADS_CONFIG_2_BIT_7_POS                     0x07
#define ADS_CONFIG_2_BIT_7                         (0x01 << ADS_CONFIG_2_BIT_7_POS)

/*Channel 1 register*/

/*Channel 1 input selection*/
#define ADS_CHANNEL_1_INPUT_SELECTION_POS          0x00
#define ADS_CHANNEL_1_INPUT_SELECTION_LEN          0x04

#define ADS_CHANNEL_1_NORMAL_ELECTRODE_INPUT       (0x00 << ADS_CHANNEL_1_INPUT_SELECTION_POS)
#define ADS_CHANNEL_1_INPUT_SHORTED                (0x01 << ADS_CHANNEL_1_INPUT_SELECTION_POS)
#define ADS_CHANNEL_1_RLD_MEASURE                  (0x02 << ADS_CHANNEL_1_INPUT_SELECTION_POS)
#define ADS_CHANNEL_1_MVDD                         (0x03 << ADS_CHANNEL_1_INPUT_SELECTION_POS)
#define ADS_CHANNEL_1_TEMPERATURE_SENSOR           (0x04 << ADS_CHANNEL_1_INPUT_SELECTION_POS)
#define ADS_CHANNEL_1_TEST_SIGNAL                  (0x05 << ADS_CHANNEL_1_INPUT_SELECTION_POS)
#define ADS_CHANNEL_1_RLD_DRP                      (0x06 << ADS_CHANNEL_1_INPUT_SELECTION_POS)
#define ADS_CHANNEL_1_RLD_DRM                      (0x07 << ADS_CHANNEL_1_INPUT_SELECTION_POS)  
#define ADS_CHANNEL_1_RLD_DRPM                     (0x08 << ADS_CHANNEL_1_INPUT_SELECTION_POS)
#define ADS_CHANNEL_1_ROUTE_IN3P_IN3N              (0x09 << ADS_CHANNEL_1_INPUT_SELECTION_POS)

/*Channel 1 PGA gain setting*/
#define ADS_CHANNEL_1_PGA_GAIN_POS                 0x04
#define ADS_CHANNEL_1_PGA_GAIN_LEN                 0x03

#define ADS_CHANNEL_1_PGA_GAIN_6                   (0x00 << ADS_CHANNEL_1_PGA_GAIN_POS)
#define ADS_CHANNEL_1_PGA_GAIN_1                   (0x01 << ADS_CHANNEL_1_PGA_GAIN_POS)
#define ADS_CHANNEL_1_PGA_GAIN_2                   (0x02 << ADS_CHANNEL_1_PGA_GAIN_POS)
#define ADS_CHANNEL_1_PGA_GAIN_3                   (0x03 << ADS_CHANNEL_1_PGA_GAIN_POS)
#define ADS_CHANNEL_1_PGA_GAIN_4                   (0x04 << ADS_CHANNEL_1_PGA_GAIN_POS)
#define ADS_CHANNEL_1_PGA_GAIN_8                   (0x05 << ADS_CHANNEL_1_PGA_GAIN_POS)
#define ADS_CHANNEL_1_PGA_GAIN_12                  (0x06 << ADS_CHANNEL_1_PGA_GAIN_POS)

/*Channel 1 power down setting*/
#define ADS_CHANNEL_1_POWER_DOWN_POS               0x07
#define ADS_CHANNEL_1_POWER_DOWN_LEN               0x01

#define ADS_CHANNEL_1_POWER_DOWN_NORMAL_OPERATION  (0x00 << ADS_CHANNEL_1_POWER_DOWN_POS)
#define ADS_CHANNEL_1_POWER_DOWN_CH1_POWER_DOWN    (0x01 << ADS_CHANNEL_1_POWER_DOWN_POS)

/*Channel 2 register*/

/*Channel 2 input selection*/
#define ADS_CHANNEL_2_INPUT_SELECTION_POS          0x00
#define ADS_CHANNEL_2_INPUT_SELECTION_LEN          0x04

#define ADS_CHANNEL_2_NORMAL_ELECTRODE_INPUT       (0x00 << ADS_CHANNEL_2_INPUT_SELECTION_POS)
#define ADS_CHANNEL_2_INPUT_SHORTED                (0x01 << ADS_CHANNEL_2_INPUT_SELECTION_POS)
#define ADS_CHANNEL_2_RLD_MEASURE                  (0x02 << ADS_CHANNEL_2_INPUT_SELECTION_POS)
#define ADS_CHANNEL_2_MVDD                         (0x03 << ADS_CHANNEL_2_INPUT_SELECTION_POS)
#define ADS_CHANNEL_2_TEMPERATURE_SENSOR           (0x04 << ADS_CHANNEL_2_INPUT_SELECTION_POS)
#define ADS_CHANNEL_2_TEST_SIGNAL                  (0x05 << ADS_CHANNEL_2_INPUT_SELECTION_POS)
#define ADS_CHANNEL_2_RLD_DRP                      (0x06 << ADS_CHANNEL_2_INPUT_SELECTION_POS)
#define ADS_CHANNEL_2_RLD_DRM                      (0x07 << ADS_CHANNEL_2_INPUT_SELECTION_POS)  
#define ADS_CHANNEL_2_RLD_DRPM                     (0x08 << ADS_CHANNEL_2_INPUT_SELECTION_POS)
#define ADS_CHANNEL_2_ROUTE_IN3P_IN3N              (0x09 << ADS_CHANNEL_2_INPUT_SELECTION_POS)

/*Channel 2 PGA gain setting*/
#define ADS_CHANNEL_2_PGA_GAIN_POS                 0x04
#define ADS_CHANNEL_2_PGA_GAIN_LEN                 0x03

#define ADS_CHANNEL_2_PGA_GAIN_6                   (0x00 << ADS_CHANNEL_2_PGA_GAIN_POS)
#define ADS_CHANNEL_2_PGA_GAIN_1                   (0x01 << ADS_CHANNEL_2_PGA_GAIN_POS)
#define ADS_CHANNEL_2_PGA_GAIN_2                   (0x02 << ADS_CHANNEL_2_PGA_GAIN_POS)
#define ADS_CHANNEL_2_PGA_GAIN_3                   (0x03 << ADS_CHANNEL_2_PGA_GAIN_POS)
#define ADS_CHANNEL_2_PGA_GAIN_4                   (0x04 << ADS_CHANNEL_2_PGA_GAIN_POS)
#define ADS_CHANNEL_2_PGA_GAIN_8                   (0x05 << ADS_CHANNEL_2_PGA_GAIN_POS)
#define ADS_CHANNEL_2_PGA_GAIN_12                  (0x06 << ADS_CHANNEL_2_PGA_GAIN_POS)

/*Channel 2 power down setting*/
#define ADS_CHANNEL_2_POWER_DOWN_POS               0x07
#define ADS_CHANNEL_2_POWER_DOWN_LEN               0x01

#define ADS_CHANNEL_2_POWER_DOWN_NORMAL_OPERATION  (0x00 << ADS_CHANNEL_2_POWER_DOWN_POS)
#define ADS_CHANNEL_2_POWER_DOWN_CH1_POWER_DOWN    (0x01 << ADS_CHANNEL_2_POWER_DOWN_POS)

/**
  * @brief  This function can be used to get the Bio-Impedence data through ADS1292r with 
  *         the help of SPI communication.	
  */
uint64_t ads1292r_get_bioimpedance (void);

/**
  * @brief  Various ADS1292r registers can be set using this function	
  * @param  rgstr : Register whose value to be set (see ads1292r.h for list of registers)
  *         num_of_regs: Number of registers whose values are going to be set
  *         val: The value to which the register is set to
  */
void ads1292r_set_register(uint8_t rgstr, uint8_t num_of_regs, uint8_t val);

/**
  * @brief  Various ADS1292r registers can be read using this function	
  * @param  rgstr : Register whose value to be set (see ads1292r.h for list of registers)
  *         num_of_regs: Number of registers whose values are going to be set
  */
uint8_t ads1292r_read_register (uint8_t rgstr, uint8_t num_of_regs);

/**
  * @brief  Configure the pin for ADS1292r	
  */
void ads1292r_pin_config(void);

/**
  * @brief  This function can be used to send commands to the register
  *         
  * @param  cmd: different commands like ADS_WAKEUP, ADS_ENTER_STANDBY, ADS_RESET_DEVICE, ADS_START_CONVERSION,
  *                    ADS_STOP_CONVERSION, ADS_OFFSETCAL
  */
void ads1292r_send_command(uint8_t cmd);

/**
  * @brief  Enables the ADS1292r
  */
void ads1292r_enable(void);

/**
  * @brief  Initialize the ADS1292r interface
  */
void ads1292r_interface_init(void);

/**
  * @brief  Read the id of ADS1292r
  */
uint8_t ads1292r_read_id(void);

/**
  * @brief  Initialize ADS1292r
  */
void ads1292r_init(void);

#endif
