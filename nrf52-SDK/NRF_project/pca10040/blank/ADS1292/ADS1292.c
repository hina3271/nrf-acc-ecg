
/****************************************************************************************************************************************************
*	ADS1x9x.c  - Provides access to ADS1x9x ECG Data Converter.																					*
* 		Functions: 
* 				1. ADS1x9x_Init() : Initiazation of ADS1292																						*
*				2. ADS1x9x_Default_Reg_Init(): 							                                                  													*
*				3. 								                                                  													*
*												                                                  													*
*****************************************************************************************************************************************************/
//#include "..\Common\device.h"
//#include "..\Common\types.h"          // Basic Type declarations

//#include "ADS1292_Nand_Flash.h"
//#include "ADS1292_USB_Communication.h"

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

#include "ADS1292_main.h"
#include "ADS1292.h"

//must check all header file    2020  06/13 yonghun

/**************************************************************************************************************************************************
*	        Prototypes									                                                  										  *
**************************************************************************************************************************************************/

/**************************************************************************************************************************************************
*	        Global Variables										                                  											  *
**************************************************************************************************************************************************/
unsigned char ADC_Read_data[16];
unsigned char ADS129x_SPI_cmd_Flag=0, ADS129x_SPI_data_Flag=0,  SPI_Send_count=0, SPI_Tx_Count = 0, SPI_Tx_buf[3];
unsigned char SPI_Rx_Data_Flag = 0,  SPI_Rx_buf[9], SPI_Rx_Count=0, SPI_Rx_exp_Count=0 ;
unsigned char ECG_Data_rdy;
long ADS1x9x_ECG_Data_buf[6];   //4bytes, 2020 06/13 yonghun

extern struct ADS1x9x_state ECG_Recoder_state;
extern unsigned char ECGRecorder_data_Buf[256], Recorder_head,Recorder_tail;
//extern struct ECGPage ECGPageBuf2_data;
//extern unsigned char ECGRecorder_ACQdata_Buf[128];
extern unsigned char Store_data_rdy;
//unsigned char bankFlag = 0;
//unsigned char ECG_recorderReadyFlag = 0;
//unsigned short sampleCount = 0;

#define DELAY_COUNT 2


unsigned char ads1292r_data_buff[9];

/****************************************************************/
/*    SPI define    */
/****************************************************************/
#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

#define TEST_STRING "Nordic"
//extern static uint8_t       m_tx_buf[16];// = {0x40, 0x1, 0x0, 0x0};           /**< TX buffer. */      //not use this one
//extern static uint8_t       m_rx_buf[16];    /**< RX buffer. */                 //not use this one
static const uint8_t m_length = sizeof(SPI_Tx_buf);        /**< Transfer length. */


//volatile bool new_data = false;   //yonghun 2020 06/15    add and erase










/* ADS1x9x Register values*/

unsigned char 	ADS1x9xRegVal[16] = {

    //Device ID read Ony
    0x00,
    //CONFIG1
    0x02,
    //CONFIG2
    0xE0,
    //LOFF
    0xF0,
    //CH1SET (PGA gain = 6)
    0x00,
    //CH2SET (PGA gain = 6)
    0x00,
    //RLD_SENS (default)
    0x2C,
    //LOFF_SENS (default)
    0x0F,    
    //LOFF_STAT
    0x00,
    //RESP1
    0xEA,
    //RESP2
    0x03,
    //GPIO
    0x0C 
};		
unsigned char 	ADS1x9xR_Default_Register_Settings[15] = {

    //Device ID read Ony
    0x00,
    //CONFIG1
    0x02,
    //CONFIG2
    0xE0,
    //LOFF
    0xF0,
    //CH1SET (PGA gain = 6)
    0x00,
    //CH2SET (PGA gain = 6)
    0x00,
    //RLD_SENS (default)
    0x2C,
    //LOFF_SENS (default)
    0x0F,    
    //LOFF_STAT
    0x00,
    //RESP1
    0xEA,
    //RESP2
    0x83,
    //GPIO
    0x0C 
};	
//now use test
unsigned char 	ADS1292R_Register_Settings_yonghun[15] = {

    //Device ID read Ony
    0x00, //fix 
    //CONFIG1
    0x02, //fix
    //CONFIG2
    0xA3, //fix
    //LOFF
    0x10, //fix
    //CH1SET (PGA gain = 6)
    0x80, //fix(not use ch1)
    //CH2SET (PGA gain = 6)
    0x05, //0x00, // fix(use ch2 for read ecg)
    //RLD_SENS (default)
    0x00, //  fix
    //LOFF_SENS (default)
    0x00, //fix
    //LOFF_STAT
    0x00, //fix
    //RESP1
    0x02,
    //RESP2
    0x03,
    //GPIO
    0x0C 
};
	
unsigned char 	ADS1x9x_Default_Register_Settings[15] = {

    //Device ID read Ony
    0x00,
    //CONFIG1
    0x02,
    //CONFIG2
    0xE0,
    //LOFF
    0xF0,
    //CH1SET (PGA gain = 6)
    0x00,
    //CH2SET (PGA gain = 6)
    0x00,
    //RLD_SENS (default)
    0x2C,
    //LOFF_SENS (default)
    0x0F,    
    //LOFF_STAT
    0x00,
    //RESP1
    0x02,
    //RESP2
    0x03,
    //GPIO
    0x0C 
};	
	

/***********************************************************************************************************
*	        Variables for the SPI Interaction                                                          	   *
***********************************************************************************************************/


/**********************************************************************************************************
*	External Variables																			          *
**********************************************************************************************************/


/**********************************************************************************************************
* ADS1x9x_Clock_Select																					  *
* Input : 0 - external																					  *
*       : 1 - internal					                                         						  *
**********************************************************************************************************/

//we can't control this pin, because no connected
/*
void ADS1x9x_Clock_Select(unsigned char clock_in)    //P2out = clk pin (maybe)   //we use internal clock    //no define
  {
  	
    if (clock_in == 1)
    {
        P2OUT |= (enum PORT2_ADC_CONTROL)ADC_CLK_SEL;	// Choose internal clock input    //OR calculate -> assign, P2OUT no declare
    }
    else
    {
        P2OUT &= ~(enum PORT2_ADC_CONTROL)ADC_CLK_SEL;	// Choose external clock input
    }
  	
}
*/

/**********************************************************************************************************
* pin define function			 			                                         						  	  *
**********************************************************************************************************/
void Pin_Define(void)    //P8out = reset pin (maybe)
{
    nrf_gpio_cfg_output(ADS_CSN_PIN);
    nrf_gpio_cfg_output(ADS_RST_PIN);   //by yonghun 2020 06/15
    nrf_gpio_pin_write(ADS_RST_PIN, 0);
    nrf_gpio_cfg_output(ADS_START_PIN);   //by yonghun 2020 06/15
    nrf_gpio_pin_write(ADS_START_PIN, 0);
}

/**********************************************************************************************************
* ADS1x9x_Reset			 			                                         						  	  *
**********************************************************************************************************/
void ADS1x9x_Reset(void)    //P8out = reset pin (maybe)
{
    unsigned short i;
    
    nrf_gpio_pin_set(ADS_RST_PIN);   //by yonghun 2020 06/15
    //P8OUT |= (enum PORT8_ADC_CONTROL)ADC_RESET;		// Set High     //declare port: no declare
    /* Provide suficient dealy*/
    for(i= 0;	i < 5000; i++);						// Wait 1 mSec
    
    nrf_gpio_pin_clear(ADS_RST_PIN);
    //P8OUT &= ~(enum PORT8_ADC_CONTROL)ADC_RESET;	// Set to low  
    for(i= 0;	i < 5000; i++);					    // Wait 1 mSec
  
    nrf_gpio_pin_set(ADS_RST_PIN);
    //P8OUT |= (enum PORT8_ADC_CONTROL)ADC_RESET;		// Set High
    for(i= 0;	i < 35000; i++);

}
  
/**********************************************************************************************************
* ADS1x9x_Disable_Start						                                          					  *
**********************************************************************************************************/
void ADS1x9x_Disable_Start(void)      //P8out = reset pin (maybe) I don't know where declare
{
    unsigned short i;
    
    nrf_gpio_pin_write(ADS_START_PIN, 0);   //by yonghun 2020 06/15
    //P8OUT &= ~(enum PORT8_ADC_CONTROL)ADC_START;	// Set to LOW
    for(i=0; i<35000; i++);        					// Small Delay to settle   
}
/*********************************************************************************************************/
/**********************************************************************************************************
* ADS1x9x_Enable_Start						                                          					  *
**********************************************************************************************************/
void ADS1x9x_Enable_Start(void)
{
    unsigned short i;
   
    nrf_gpio_pin_write(ADS_START_PIN, 1);   //by yonghun 2020 06/15
    //P8OUT |= (enum PORT8_ADC_CONTROL)ADC_START;		// Set to High
    for(i=0; i<50000; i++);        					// Small Delay to settle   
}
/*********************************************************************************************************/
/**********************************************************************************************************
* Set_ADS1x9x_Chip_Enable																                  *
**********************************************************************************************************/
void Set_ADS1x9x_Chip_Enable (void)       //cs low -> mean we can use this pin
{
    /* ADS1x9x CS is Active low*/

    nrf_gpio_pin_write(ADS_CSN_PIN, 0);   //by yonghun 2020 06/15
    //P8OUT &= ~(enum PORT8_ADC_CONTROL)ADC_CS;		// Set to LOW
}
/**********************************************************************************************************
* Clear_ADS1x9x_Chip_Enable						                                          			  *
**********************************************************************************************************/
void Clear_ADS1x9x_Chip_Enable (void)       //high -> noe yet select
{
    unsigned char CsDelay;
    for ( CsDelay = 0;  CsDelay < 100 ;CsDelay++);
    /* ADS1x9x CS is Active low*/
    //P8OUT |= (enum PORT8_ADC_CONTROL)ADC_CS;		// Set to High

    nrf_gpio_pin_write(ADS_CSN_PIN, 1);   //by yonghun 2020 06/15
}



//put DRDY function -> main.c   //yonghun 2020 06/15
/**********************************************************************************************************
* Init_ADS1x9x_DRDY_Interrupt       //(sensor -> MCU)												                                          *
**********************************************************************************************************/
/*
void Init_ADS1x9x_DRDY_Interrupt (void)           //we must know pin name(each pin)   //P1: DRDY pin    //must fine this pin name
{
	
    P1DIR &= ~0x02;
    P1REN |= BIT1;                            	// Enable P1.1 internal resistance// BIT1 = 0x0002
    P1OUT |= BIT1;                            	// Set P1.1 as pull-Up resistance
    P1IES |= BIT1;                           		// P1.1 Lo/Hi edge
    P1IFG &= ~BIT1;                           	// P1.1 IFG cleared
    P1IE &= ~BIT1;                             	// P1.1 interrupt disabled
	
}
*/
/**********************************************************************************************************
* Enable_ADS1x9x_DRDY_Interrupt												                                          *
**********************************************************************************************************/
/*
void Enable_ADS1x9x_DRDY_Interrupt (void)
{
    P1IFG &= ~BIT1;                           	// P1.1 IFG cleared
    P1IE |= BIT1;                             	// P1.1 interrupt enabled
}
*/
/**********************************************************************************************************
* Disable_ADS1x9x_DRDY_Interrupt												                                          *
**********************************************************************************************************/
/*
void Disable_ADS1x9x_DRDY_Interrupt (void)
{
    P1IFG &= ~BIT1;                           	// P1.1 IFG cleared
    P1IE &= ~BIT1;                             	// P1.1 interrupt disabled
}
*/





//we must know pin number and name(each pins)



/**********************************************************************************************************
* Set_GPIO	//not use 													                                          *
**********************************************************************************************************/
/*
void Set_GPIO(void)
{
	
    P2SEL = 0x00;                            
    P2DIR |= 0x8F;
    P2OUT |= (enum PORT2_ADC_CONTROL)POW_CE;                            
    P8DIR |= 0x07;       
    P8OUT &= 0xF8;
    P8OUT |= (enum PORT8_ADC_CONTROL)ADC_CS;		// Set RESET, START to Low and CS to High
    P8OUT |= (enum PORT8_ADC_CONTROL)ADC_RESET;	// Set RESET, START to Low and CS to High
    P2OUT = 0x03;
    //dataCnt = 0;

}  
*/


/**********************************************************************************************************
*	        NRF52 spi event handler          				                  					  *
**********************************************************************************************************/
/*
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
//    NRF_LOG_INFO("Transfer completed.");
//    NRF_LOG_INFO(" Received:");
//    NRF_LOG_HEXDUMP_INFO(m_rx_buf, 16);
    
}
*/


//put SPI function -> main.c   //yonghun 2020 06/15
/**********************************************************************************************************
* Set_UCB0_SPI		//(SPI)	//maybe we use this one		//yonghun 2020 06/15    //maybe we do it in here                                     *
**********************************************************************************************************/
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
//    NRF_LOG_INFO("Transfer completed.");
//    NRF_LOG_INFO(" Received:");
//    NRF_LOG_HEXDUMP_INFO(m_rx_buf, 16);
    
}

void Set_NRF_SPI(void)
{
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    // spi_config.ss_pin = 7;
    spi_config.ss_pin = ADS_CSN_PIN;
    spi_config.miso_pin = ADS_MISO_PIN;
    spi_config.mosi_pin = ADS_MOSI_PIN;
    spi_config.sck_pin = ADS_SCK_PIN;
    spi_config.mode = NRF_DRV_SPI_MODE_1;   //check this mode with ADS1292 datasheet, yonghun 2020 6/9
    
    // spi_config.frequency = NRF_DRV_SPI_FREQ_500K;
    spi_config.frequency = NRF_DRV_SPI_FREQ_250K;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));

    nrf_gpio_pin_clear(ADS_RST_PIN);

    
    SPI_Tx_buf[0] = 0x40;
    SPI_Tx_buf[1] = 0x1;
    SPI_Tx_buf[2] = 0;
    /*
    P3SEL |= BIT2+BIT1+BIT0;  				// Set SPI peripheral bits
    P3DIR |= BIT0+BIT2;						// Clock and DOUT as output
    P3DIR &= ~BIT1;                         	// Din as input 
    UCB0CTL1 |= UCSWRST;               		// Enable SW reset
    UCB0CTL0 |= UCMSB+UCMST+UCSYNC;			//[b0]   1 -  Synchronous mode 
												//[b2-1] 00-  3-pin SPI
												//[b3]   1 -  Master mode
												//[b4]   0 - 8-bit data
												//[b5]   1 - MSB first
												//[b6]   0 - Clock polarity low.
												//[b7]   1 - Clock phase - Data is captured on the first UCLK edge and changed on the following edge.

    UCB0CTL1 |= UCSSEL__ACLK;               	// ACLK
    UCB0BR0 = 24;                             // 1 MHz
    UCB0BR1 = 0;                              //
    UCB0CTL1 &= ~UCSWRST;              		// Clear SW reset, resume operation
    */
}  

/**********************************************************************************************************
* Set_DMA_SPI		//????maybe check this one //maybe not use this one														                                          *
**********************************************************************************************************/
/*
void Set_DMA_SPI(void)
{
    DMACTL0 = DMA0TSEL_12;                 			// USCI_B0 Transmit Ready Trigger
    //DMA0SA = (void (*)())&UCB0RXBUF;       			// Source block address
    //DMA0DA = (void (*)())ADC_Read_data;    			// Destination single address
    DMA0SZ = 16;                           			// Block size
    DMA0CTL = DMADT_4 + DMADSTINCR_3 + DMADSTBYTE + DMASRCBYTE;
                                         			// Rpt, inc src, byte-byte
    DMA0CTL |= DMAEN;                      			// Enable DMA for consecutive Xfers
  
    
}
*/


//put SPI COMMAND function -> main.c   //yonghun 2020 06/15
/**********************************************************************************************************
* ADS1x9x_SPI_Command_Data						                                          *
**********************************************************************************************************/

void ADS1x9x_SPI_Command_Data(unsigned char Data)         //problem in here we can change this function
{
    unsigned char delayVar;
    Set_ADS1x9x_Chip_Enable();    //cs low
    for (delayVar = 0; delayVar < 50; delayVar++);
    Clear_ADS1x9x_Chip_Enable();    //cs high
    Set_ADS1x9x_Chip_Enable();    //cs low


    memset(SPI_Tx_buf, 0, 3);
    memset(SPI_Rx_buf, 0, 9);

    spi_xfer_done = false;
    SPI_Tx_buf[0] = Data;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, SPI_Tx_buf, 1, SPI_Rx_buf, 0));
    
    
    while (!spi_xfer_done);
    
	
    //UCB0TXBUF = Data;                                     // Send the data sitting at the pointer DATA to the TX Buffer   //spi communication
    //while ( (UCB0STAT & UCBUSY) );

    //delayVar = UCB0RXBUF;
    //delayVar = SPI_Rx_buf;      //something delay proble in here
    Clear_ADS1x9x_Chip_Enable();    //cs high

    for (delayVar = 0; delayVar < 150; delayVar++);

}

/**********************************************************************************************************
* Init_ADS1x9x_Resource		//I don't know this function 2020 06/13 yonghun				                                          *
**********************************************************************************************************/

void Init_ADS1x9x_Resource(void)
{
    //Set_GPIO();	//erase yonghun 2020 06/13									// Initializes ADS1x9x's input control lines
    Set_NRF_SPI();									// Initialize SPI regs.
    //Set_DMA_SPI();	   								// Initialize DMA regs for SPI.
    
}




/**********************************************************************************************************
*	        ADS1x9x Control Registers      				                                  *
**********************************************************************************************************/
/**********************************************************************************************************
* Wake_Up_ADS1x9x						                                          						  *
**********************************************************************************************************/
void Wake_Up_ADS1x9x (void)
{ 
    ADS1x9x_SPI_Command_Data (COMMAND_WAKEUP);                   // Send 0x02 to the ADS1x9x //after "4t_clk" -> we can send another command                                                     
}

/**********************************************************************************************************
* Put_ADS1x9x_In_Sleep						                                          					  *
**********************************************************************************************************/
void Put_ADS1x9x_In_Sleep (void)
{
    ADS1x9x_SPI_Command_Data (COMMAND_STANDBY);                 // Send 0x04 to the ADS1x9x   //after this command -> only send "wake up(0x02)" command
}
/**********************************************************************************************************
* Soft_Reset_ADS1x9x					                                          						  *
**********************************************************************************************************/
void Soft_Reset_ADS1x9x (void)
{
    ADS1x9x_SPI_Command_Data (COMMAND_RESET);                   // Send 0x06 to the ADS1x9x//takes 9f_MOD cycles, reset digital filter and all register value to default
}
/**********************************************************************************************************
* Soft_Start_ReStart_ADS1x9x			                                          						  *
**********************************************************************************************************/
void Soft_Start_ReStart_ADS1x9x (void)
{
    ADS1x9x_SPI_Command_Data(COMMAND_START);                  // Send 0x08 to the ADS1x9x//before stop command, it works.., remain start pin low state to use this command
    Clear_ADS1x9x_Chip_Enable ();                                                       
}
/**********************************************************************************************************
* Hard_Start_ReStart_ADS1x9x			                                          						  *
**********************************************************************************************************/
void Hard_Start_ReStart_ADS1x9x(void)
{
    //P8OUT |= (enum PORT8_ADC_CONTROL)ADC_START;			// Set Start pin to High//use pin state high to start//not use command
    
    nrf_gpio_pin_write(ADS_START_PIN, 1);   //by yonghun 2020 06/15
}

/**********************************************************************************************************
* Soft_Start_ADS1x9x					                                          						  *
**********************************************************************************************************/
//maybe not use this function
void Soft_Start_ADS1x9x (void)
{
    ADS1x9x_SPI_Command_Data(COMMAND_START);                   // Send 0x0A???????? to the ADS1x9x
}


/**********************************************************************************************************
* Soft_Stop_ADS1x9x					                                          						  *
**********************************************************************************************************/
void Soft_Stop_ADS1x9x (void)
{
    ADS1x9x_SPI_Command_Data (COMMAND_STOP);                   // Send 0x0A to the ADS1x9x
}
/**********************************************************************************************************
* read_unit_data_ADS1x9x					                                          						  *
**********************************************************************************************************/
void Read_unit_data_ADS1x9x (void)
{
    ADS1x9x_SPI_Command_Data (COMMAND_STOP);                   // Send 0x0A to the ADS1x9x
}
/**********************************************************************************************************
* Hard_Stop_ADS1x9x					                                          						  *
**********************************************************************************************************/
void Hard_Stop_ADS1x9x (void)
{
    unsigned short i, j;
    //P8OUT &= ~(enum PORT8_ADC_CONTROL)ADC_START;		// Set Start pin to Low   //not use command
    
    nrf_gpio_pin_write(ADS_START_PIN, 0);   //by yonghun 2020 06/15
    for (j = 0; j < DELAY_COUNT; j++)
    {
        for ( i=0; i < 35000; i++);
    }
}


/**********************************************************************************************************
* Start_Read_Data_Continuous			                                          						  *
**********************************************************************************************************/
void Start_Read_Data_Continuous (void)
{
    ADS1x9x_SPI_Command_Data (COMMAND_RDATAC);					// Send 0x10 to the ADS1x9x
}

/**********************************************************************************************************
* Soft_Start_ReStart_ADS1x9x			                                          						  *
**********************************************************************************************************/

void Stop_Read_Data_Continuous (void)
{
    ADS1x9x_SPI_Command_Data(COMMAND_SDATAC);					// Send 0x11 to the ADS1x9x
}

/**********************************************************************************************************
* Start_Data_Conv_Command			                                          						  *
**********************************************************************************************************/
//maybe not use this function
void Start_Data_Conv_Command (void)
{
    ADS1x9x_SPI_Command_Data(COMMAND_START);					// Send 0x08 to the ADS1x9x
}
/**********************************************************************************************************
* Start_Data_Conv_Command			                                          						  *
**********************************************************************************************************/
//maybe not use this function
void Channel_offset_Command (void)
{
    ADS1x9x_SPI_Command_Data(COMMAND_OFFSETCAL);					// Send 0x08 to the ADS1x9x
}
/**********************************************************************************************************
* Read data by command; supports multiple read back.			                                          						  *
**********************************************************************************************************/
//maybe not use this function
void Read_One_Data (void)
{
    ADS1x9x_SPI_Command_Data(COMMAND_RDATA);					// Send 0x08 to the ADS1x9x
}


//finish setting command (not include 0x12(RDATA)   0x2x(RREG)   0x4x(WREG))




/**********************************************************************************************************
* Initialize ADS1x9x						                                          *
**********************************************************************************************************/
void Init_ADS1x9x (void)
{
	ADS1x9x_Reset();
	ADS1x9x_Disable_Start();
	ADS1x9x_Enable_Start();
}
/*********************************************************************************************************/

/**********************************************************************************************************
* enable_ADS1x9x_Conversion													                          *
**********************************************************************************************************/
void enable_ADS1x9x_Conversion (void)
  {
    Start_Read_Data_Continuous ();		//RDATAC command
    
    Hard_Start_ReStart_ADS1x9x();               //start by pin state high

  }
/*********************************************************************************************************/








/*********************************************************************************************************
* ADS1x9x_Reg_Write																	                 *
**********************************************************************************************************/
void ADS1x9x_Reg_Write (unsigned char READ_WRITE_ADDRESS, unsigned char DATA)
{ 
    short i;
    switch (READ_WRITE_ADDRESS)
    {
        case 1:             //what mean "case 1" 2020 06/13 yonghun
            DATA = DATA & 0x87;
            break;

        case 2:
            DATA = DATA & 0xFB;
            DATA |= 0x80;	
            break;

        case 3:
            DATA = DATA & 0xFD;
            DATA |= 0x10;
            break;

        case 7:
            DATA = DATA & 0x3F;
            break;

        case 8:
            DATA = DATA & 0x5F;
            break;

        case 9:
            DATA |= 0x02;
            break;

        case 10:
            DATA = DATA & 0x87;
            DATA |= 0x01;
            break;

        case 11:
            DATA = DATA & 0x0F;
            break;
  		
        default:
            break;
  		
    }
    //define DATA value
    memset(SPI_Tx_buf, 0, 3);
    memset(SPI_Rx_buf, 0, 9);
    spi_xfer_done = false;

    SPI_Tx_buf[0] = READ_WRITE_ADDRESS | 0x40;        //SPI_Tx_buf -> 1byte, 10 array  //WREG=write to register
    SPI_Tx_buf[1] = 0;						// Write Single byte
    SPI_Tx_buf[2] = DATA;					// Write Single byte

    Set_ADS1x9x_Chip_Enable();
	
    for ( i =0; i < 50;i++);        //to delay
        /*
	UCB0TXBUF = SPI_Tx_buf[0];              // Send the first data to the TX Buffer
 	while ( (UCB0STAT & UCBUSY) );			// USCI_B0 TX buffer ready?
	i = UCB0RXBUF;							// Read Rx buf

	UCB0TXBUF = SPI_Tx_buf[1];              // Send the first data to the TX Buffer
	while ( (UCB0STAT & UCBUSY) );			// USCI_B0 TX buffer ready?
	i = UCB0RXBUF;
	UCB0TXBUF = SPI_Tx_buf[2];              // Send the first data to the TX Buffer
	while ( (UCB0STAT & UCBUSY) );			// USCI_B0 TX buffer ready?
	i = UCB0RXBUF;
        */
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, SPI_Tx_buf, 3, SPI_Rx_buf, 0));

    while (!spi_xfer_done);


        //must check UCB0RXBUF 2020 06/13 yonghun
}
/*********************************************************************************************************
* ADS1x9x_Reg_Read																	                 *
**********************************************************************************************************/
unsigned char ADS1x9x_Reg_Read(unsigned char Reg_address)
{
    unsigned char retVal;
    memset(SPI_Tx_buf, 0, 3);
    memset(SPI_Rx_buf, 0, 9);
    spi_xfer_done = false;
    SPI_Tx_buf[0] = 0x20 | Reg_address;               //SPI_Tx_buf -> 1byte, 10 array  //WREG=write to register
    SPI_Tx_buf[1] = 0x01;							// Read number of bytes - 1
		
    Set_ADS1x9x_Chip_Enable();					// Set chip select to low

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, SPI_Tx_buf, 3, SPI_Rx_buf, 3));
    while (!spi_xfer_done);

		/*
		UCB0TXBUF = SPI_Tx_buf[0];                  // Send the first data to the TX Buffer
		while ( (UCB0STAT & UCBUSY) );				// USCI_B0 TX buffer ready?
		UCB0TXBUF = SPI_Tx_buf[1];                  // Send the first data to the TX Buffer
		while ( (UCB0STAT & UCBUSY) );				// USCI_B0 TX buffer ready?
		retVal = UCB0RXBUF;							// Read RX buff
		UCB0TXBUF = 0x00;                           // Send the first data to the TX Buffer
		while ( (UCB0STAT & UCBUSY) );				// USCI_B0 TX buffer ready?
		retVal = UCB0RXBUF;							// Read RX buff
                */
    
    retVal = SPI_Rx_buf[2];

    return retVal;
                        //must check UCB0RXBUF 2020 06/13 yonghun
}








/**********************************************************************************************************
*	        ADS1x9x default Initialization          				                  					  *
**********************************************************************************************************/
void ADS1x9x_Default_Reg_Init(void)
{

    unsigned char Reg_Init_i;
    Set_ADS1x9x_Chip_Enable();    //cs low
    for ( Reg_Init_i =0; Reg_Init_i <100;Reg_Init_i++);
    Clear_ADS1x9x_Chip_Enable();    //cs high(not select)
    Set_ADS1x9x_Chip_Enable();    //cs low
    for ( Reg_Init_i =0; Reg_Init_i <100;Reg_Init_i++);
        //acess success    by yh 2020 06/16 
	
    if ((ADS1292R_Register_Settings_yonghun[0] | 0X20) == 0x20)    //change this code to read register correctly
    {
        for ( Reg_Init_i = 1; Reg_Init_i < 12; Reg_Init_i++)
        {
            ADS1x9x_Reg_Write(Reg_Init_i,ADS1292R_Register_Settings_yonghun[Reg_Init_i]);
        }
    }
    else
    {
        for ( Reg_Init_i = 1; Reg_Init_i < 12; Reg_Init_i++)
        {
            ADS1x9x_Reg_Write(Reg_Init_i,ADS1x9x_Default_Register_Settings[Reg_Init_i]);
        }
    }

}

/**********************************************************************************************************
*	        ADS1x9x_Read_All_Regs          				                  					  *
**********************************************************************************************************/

void ADS1x9x_Read_All_Regs(unsigned char ADS1x9xeg_buf[])
{
	unsigned char Regs_i;
	Set_ADS1x9x_Chip_Enable();      //cs low  
	for ( Regs_i =0; Regs_i <200;Regs_i++);
	Clear_ADS1x9x_Chip_Enable();    //cs high
        for ( Regs_i =0; Regs_i <200;Regs_i++);
	Set_ADS1x9x_Chip_Enable();      //cs low  
	for ( Regs_i =0; Regs_i <200;Regs_i++);

	for ( Regs_i = 0; Regs_i < 12; Regs_i++)
	{
		ADS1x9xeg_buf[Regs_i] = ADS1x9x_Reg_Read(Regs_i);   //change cs pin from high to low
                printf("\r\n%d:,   %x\n\r", Regs_i, ADS1x9xeg_buf[Regs_i]);

	}

}
/*********************************************************************************************************/
/**********************************************************************************************************
*	        ADS1x9x_Read_All_Regs          				                  					  *
**********************************************************************************************************/

void ADS1x9x_Read_ID_Regs(unsigned char ads_regs)
{
	unsigned char Regs_i;
	Set_ADS1x9x_Chip_Enable();      //cs low  
	for ( Regs_i =0; Regs_i <200;Regs_i++);
	Clear_ADS1x9x_Chip_Enable();    //cs high
        for ( Regs_i =0; Regs_i <200;Regs_i++);
	Set_ADS1x9x_Chip_Enable();      //cs low  
	for ( Regs_i =0; Regs_i <200;Regs_i++);

	Regs_i = ADS1x9x_Reg_Read(ads_regs);   //change cs pin from high to low
        printf("\r\nID: %x\n\r", Regs_i);
}
/*********************************************************************************************************/
/**********************************************************************************************************
*	        ADS1x9x_PowerOn_Init          				                  					  			  *
***********************************************************************************************************/
void ADS1x9x_PowerOn_Init(void)
{
    Init_ADS1x9x_Resource();      //setting SPI
    printf("\r\nSPI example started.\n\r");

    volatile unsigned short Init_i, j;

    ADS1x9x_Reset();                  //reset pin high state

    for (j = 0; j < DELAY_COUNT; j++)         //DELAY_COUNT = 2
    {
        for ( Init_i =0; Init_i < 20000; Init_i++);
        for ( Init_i =0; Init_i < 20000; Init_i++);
        for ( Init_i =0; Init_i < 20000; Init_i++);
    }       //delay
   //Init_ADS1x9x_DRDY_Interrupt();

   //ADS1x9x_Clock_Select(1);		// Set internal clock   //yonghun  2020 06/15 erase

    for ( Init_i =0; Init_i < 20000; Init_i++);    //delay
    for ( Init_i =0; Init_i < 20000; Init_i++);    //delay
    for ( Init_i =0; Init_i < 20000; Init_i++);    //delay
    //ADS1x9x_Disable_Start();        //start pin 0 state
    //ADS1x9x_Enable_Start();         //start pin 1 state

    //Hard_Stop_ADS1x9x();            //start pin 0 state
   
    //Start_Data_Conv_Command();      //start by command
    //printf("\r\n1_2\r\n");
   
    //Soft_Stop_ADS1x9x();            //stop by command
    //nrf_delay_ms(100);
    //printf("\r\n1_2_1\r\n");
    
    /*
    for (j = 0; j < DELAY_COUNT; j++)
    {
        for ( Init_i =0; Init_i < 20000; Init_i++);
    }
    */
    


    //for ( Init_i =0; Init_i < 40000; Init_i++);   //error in here

    Stop_Read_Data_Continuous();					// SDATAC command
    nrf_delay_ms(100);

    /*
    for (j = 0; j < DELAY_COUNT; j++)
    {
        for ( Init_i =0; Init_i < 35000; Init_i++);
    }
    for (j = 0; j < DELAY_COUNT; j++)
    {
        for ( Init_i =0; Init_i < 35000; Init_i++);
    }
    */

    //nrf_delay_ms(20);

    //ADS1x9x_Enable_Start();         //start pin 1 state   //put by yonghun  2020 06/15

//erase by yh 2020 06/17
    //Asserts start bit
    //ADS1x9x_SPI_Command_Data(COMMAND_START);
    //nrf_delay_ms(2000);

    //Start Read data Continouously
    //ADS1x9x_SPI_Command_Data(COMMAND_RDATAC);
    //nrf_delay_ms(2000);

    //ADS1x9x_SPI_Command_Data(COMMAND_OFFSETCAL);
    //printf("\r\nRunning...\n\r");


    //printf("\r\n1_3_2\r\n");
    //ADS1x9x_Read_All_Regs(ADS1x9xRegVal);
    //ADS1x9x_Default_Reg_Init();
    //ADS1x9x_Read_All_Regs(ADS1x9xRegVal);
    //printf("\r\n1_4\r\n");



}
//init function we must use this function 2020 06/13 yonghun



/*********************************************************************************************************
//not use this function
**********************************************************************************************************/
void ADS1191_Parse_data_packet(void)    //to pasing_ADS1191 
{
	unsigned char ECG_Chan_num;

	switch (ECG_Recoder_state.state)
	{

       case IDLE_STATE:
       break;
       case DATA_STREAMING_STATE:
       {
      		for (ECG_Chan_num = 0; ECG_Chan_num < 2; ECG_Chan_num++)
      		{
	      		ADS1x9x_ECG_Data_buf[ECG_Chan_num] = (signed long)SPI_Rx_buf[2*ECG_Chan_num]; 	// Get MSB 8 bits 
	      		ADS1x9x_ECG_Data_buf[ECG_Chan_num] = ADS1x9x_ECG_Data_buf[ECG_Chan_num] << 8;
	      		ADS1x9x_ECG_Data_buf[ECG_Chan_num] |= SPI_Rx_buf[2*ECG_Chan_num+1];				// Get LSB 8 bits
      		}
      		ADS1x9x_ECG_Data_buf[0] = ADS1x9x_ECG_Data_buf[0] << 8;								// to make compatable with 24 bit devices
       }
       break;

        
       case ACQUIRE_DATA_STATE:

       case ECG_RECORDING_STATE:
		{
   			unsigned char *ptr;
			ptr = &ECGRecorder_data_Buf[Recorder_head << 3]; // Point to Circular buffer at head*8;
			*ptr++ = SPI_Rx_buf[0];				// Store status 
			*ptr++ = SPI_Rx_buf[1];				// Store status 
//			if ((SPI_Rx_buf[2] & 0x80 ) == 0x80)// CH0[15-8] = MSB ( 16Bit device)
//			*ptr++ = 0xFF;						// CH0[23-16] = 0xFF ( 16Bit device) sign
//			else
//			*ptr++ = 0;							// CH0[23-16] = 0 ( 16Bit device)
			*ptr++ = SPI_Rx_buf[2];				// CH0[15-8] = MSB ( 16Bit device)
			*ptr++ = SPI_Rx_buf[3];				// CH0[7-0] = LSB ( 16Bit device)
			*ptr++ = 0;
//			if ((SPI_Rx_buf[2] & 0x80 ) == 0x80)// CH0[15-8] = MSB ( 16Bit device)
//			*ptr++ = 0xFF;						// CH0[23-16] = 0xFF ( 16Bit device) sign
//			else
//			*ptr++ = 0;							// CH0[23-16] = 0 ( 16Bit device)
			*ptr++ = SPI_Rx_buf[2];				// CH0[15-8] = MSB ( 16Bit device)
			*ptr++ = SPI_Rx_buf[3];				// CH0[7-0] = LSB ( 16Bit device)
			*ptr++ = 0;
			Recorder_head ++;					// Increment Circuler buffer pointer
			
			if (Recorder_head == 32)			// Check for circuler buffer depth.
				Recorder_head = 0;				// Rest once it reach to MAX
		}
            break;
       
       default:
            break;

	}

}


/*********************************************************************************************************
//not use this function
**********************************************************************************************************/
void ADS1192_Parse_data_packet(void)
{
	unsigned char ECG_Chan_num;
	
	switch (ECG_Recoder_state.state)
	{

       case IDLE_STATE:
       break;
       case DATA_STREAMING_STATE:
       {
      		for (ECG_Chan_num = 0; ECG_Chan_num < 3; ECG_Chan_num++)
      		{
	      		ADS1x9x_ECG_Data_buf[ECG_Chan_num] = (signed long)SPI_Rx_buf[2*ECG_Chan_num];	// Get MSB Bits15-bits8
	      		ADS1x9x_ECG_Data_buf[ECG_Chan_num] = ADS1x9x_ECG_Data_buf[ECG_Chan_num] << 8;
	      		ADS1x9x_ECG_Data_buf[ECG_Chan_num] |= SPI_Rx_buf[2*ECG_Chan_num+1];				// Get LSB Bits7-bits0
      		}
      		ADS1x9x_ECG_Data_buf[0] = ADS1x9x_ECG_Data_buf[0] << 8;				// to make compatable with 24 bit devices
       }
	   
       break;

        
       case ACQUIRE_DATA_STATE:

       case ECG_RECORDING_STATE:
		{
   			unsigned char *ptr;
   			
			ptr = &ECGRecorder_data_Buf[Recorder_head << 3]; // Point to Circular buffer at head*8;
			*ptr++ = SPI_Rx_buf[0];				// Store status 
			*ptr++ = SPI_Rx_buf[1];				// Store status 

//			if ((SPI_Rx_buf[2] & 0x80 ) == 0x80)// CH0[15-8] = MSB ( 16Bit device)
//			*ptr++ = 0xFF;						// CH0[23-16] = 0xFF ( 16Bit device) sign
//			else
//			*ptr++ = 0;							// CH0[23-16] = 0 ( 16Bit device)
			*ptr++ = SPI_Rx_buf[2];				// CH0[15-8] = MSB ( 16Bit device)
			*ptr++ = SPI_Rx_buf[3];				// CH0[7-0] = LSB ( 16Bit device)
			*ptr++ = 0;

//			if ((SPI_Rx_buf[4] & 0x80 ) == 0x80)// CH1[15-8] = MSB ( 16Bit device)
//			*ptr++ = 0xFF;						// CH1[23-16] = 0xFF ( 16Bit device) sign
//			else
//			*ptr++ = 0;							// CH1[23-16] = 0 ( 16Bit device)
			*ptr++ = SPI_Rx_buf[4];				// CH1[15-8] = MSB ( 16Bit device)
			*ptr++ = SPI_Rx_buf[5];				// CH1[7-0] = LSB ( 16Bit device)
			*ptr++ = 0;
			Recorder_head ++;					// Increment Circuler buffer pointer
			
			if (Recorder_head == 32)			// Check for circuler buffer depth.
				Recorder_head = 0;				// Rest once it reach to MAX
		}
            break;
       
       default:
            break;

	}

}


/*********************************************************************************************************
//not use this function
**********************************************************************************************************/
void ADS1291_Parse_data_packet(void)
{
	unsigned char ECG_Chan_num;

	switch (ECG_Recoder_state.state)
	{		
       case DATA_STREAMING_STATE:
       {
      		for (ECG_Chan_num = 0; ECG_Chan_num < 2; ECG_Chan_num++)
      		{
	      		ADS1x9x_ECG_Data_buf[ECG_Chan_num] = (signed long)SPI_Rx_buf[3*ECG_Chan_num];	// Get Bits23-bits16
	      		ADS1x9x_ECG_Data_buf[ECG_Chan_num] = ADS1x9x_ECG_Data_buf[ECG_Chan_num] << 8;
	      		ADS1x9x_ECG_Data_buf[ECG_Chan_num] |= SPI_Rx_buf[3*ECG_Chan_num+1];				// Get Bits15-bits8
	      		ADS1x9x_ECG_Data_buf[ECG_Chan_num] = ADS1x9x_ECG_Data_buf[ECG_Chan_num] << 8;
	      		ADS1x9x_ECG_Data_buf[ECG_Chan_num] |= SPI_Rx_buf[3*ECG_Chan_num+2];				// Get Bits7-bits0
      		}
       }
       break;

            
       case ACQUIRE_DATA_STATE:

       case ECG_RECORDING_STATE:
   		{
   			unsigned char *ptr;
   			
			ptr = &ECGRecorder_data_Buf[Recorder_head << 3]; // Point to Circular buffer at head*8;

			*ptr++ = SPI_Rx_buf[0];				// Store status 
			*ptr++ = SPI_Rx_buf[1];				// Store status 
			//SPI_Rx_buf[2] is always 0x00 so it is discarded

			*ptr++ = SPI_Rx_buf[3];				// CH0[23-16] = MSB ( 24 Bit device)
			*ptr++ = SPI_Rx_buf[4];				// CH0[15-8] = MID ( 24 Bit device)
			*ptr++ = SPI_Rx_buf[5];				// CH0[7-0] = LSB ( 24 Bit device)

			*ptr++ = SPI_Rx_buf[3];				// CH1[23-16] = Ch0 to mentain uniformality
			*ptr++ = SPI_Rx_buf[4];				// CH1[15-8] =  Ch0 to mentain uniformality
			*ptr++ = SPI_Rx_buf[5];				// CH1[7-0] =  Ch0 to mentain uniformality

			Recorder_head++;					// Increment Circuler buffer pointer
			if (Recorder_head == 32)			// Check for circuler buffer depth.
				Recorder_head = 0;				// Rest once it reach to MAX
   		}
       
        break;
       
       default:
            break;

	}

}


/*********************************************************************************************************
//maybe use this function
**********************************************************************************************************/
void ADS1292x_Parse_data_packet(void)
{
	unsigned char ECG_Chan_num;
	switch (ECG_Recoder_state.state)
	{		
       case DATA_STREAMING_STATE:
       {
      		for (ECG_Chan_num = 0; ECG_Chan_num < 3; ECG_Chan_num++)
      		{
	      		ADS1x9x_ECG_Data_buf[ECG_Chan_num] = (signed long)SPI_Rx_buf[3*ECG_Chan_num];
	      		ADS1x9x_ECG_Data_buf[ECG_Chan_num] = ADS1x9x_ECG_Data_buf[ECG_Chan_num] << 8;
	      		ADS1x9x_ECG_Data_buf[ECG_Chan_num] |= SPI_Rx_buf[3*ECG_Chan_num+1];
	      		ADS1x9x_ECG_Data_buf[ECG_Chan_num] = ADS1x9x_ECG_Data_buf[ECG_Chan_num] << 8;
	      		ADS1x9x_ECG_Data_buf[ECG_Chan_num] |= SPI_Rx_buf[3*ECG_Chan_num+2];
      		}
       }
       break;

            
	   case ACQUIRE_DATA_STATE:

       case ECG_RECORDING_STATE:
   		{
   			unsigned char *ptr;
   			
			ptr = &ECGRecorder_data_Buf[Recorder_head << 3]; // Point to Circular buffer at head*8;
			*ptr++ = SPI_Rx_buf[0];				// Store status 
			*ptr++ = SPI_Rx_buf[1];				// Store status 
			//SPI_Rx_buf[2] is always 0x00 so it is discarded
			
			*ptr++ = SPI_Rx_buf[3];				// CH0[23-16] = MSB ( 24 Bit device)
			*ptr++ = SPI_Rx_buf[4];				// CH0[15-8] = MID ( 24 Bit device)
			*ptr++ = SPI_Rx_buf[5];				// CH0[7-0] = LSB ( 24 Bit device)
			
			*ptr++ = SPI_Rx_buf[6];				// CH1[23-16] = MSB ( 24 Bit device)
			*ptr++ = SPI_Rx_buf[7];				// CH1[15-8] = MID ( 24 Bit device)
			*ptr++ = SPI_Rx_buf[8];				// CH1[7-0] = LSB ( 24 Bit device)
			
			Recorder_head ++;					// Increment Circuler buffer pointer
			if (Recorder_head == 32)			// Check for circuler buffer depth.
				Recorder_head = 0;				// Rest once it reach to MAX

   		}
       
        break;
       
       default:
       break;
	}
}

/*********************************************************************************************************
**********************************************************************************************************/

void ADS1x9x_Parse_data_packet(void)
{

	switch( ADS1x9xRegVal[0] & 0x03)    //check ID register --> 1292R --> 0111 0011
	{
		case ADS1191_16BIT:
		{
			ADS1191_Parse_data_packet();
		}		
		
		break;
		
		case ADS1192_16BIT:
		{
			ADS1192_Parse_data_packet();
		}		
		
		break;
		
		case ADS1291_24BIT:
		{
			ADS1291_Parse_data_packet();
		}			

		break;
		
		case ADS1292_24BIT:
		{
			ADS1292x_Parse_data_packet();//we use this one!!!!!!!!!!!

		}			
		break;
	}
	ECG_Data_rdy = 1;
	//SPI_Rx_exp_Count = 1;

}
/*********************************************************************************************************/



/**********************************************************************************************************/
// Echo character       //DRDY interrupt change -> drdy_handler
/*
#pragma vector=USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
  switch(__even_in_range(UCB0IV,4))
  {
    case 0:break;                             // Vector 0 - no interrupt
    case 2:                                   // Vector 2 - RXIFG
    
      //while (!(UCB0IFG&UCTXIFG));             // USCI_B0 TX buffer ready?
      	
      	SPI_Rx_buf[SPI_Rx_Count] = UCB0RXBUF;
      	SPI_Rx_Count++;
      	if ( SPI_Rx_Count == SPI_Rx_exp_Count)
      	{
			UCB0IE &= ~UCRXIE;                 // Disable USCI_B0 RX interrupt
			ADS1x9x_Parse_data_packet();
      	}
      	else 
      	{
      		
			UCB0TXBUF = 0; 					// To get Next byte.
      	}
      break;
    case 4:break;                             // Vector 4 - TXIFG

    default: break;
  }
}
// Port 1 interrupt service routine
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{	
	if ( P1IFG &= BIT1)
	{
		P1IFG &= ~BIT1;                 // Clear P1.1 IFG i.e Data RDY interrupt status
		SPI_Rx_Count = UCB0RXBUF; 		// Dummy Read
		SPI_Rx_Count=0;
		
		UCB0TXBUF = 0;
		UCB0IE |= UCRXIE;               // Enable USCI_B0 RX interrupt
	}
}
*/
/*pun this function -> main.c*/
/*
void drdy_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) 
{
    new_data = true;
//  NRF_LOG_DEBUG("Handler");
//  NRF_LOG_FLUSH();

}
*/


/**********************************************************************************************************
* Set_Device_out_bytes						                                          					  *
* 		: Selects number of sample to be recieved from device based device ID							
**********************************************************************************************************/

void Set_Device_out_bytes(void)
{
	switch( ADS1x9xRegVal[0] & 0x03)
	{
		
		case ADS1191_16BIT:
			SPI_Rx_exp_Count=4;		// 2 byte status + 2 bytes CH0 data
		
		break;
		
		case ADS1192_16BIT:	
			SPI_Rx_exp_Count=6;		// 2 byte status + 2 bytes ch1 data + 2 bytes CH0 data
		
		break;
		
		case ADS1291_24BIT:
			SPI_Rx_exp_Count=6;		// 3 byte status + 3 bytes CH0 data

		break;
		
		case ADS1292_24BIT:
			SPI_Rx_exp_Count=9;		// 3 byte status + 3 bytes ch1 data + 3 bytes CH0 data
		
		break;
		
	}
}

int32_t GetData() //32 exchange need, 2020 6/16 by songyonghun//change 6/17
{
    
    int32_t raw_value;
    memset(SPI_Tx_buf, 0, 1);
    memset(SPI_Rx_buf, 0, 9);
    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, SPI_Tx_buf, 0, SPI_Rx_buf, 9));

    while (!spi_xfer_done);

    raw_value = 0x00;
    
    //  when read ch0 -> other person use this code
    /*
    if (SPI_Rx_buf[3] & 0x80) 
    {
        raw_value = 0xff;
    }
    */

    raw_value <<= 8;
    raw_value += SPI_Rx_buf[0];
    raw_value <<= 8;
    raw_value += SPI_Rx_buf[1];
    raw_value <<= 8;
    raw_value += SPI_Rx_buf[2];
    return raw_value;         //4byte = 0x00 + SPI_RX_buf[3] + SPI_RX_buf[4] + SPI_RX_buf[5] //read MSB
}



int32_t ECG_GetData() //32 exchange need, 2020 6/16 by songyonghun//change 6/17
{
    
    uint32_t raw_value;           //change int321 - > uint32
    memset(SPI_Tx_buf, 0, 1);
    memset(SPI_Rx_buf, 0, 9);
    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, SPI_Tx_buf, 0, SPI_Rx_buf, 9));

    while (!spi_xfer_done);

    raw_value = 0x00;
    
    if (SPI_Rx_buf[6] & 0x80 == 0x80) 
    {
        raw_value = 0xff;
    }    
    
    raw_value <<= 8;
    raw_value += SPI_Rx_buf[6];
    raw_value <<= 8;
    raw_value += SPI_Rx_buf[7];
    raw_value <<= 8;
    raw_value += SPI_Rx_buf[8];

    return raw_value;         //4byte = 0x00 + SPI_RX_buf[3] + SPI_RX_buf[4] + SPI_RX_buf[5] //read MSB
}



int32_t RES_GetData() //32 exchange need, 2020 6/16 by songyonghun//change 6/17
{
    
    int32_t raw_value;
    memset(SPI_Tx_buf, 0, 1);
    memset(SPI_Rx_buf, 0, 9);
    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, SPI_Tx_buf, 0, SPI_Rx_buf, 9));

    while (!spi_xfer_done);

    raw_value = 0x00;
    
    //  when read ch0 -> other person use this code
    /*
    if (SPI_Rx_buf[3] & 0x80) 
    {
        raw_value = 0xff;
    }
    */
    raw_value <<= 8;
    raw_value += SPI_Rx_buf[3];
    raw_value <<= 8;
    raw_value += SPI_Rx_buf[4];
    raw_value <<= 8;
    raw_value += SPI_Rx_buf[5];
    return raw_value;         //4byte = 0x00 + SPI_RX_buf[3] + SPI_RX_buf[4] + SPI_RX_buf[5] //read MSB
}




/////// by yonghun 2020 06/22 ///////////
char* ADS1292R_ReadData(void)
{
        memset(SPI_Rx_buf, 0, 9);
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, SPI_Tx_buf, 0, SPI_Rx_buf, 9));
	while(spi_xfer_done == false);

	for(int i=0;i<9;i++)
	{
		ads1292r_data_buff[i]=SPI_Rx_buf[i];
	}
	return ads1292r_data_buff;
}
// End of file
