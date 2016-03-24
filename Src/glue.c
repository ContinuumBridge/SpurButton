/****************************************************
 glue.c:    Glue routines, required for portability
 Author:    B.Harris
 Date:      28/01/2016
 ****************************************************/
#include "define.h"
#include "stm32l1xx_hal.h"
#include "spi.h"
#include "usart.h"
#include "cbutils.h"

SPI_HandleTypeDef hspi1;

/**************************************
 misc_sgn - Return sign of an integer
 Author:    B.Harris
 Enters:    Nothing
 Exits:     Nothing
 Date:      13/03/13
 **************************************/
int misc_sgn(int n)
{
  return(n==0? 0:n<0? -1:1);                                                            /* Return sign */
}


/*******************************
 spi_init -     Initialise SPI
 Author:        B.Harris
 Inputs:        Nothing
 Outputs:       Nothing
 Date:          28/01/15
 *******************************/
void spi_init(void)
{
  //static uint8_t initialised;
  // PCX
  //SPI1_Init_Advanced(_SPI_FPCLK_DIV2,_SPI_MASTER|_SPI_8_BIT|_SPI_CLK_IDLE_LOW|_SPI_FIRST_CLK_EDGE_TRANSITION|_SPI_MSB_FIRST|_SPI_SS_DISABLE|_SPI_SSM_ENABLE|_SPI_SSI_1,&_GPIO_MODULE_SPI1_PA567);
  //HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
  MX_SPI1_Init();
  //DEBUG_TX("Initialised SPI\r\n");
}

/*******************************
 spi_deinit -   Deinitialise SPI
 Author:        Peter Claydon
 Inputs:        Nothing
 Outputs:       Nothing
 Date:          2015-02-03
 *******************************/
void spi_deinit()
{
	//HAL_SPI_MspDeInit(&hspi1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    //ECOG_CS_OFF;
    //DEBUG_TX("Deinitialised SPI\r\n");

}

/*************************************************
 spi_hw -   Send to and receive from spi channel
 Author:    B.Harris
 Inputs:    Byte to send to SPI
 Outputs:   Byte received from SPI
 Date:      08/11/13
 *************************************************/
uint8_t spi_hw(uint8_t dat)
{
  // PCX
  //return(SPI1_Read(dat));                                                               /* Return data */
  uint8_t rx = 0xAA;
  //uint8_t tx[2] = {dat, 0};
  SPI_TXRX(&dat, &rx, 1);
  //sprintf(buffer, "spi_hw, wrote: %02X, read %02X\r\n", tx[0], rx[0]);
  //DEBUG_TX(buffer);
  return rx;
}


/**********************************************************
 timers_sdelay -   Delay for given number of milliseconds
 Author:           B.Harris
 Inputs:           Number of milliseconds to delay
 Outputs:          Nothing
 Date:             28/01/16
 **********************************************************/
void timers_sdelay(uint32_t ms)
{
	//while(ms--) delay_ms(1);
	while(ms--) Delay_ms(1);
  //Delay_ms(ms);
}


/**********************************************
 misc_reverse_byte -   Reverse byte bit order
 Author:               B.Harris
 Enters:               Reset cause
 Exits:                Nothing
 Date:                 05/11/15
 **********************************************/
uint8_t misc_reverse_byte(uint8_t byteval)
{
  uint8_t f;                                                                            /* General loop counter */
  uint8_t reversed;                                                                     /* Reversed data */

  for(f=0;f<8;f++)                                                                      /* Loop for all bits */
  {
    reversed<<=1;                                                                       /* Shift result left 1 bit */
    reversed|=(byteval&0x01);                                                           /* Include bit in result */
    byteval>>=1;                                                                        /* Shift data right 1 bit */
  }
  return(reversed);                                                                     /* Return reversed byte */
}
