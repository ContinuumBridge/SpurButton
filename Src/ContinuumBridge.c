/******************************
 main.c:         Main program
 Author:         B.Harris
 Date:           28/01/2016
 ******************************/
#include "define.h"




void main()
{
//#define ECOG_CS_ON               (GPIOA_ODR&=0xffef)
//#define ECOG_CS_OFF              (GPIOA_ODR|=0x0010)
//#define ECOG_DISCHARGE_ON        (GPIOA_ODR|=0x1000)
//#define ECOG_DISCHARGE_OFF       (GPIOA_ODR&=0xefff)
//#define ECOG_RESET_ON            (GPIOB_ODR&=0xffdf)
//#define ECOG_RESET_OFF           (GPIOB_ODR|=0x0020)
//#define ECOG_POWER_ON            (GPIOB_ODR|=0x0040)
//#define ECOG_POWER_OFF           (GPIOB_ODR&=0xffbf)
  SystemInit32();                                                                       /* Start clock up at 32Mhz */
//  UART1_Init(115200);                                                                   /* Initialise USB Uart */
//  UART1_Write_Text("Enabling PORTA clock\r\n");
  GPIO_Clk_Enable(&GPIOA_BASE);                                                         /* Enable clock on port A */
//  UART1_Write_Text("Enabling PORTB clock\r\n");
  GPIO_Clk_Enable(&GPIOB_BASE);                                                         /* Enable clock on port B */
//  UART1_Write_Text("Setting PORTA direction\r\n");
  GPIOA_MODER=(GPIOA_MODER&0xfc3ffcff)|0x01400100;                                      /* Configure inputs/outputs */
//  UART1_Write_Text("Setting PORTB direction\r\n");
  GPIOB_MODER=(GPIOB_MODER&0xffffc3ff)|0x00001400;
//  UART1_Write_Text("Starting main process\r\n");
  process_main();
  while(1);
}