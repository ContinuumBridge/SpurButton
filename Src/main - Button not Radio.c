/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
#include "cbutils.h"
#include "cbdefines.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//uint8_t aTxBuffer[] = " **** Hello from Galvanize Button **** \r\n";
uint8_t aRxBuffer[128];
uint16_t status;
static uint16_t buttonPressed;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t aTxBuffer[] = " **** Hello from Galvanize Button **** \r\n";
	uint8_t aRxBuffer[128];
	uint16_t  i;

  typedef enum {initial, normal, pressed, search, search_failed, reverting} NodeState;
  NodeState         nodeState           = search;
  uint16_t          bridgeAddress       = 0;
  uint8_t           test2[]             = "Test2\r\n";
  uint8_t           test3[20];
  static uint16_t   destination;
  static uint16_t   source;
  static uint16_t   wakeup;
  static uint8_t    function;
  static uint8_t    length;
  static uint8_t    nodeAddress         = 0;
  static uint8_t    *payload;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  buttonPressed = 0;
  HAL_GPIO_WritePin(GPIOB, HOST_READY_PB14_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RADIO_POWER_PH1_GPIO_Port, RADIO_POWER_PH1_Pin, GPIO_PIN_SET);
  HAL_Delay(2000);
  memcpy(test3, "TestA", 5);
  memcpy(test3 + 5, test2, 7);
  UART_Tx(&huart1, test3, (COUNTOF(test3) - 1));
  UART_Tx(&huart1, (uint8_t*)aTxBuffer, TXBUFFERSIZE);
  DEBUG_TX("Testing DEBUG\r\n");
  GPIO_PinState  radio_busy = HAL_GPIO_ReadPin(GPIOB, RADIO_BUSY_PB12_Pin);
  if (radio_busy == GPIO_PIN_RESET)
	  DEBUG_TX("Radio not busy\r\n");
  else if (radio_busy == GPIO_PIN_SET)
	  DEBUG_TX("Radio busy\r\n");
  else
	  DEBUG_TX("Radio state unknown\r\n");
  HAL_Delay(100);
  __HAL_UART_FLUSH_DRREGISTER(&huart3);
  UART_Tx(&huart3, (uint8_t*)"Hello World", 11);
  DEBUG_TX("Sent Hello World\r\n");
  RADIO_TXS("Radio test", 10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  DEBUG_TX("Type 3 characters\r\n");
	  DEBUG_RX(aRxBuffer, 3);
	  DEBUG_TX("You typed: ");
	  DEBUG_TXS(aRxBuffer, 3);
	  /*
	  if (status == 0)
	  {
		  DEBUG_TX("You typed: ");
		  DEBUG_TXS(aRxBuffer, 3);

	  }
	  else
		  DEBUG_TX("You didn't type anything");
		  */
	  DEBUG_TX("\r\n");
	  /*
	  HAL_Delay(2000);
	  UART_Tx(&huart3, (uint8_t*)"Hello World", 11);
	  //DEBUG_TX("Sent Hello World\r\n");

	  for (i=0; i<64; i++)
		  aRxBuffer[i] = 0;
	  DEBUG_TX("Receiving\r\n");
	  status = RADIO_RX(aRxBuffer, 64);
	  DEBUG_TX("Received from bridge 1: ");
	  DEBUG_TXS(aRxBuffer, 64);
	  DEBUG_TX("\r\n");
	  __HAL_UART_FLUSH_DRREGISTER(&huart3);

	  destination = (aRxBuffer[0] << 8) | aRxBuffer[1];
	  if ((destination == nodeAddress) || (destination == BEACON_ADDRESS) || (destination == GRANT_ADDRESS))
	  {
	    source = (aRxBuffer[2] << 8) | aRxBuffer[3];
	    function = aRxBuffer[4];
	    length = aRxBuffer[5];
	    if (length > 6)
	        wakeup = (aRxBuffer[6] << 8) | aRxBuffer[7];
	    else
	        wakeup = 0;
	    if (length > 8)
	    {
	        payload = aRxBuffer + 8;
	    }
	    if (function == beacon)
	    {
	        //manageSend;
	        if (nodeState == search)
	        {
	            bridgeAddress = source;
	            nodeState = include_req;
	            //sendRadio(include_req, NODE_ID);
	            //setDisplay(connecting);
	        }
	    }
	    else if (function == include_grant)
	    {
	        nodeState = normal;
	        //setDisplay(m1);
	        //onIncludeGrant(payload);
	        //sendRadio(ack);
	    }
	    else if (function == config)
	    {
	        //onConfig(payload);
	        //sendRadio(ack);
	    }
	    else if (function == send_battery)
	    {
	        //sendBattery;
	    }
	    else if (function == ack)
	    {
	        //acknowledged();
	    }
	    else
	    {
	    }
	    //if (function != beacon)
	    //    setWakeup(wakeup);
	  }
	  */

	  //DEBUG_TX("Desination: "); DEBUG_TXS(destination, 2); DEBUG_TX("\r\n");	   */
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_3)
  {
	DEBUG_TX("Button 3 pressed: \r\n");
	buttonPressed = 1;
  }
  else
  {
	DEBUG_TX("Other button pressed: \r\n");
	buttonPressed = 1;
  }
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
