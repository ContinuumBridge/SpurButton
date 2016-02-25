/**
  ******************************************************************************
  * File Name          : cbutils.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 ContinuumBridge
  *
 **/

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>
#include "usart.h"
#include "gpio.h"
#include "cbutils.h"

/* Code */

static int debug_ready   = SET;

__IO ITStatus UartReady = RESET;

void *array_concat(const void *a, size_t an, const void *b, size_t bn, size_t s)
{
  char *p = malloc(s * (an + bn));
  memcpy(p, a, an*s);
  memcpy(p + an*s, b, bn*s);
  return p;
}

/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  UartReady = SET;
}
*/

/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	radio_ready = SET;
	debug_ready = SET;
	if (huart->Instance == USART3)
	{
		radio_ready = SET;
		DEBUG_TX("HAL_UART_RxCpltCallback\r\n");
	}
	else
		debug_ready = SET;
}
*/

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	DEBUG_TX("UART Error Callback\r\n");
}

void Debug_Tx(UART_HandleTypeDef *uart, uint8_t *buffer, uint16_t buffer_size)
{
#ifdef CB_DEBUG
	HAL_UART_Transmit(uart, (uint8_t *)buffer, buffer_size, 50);
#endif
}

extern uint32_t SystemCoreClock;

void DWT_Init(void)
{
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
  {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }
}

uint32_t DWT_Get(void)
{
  return DWT->CYCCNT;
}

__inline
uint8_t DWT_Compare(int32_t tp)
{
  return (((int32_t)DWT_Get() - tp) < 0);
}

void DWT_Delay(uint32_t us) // microseconds
{
  int32_t tp = DWT_Get() + us * (SystemCoreClock/1000000);
  while (DWT_Compare(tp));
}

uint16_t SPI_Rx(SPI_HandleTypeDef *hspi, uint8_t *buffer, uint16_t buffer_size)
{
	// Timeout = 1000 ms
	switch (HAL_SPI_Receive(hspi, (uint8_t *)buffer, buffer_size, 1000))
	{
		case HAL_OK:
			//DEBUG_TX("SPI Rx OK\r\n");
			return 1;
	        break;
	    case HAL_TIMEOUT:
	    	DEBUG_TX("SPI Rx Timeout\r\n");
	    	return 0;
	    case HAL_ERROR:
	    	DEBUG_TX("SPI Rx Error\r\n");
	    	return 0;
	      break;
	    default:
	      break;
	}
	return 0;
}

uint16_t SPI_Tx(SPI_HandleTypeDef *hspi, uint8_t *buffer, uint16_t buffer_size)
{
	// Timeout = 1000 ms
	switch (HAL_SPI_Transmit(hspi, (uint8_t *)buffer, buffer_size, 1000))
	{
		case HAL_OK:
			DEBUG_TX("SPI Tx OK\r\n");
			return 1;
	        break;
	    case HAL_TIMEOUT:
	    	DEBUG_TX("SPI Tx Timeout\r\n");
	    	return 0;
	    case HAL_ERROR:
	    	DEBUG_TX("SPI Tx Error\r\n");
	    	return 0;
	      break;
	    default:
	      break;
	}
	return 0;
}

uint16_t SPI_TxRx(SPI_HandleTypeDef *hspi, uint8_t *aTxBuffer, uint8_t *aRxBuffer, uint16_t buffer_size)
{
	switch(HAL_SPI_TransmitReceive(hspi, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, buffer_size, 1000))
	{
		case HAL_OK:
			//DEBUG_TX("SPI TxRx OK\r\n");
			return 1;
			break;
		case HAL_TIMEOUT:
	    	DEBUG_TX("SPI TxRx Timeout\r\n");
	    	return 0;
	    	break;
		case HAL_ERROR:
	    	DEBUG_TX("SPI TxRx Error\r\n");
	    	return 0;
			break;
		default:
			break;
	}
	return 0;
}
