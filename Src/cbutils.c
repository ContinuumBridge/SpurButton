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
#include "rtc.h"
#include "cbutils.h"

extern char debug_buff[128];

/* Code */

__IO ITStatus UartReady = RESET;

void *array_concat(const void *a, size_t an, const void *b, size_t bn, size_t s)
{
  char *p = malloc(s * (an + bn));
  memcpy(p, a, an*s);
  memcpy(p + an*s, b, bn*s);
  return p;
}

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

uint32_t Cbr_Now(void)
{
	RTC_TimeTypeDef rtc_time;
	RTC_DateTypeDef sdatestructureget;
	HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
	//sprintf(debug_buff,"Cbr_Now: %02d:%02d:%02d:%d\r\n",rtc_time.Hours, rtc_time.Minutes, rtc_time.Seconds, sixteenths);
	//DEBUG_TX(debug_buff);
	return rtc_time.Hours*3600 + rtc_time.Minutes*60 + rtc_time.Seconds;
}

void RTC_Delay(uint32_t delay)    // Delay is in seconds
{
	RTC_TimeTypeDef rtcTime;
	RTC_DateTypeDef sdatestructureget;
	RTC_AlarmTypeDef sAlarm;
	uint8_t h, m, s, cs, cm;
	HMS(delay, &h, &m, &s);
	HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);  // Needed after HAL_RTC_GetTime to prevent locking
    cs = (rtcTime.Seconds + s)/60; s = (rtcTime.Seconds + s)%60;
    cm = (rtcTime.Minutes + m + cs)/60; m = (rtcTime.Minutes + m + cs)%60;
    h = (rtcTime.Hours + h + cm)%24;
	sprintf(debug_buff,"Now: %02d:%02d:%02d. Alarm at: %02d:%02d:%02d\r\n", rtcTime.Hours, rtcTime.Minutes, rtcTime.Seconds, h ,m, s);
	DEBUG_TX(debug_buff);
	sAlarm.AlarmTime.Hours = h;
	sAlarm.AlarmTime.Minutes = m;
	sAlarm.AlarmTime.Seconds = s;
	sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
	sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
	sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	sAlarm.AlarmDateWeekDay = 1;
	sAlarm.Alarm = RTC_ALARM_A;
	HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, FORMAT_BIN);
}

void HMS(uint32_t e, uint8_t *h, uint8_t *m, uint8_t *s)
{
    *s = e%60; e /= 60;
    *m = e%60; e /= 60;
    *h = e%24; e /= 24;
}

void Enable_IRQ(void)
{
	//uint32_t c = 1000;  // In case something has gone wrong
	DEBUG_TXS("Enable_IRQ\r\n", 12);
	// This is the recommended way of clearing IRQs
	//while (HAL_NVIC_GetPendingIRQ(EXTI15_10_IRQn) && c--)
	//{
	//	__HAL_GPIO_EXTI_CLEAR_IT(EXTI15_10_IRQn);
	//	HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
	//}
	//HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	//c = 1000;
	//while (HAL_NVIC_GetPendingIRQ(EXTI3_IRQn) && c--)
	//{
	//	__HAL_GPIO_EXTI_CLEAR_IT(EXTI3_IRQn);
	//	HAL_NVIC_ClearPendingIRQ(EXTI3_IRQn);
	//}
	//HAL_NVIC_SetPriority(EXTI3_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	return;
}

