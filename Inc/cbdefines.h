/**
  ******************************************************************************
  * @file    cbdefines.h
  * @brief   ContinuumBridge utilities
  ******************************************************************************
  *
  * 
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CBDEFINES_H
#define __CBDEFINES_H

#ifdef __cplusplus
 extern "C" {
#endif 
#define COUNTOF(x)  		(sizeof(x) / sizeof((x)[0]))
#define TXBUFFERSIZE 		(COUNTOF(aTxBuffer) - 1)
#define RXBUFFERSIZE 		8

#define BEACON_ADDRESS      0xBBBB
#define GRANT_ADDRESS       0xBB00

// Function codes:
#define  include_req        0x00
#define  s_include_req      0x01
#define  include_grant      0x02
#define  reinclude          0x04
#define  config             0x05
#define  send_battery       0x06
#define  alert              0x09
#define  woken_up           0x07
#define  ack                0x08
#define  beacon             0x0A


#define LPRS_HEADER_LENGTH 	6
#define ARRAY_CONCAT(TYPE, A, An, B, Bn) \
  (TYPE *)array_concat((const void *)(A), (An), (const void *)(B), (Bn), sizeof(TYPE));
#define DEBUG_TX(MESSAGE)  (UART_Tx(&huart1, (uint8_t*)MESSAGE, (COUNTOF(MESSAGE) - 1)))
#define RADIO_TXS(MESSAGE, LENGTH)  (UART_Tx(&huart3, (uint8_t*)MESSAGE, LENGTH))
#define DEBUG_TXS(MESSAGE, LENGTH)  (UART_Tx(&huart1, (uint8_t*)MESSAGE, LENGTH))
#define DEBUG_RX(MESSAGE, LENGTH)  (UART_Rx_IT(&huart1, (uint8_t*)MESSAGE, LENGTH))
#define RADIO_RX(MESSAGE, LENGTH)  (UART_Rx_IT(&huart3, (uint8_t*)MESSAGE, LENGTH))


#ifdef __cplusplus
}
#endif

#endif /* __CBUTILS_H */

/*****************************END OF FILE****/
