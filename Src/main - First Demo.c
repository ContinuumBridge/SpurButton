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
#include "adc.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include "cbutils.h"
#include "process.h"
#include "define.h"       // Display drivers
#include "glue.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define FONT_0 Verdana_Pro_SemiBold18x16
#define FONT_1 Arial_Rounded_MT_Bold26x27
#define FONT_2 Arial_Rounded_MT_Bold19x20
#define MAX_SCREEN 6
#define DISPLAY_INITIAL 6
#define DISPLAY_CONNECTING 7
#define DISPLAY_PROBLEM 8

#define SEARCH_OK 0
#define SEARCH_TIMEOUT 1
#define SEARCH_ERROR 2
#define SEND_OK 0
#define SEND_TIMEOUT 1
#define MAX_SEARCH_TIME 30
#define ONE_DAY (24*60*60)

// Function codes:
#define  f_include_req        0x00
#define  f_s_include_req      0x01
#define  f_include_grant      0x02
#define  f_reinclude          0x04
#define  f_config             0x05
#define  f_send_battery       0x06
#define  f_alert              0x09
#define  f_woken_up           0x07
#define  f_ack                0x08
#define  f_beacon             0x0A

HAL_StatusTypeDef status;
int length;
uint8_t Rx_Buffer[128];
uint8_t tx_message[64];
uint8_t tx_length;
uint8_t node_id[] = {0x00, 0x00, 0x00, 0x2F};
uint8_t bridge_address[2] = {0xFF, 0xFF};
uint8_t node_address[2] = {0x00, 0x00};
uint8_t beacon_address[] = {0xBB, 0xBB};
uint8_t grant_address[] = {0xBB, 0x00};

typedef struct{
	char line[5][32];
	int num_lines;
	uint8_t *font;
	int justification;
} Screen;

const int y_start[5][5] = {
		{32, 0, 0 ,0, 0},
		{16, 50, 0, 0, 0},
		{5, 32, 61, 0, 0},
		{4, 26, 48, 70},
		{0, 20, 40, 60, 80}
};

Screen screen[10];                // Old way
char screens[8][4][128];          // New way

int radio_ready   = SET;
int screen_num = 0;
RTC_HandleTypeDef hrtc;
uint8_t include_state = 0;
uint8_t send_attempt = 0;
uint8_t sleeping = 0;
char debug_buff[64] = {0};
uint8_t config_stored = 0;

typedef enum {initial, normal, pressed, search, search_failed, reverting} NodeState;
NodeState         node_state           = initial;
/*
typedef struct {
	int sent;
	uint8_t l;
	uint8_t m[64];
} Tx_Message;

Tx_Message tx_queue[8];
tx_pointer uint8_t = 0;
*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void create_screen(void);
void test_screen(void);
void Set_Screen(const Screen *screen);
HAL_StatusTypeDef Rx_Message(uint8_t *buffer, int *length, uint16_t timeout);
static void RTC_TimeShow(void);
void RTC_Delay(uint32_t delay);
uint32_t Cbr_Now(void);
uint32_t Cbr_Time(uint8_t h, uint8_t m, uint8_t s);
void HMS(uint32_t e, uint8_t *h, uint8_t *m, uint8_t *s);
void Radio_On(void);
void Radio_Off(void);
uint8_t Message_Search(uint8_t *address, uint8_t *Rx_Buffer);
void Send_Message(uint8_t function, uint8_t data_length, uint8_t *data, uint8_t ack);
void Manage_Send(uint8_t ack);
void Enable_IRQ(void);
void Network_Include(void);
void Listen_Radio(void);
void Wakeup(void);
void Set_Wakeup(uint8_t force_awake);
void Build_Screen(uint8_t screen_num);
void Store_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  char buffer[128] = {45};
  int  i;
  int length;

  /*
  static uint16_t   destination;
  static uint16_t   source;
  static uint16_t   wakeup;
  static uint8_t    function;
  static uint8_t    length;
  static uint8_t    nodeAddress         = 0;
  static uint8_t    *payload;
  */


  strcpy(screen[0].line[0], "Push here to");
  strcpy(screen[0].line[1], "report a fault");
  strcpy(screen[0].line[2], "");
  screen[0].font = FONT_1;
  screen[0].num_lines = 2;
  screen[0].justification = 0;

  strcpy(screen[1].line[0], "Fault with this");
  strcpy(screen[1].line[1], "appliance has");
  strcpy(screen[1].line[2], "been reported");
  screen[1].font = FONT_1;
  screen[1].num_lines = 3;
  screen[1].justification = 0;

  strcpy(screen[2].line[0], "Push here");
  strcpy(screen[2].line[1], "for table");
  strcpy(screen[2].line[2], "service");
  screen[2].font = FONT_1;
  screen[2].num_lines = 3;
  screen[2].justification = 0;

  strcpy(screen[3].line[0], "Someone");
  strcpy(screen[3].line[1], "will be with");
  strcpy(screen[3].line[2], "you soon");
  screen[3].font = FONT_1;
  screen[3].num_lines = 3;
  screen[3].justification = 0;

  strcpy(screen[4].line[0], "Push here if this");
  strcpy(screen[4].line[1], "coffee machine");
  strcpy(screen[4].line[2], "has run out");
  strcpy(screen[4].line[3], "of coffee");
  screen[4].font = FONT_2;
  screen[4].num_lines = 4;
  screen[4].justification = 0;

  strcpy(screen[5].line[0], "Someone is on");
  strcpy(screen[5].line[1], "their way with");
  strcpy(screen[5].line[2], "more coffee for");
  strcpy(screen[5].line[3], "this machine");
  screen[5].font = FONT_2;
  screen[5].num_lines = 4;
  screen[5].justification = 0;

  strcpy(screen[6].line[0], "Welcome to Spur");
  strcpy(screen[6].line[1], "Push here for 3");
  strcpy(screen[6].line[2], "seconds to connect");
  strcpy(screen[6].line[3], "to network");
  screen[6].font = FONT_2;
  screen[6].num_lines = 4;
  screen[6].justification = 0;

  strcpy(screen[7].line[0], "Trying to connect");
  strcpy(screen[7].line[1], "to network");
  strcpy(screen[7].line[2], "Please wait");
  screen[7].font = FONT_2;
  screen[7].num_lines = 3;
  screen[7].justification = 0;

  strcpy(screen[8].line[0], "Communication");
  strcpy(screen[8].line[1], "problem");
  strcpy(screen[8].line[2], "Button not in use");
  screen[8].font = FONT_2;
  screen[8].num_lines = 3;
  screen[8].justification = 0;

  strcpy(screens[6][0], "F\x02" "Y\x04" "C\x0F" "Welcome to Spur\x01" "Y\x1A" "C\x0F" "Push here for 3\x01" "Y\x30" "C\x12" "seconds to connect\x01" "Y\x46" "C\x0A" "to network\x01" "ES");
  strcpy(screens[7][0], "F\x02" "Y\x05" "C\x11" "Trying to connect\x01" "Y\x20" "C\x0A" "to network\x01" "Y\x3D" "C\x0B" "Please wait\x01" "ES");
  //strcpy(screens[8][0], "F\x02Y\x05C\x11Trying to connect\x00Y\x20C\x0Ato network\x00Y\x3D\x0BCPlease wait\x00ES");
for(i=0; i<128; i++)
{
	if(screens[6][0][i] == 0x01)
		screens[6][0][i] = 0x00;
	if(screens[7][0][i] == 0x01)
		screens[7][0][i] = 0x00;
}

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_RTC_Init();
  //MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */

  // Set pins for display
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_VERY_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOB, BORDER_CONTROL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, DISPLAY_ON_PB6_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, _RESET_DISPLAY_Pin, GPIO_PIN_SET);
  Delay_ms(5);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(_SPI_CS_GPIO_Port, _SPI_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, DISPLAY_DISCHARGE_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB, HOST_READY_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RADIO_POWER_GPIO_Port, RADIO_POWER_Pin, GPIO_PIN_RESET);

  DEBUG_TX("Hello Computer\r\n");
  ecog_init();
  //Set_Screen(&screen[6]);
  set_display(6);
  //ecog_update_display(0);

  /*
  RTC_TimeShow();
  RTC_Delay(20);
  Delay_ms(1000);
  RTC_Delay(5);
  RTC_TimeShow();
  */

  // Set OTA data rate to 1200 bps
  /*
  RADIO_TXS("ER_CMD#B0", 9);
  RADIO_RX(buffer, 9, 1);
  DEBUG_TX("Received: ");
  DEBUG_TXS(buffer, 9);
  DEBUG_TX("\r\n");
  RADIO_TXS("ACK", 3);
  HAL_Delay(200);
  */

  Radio_On();
  RADIO_TXS("Hello World", 11);
  DEBUG_TX("Sent Hello World\r\n");
  Delay_ms(50);
  for (i=0; i<128; i++)
	  Rx_Buffer[i] = 45;
  status = Rx_Message(Rx_Buffer, &length, 2000);
  if (status == HAL_OK)
  {
	  DEBUG_TX("Received: ");
	  DEBUG_TXS(Rx_Buffer, 32);
	  DEBUG_TX("\r\n");
	  sprintf(buffer, "Length %d\r\n", length);
	  DEBUG_TX(buffer);
  }
  else
  	  DEBUG_TX("Receive problem");
  Radio_Off();

   //DWT_Init();
   //DEBUG_TX("DWT_Init\r\n");
   //RADIO_RX(Rx_Data, 1);        // Interrupt on receiving 1 byte

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

	  /*
	  __HAL_UART_FLUSH_DRREGISTER(&huart3);
	  */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

void Set_Screen(const Screen *screen)
{
	int l;
	if (screen->justification == 0)
	{
		for(l=0; l< screen->num_lines; l++)
			ecog_printfc(screen->font, y_start[screen->num_lines-1][l], screen->line[l]);
	}
	else
	{
		for(l=0; l< screen->num_lines; l++)
			ecog_printfc(screen->font, y_start[screen->num_lines-1][l], screen->line[l]);
	}
}

void test_screen(void)
{
	ecog_line(10, 10, 10, 30, ECOG_ON);

	ecog_line(15, 10, 15, 30, ECOG_ON);
	ecog_line(16, 10, 16, 30, ECOG_ON);

	ecog_line(20, 10, 20, 30, ECOG_ON);
	ecog_line(21, 10, 21, 30, ECOG_ON);
	ecog_line(22, 10, 22, 30, ECOG_ON);

	ecog_line(30, 10, 30, 30, ECOG_ON);
	ecog_line(31, 10, 31, 30, ECOG_ON);
	ecog_line(32, 10, 32, 30, ECOG_ON);
	ecog_line(33, 10, 33, 30, ECOG_ON);
	ecog_line(34, 10, 34, 30, ECOG_ON);
}

void set_display(uint8_t screen_num)
{
	if (ecog_write_inverse())
	{
		ecog_cls();
		/*
		if(config_stored < 2)
			Set_Screen(&screen[screen_num]);
		else
		*/
		Build_Screen(screen_num);
		ecog_update_display(1);
	}
}

void Build_Screen(uint8_t screen_num)
{
	int pos = 0;
	int l, len;
	//char text[32];
	uint8_t font, x, y = 0;
	uint8_t region = 0;
	uint8_t parsing = 1;
	uint8_t loops = 0;

	DEBUG_TX("Build_Screen\r\n");
	while(parsing)
	{
		if(strncmp(screens[screen_num][region]+pos, "S", 1) == 0)
		{
			pos += 2;
			DEBUG_TX("Screen\r\n");

		}
		else if(strncmp(screens[screen_num][region]+pos, "R", 1) == 0)
		{
			pos += 2;
			DEBUG_TX("Region\r\n");
		}
		else if(strncmp(screens[screen_num][region]+pos, "F", 1) == 0)
		{
			pos++; font = screens[screen_num][region][pos]; pos++;
			sprintf(debug_buff,"Build_Screen, font: %d                             \r\n", font);
			DEBUG_TX(debug_buff);
		}
		else if(strncmp(screens[screen_num][region]+pos, "Y", 1) == 0)
		{
			pos++; y = screens[screen_num][region][pos]; pos++;
			sprintf(debug_buff,"Build_Screen, y: %d                             \r\n", y);
			DEBUG_TX(debug_buff);
		}
		else if(strncmp(screens[screen_num][region]+pos, "C", 1) == 0)
		{
			pos++; len = screens[screen_num][region][pos]; pos++;
			sprintf(debug_buff,"Build_Screen, pos: %d, len: %d   \r\n", pos, len);
			DEBUG_TX(debug_buff);
			ecog_printfc(FONT_2, y, screens[screen_num][region] + pos);
			sprintf(debug_buff,"Build_Screen, text: %20s\r\n", screens[screen_num][region] + pos);
			DEBUG_TX(debug_buff);
			pos += len + 1;
			sprintf(debug_buff,"Build_Screen, pos: %d, next 3 chars: %3s\r\n", pos, screens[screen_num][region] + pos);
			DEBUG_TX(debug_buff);
		}
		else if(strncmp(screens[screen_num][region]+pos, "E", 1) == 0)
		{
			DEBUG_TX("Build_Screen found E\r\n");
			pos++;
			if(strncmp(screens[screen_num][region]+pos, "R", 1) == 0)
			{
				pos++; region++;
				loops = 0;
				DEBUG_TX("Build_Screen end region\r\n");
			}
			else if(strncmp(screens[screen_num][region]+pos, "S", 1) == 0)
			{
				parsing = 0;
				DEBUG_TX("Build_Screen end screen\r\n");
			}
        }
		loops++;
		if(loops >127)
		{
			parsing = 0;
			DEBUG_TX("Build_Screen did not find end of screen\r\n");
		}
	}
}


void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin)
{
	static uint32_t button_press_time = 0;
	uint32_t pressed_time = 0;
	uint32_t now;
	int button_pressed = 0;
	GPIO_PinState  button_state;
	uint8_t alert_id[] = {0x00, 0x00};

	if (GPIO_Pin == GPIO_PIN_3)
	{
		button_state = HAL_GPIO_ReadPin(USER_INPUT_GPIO_Port, USER_INPUT_Pin);
		sprintf(debug_buff,"Button 3 event: %d                             \r\n", (int)Cbr_Now());
		DEBUG_TX(debug_buff);
		if (button_state == GPIO_PIN_RESET)
		{
			Delay_ms(100);  // Debounce
			if (HAL_GPIO_ReadPin(USER_INPUT_GPIO_Port, USER_INPUT_Pin) == GPIO_PIN_SET)
			{
				DEBUG_TX( "Returning from pressed\r\n" );
				return;
			}
			else
			{
				button_press_time = Cbr_Now();
				DEBUG_TX( "Button 3 pressed: \r\n" );
			}
		}
		else if (button_state == GPIO_PIN_SET)
		{
			Delay_ms(100);  // Debounce
			if (HAL_GPIO_ReadPin(USER_INPUT_GPIO_Port, USER_INPUT_Pin) == GPIO_PIN_RESET)
			{
				DEBUG_TX( "Returning from released\r\n" );
				return;
			}
			else
			{
				HAL_NVIC_DisableIRQ(EXTI3_IRQn);
				now = Cbr_Now();
				pressed_time = now - button_press_time;
				button_press_time = now;
				button_pressed = 1;
				DEBUG_TX( "Button 3 released: \r\n" );
			}
		}

		if (!button_pressed)
			return;
		else
			button_pressed = 0;
		sprintf(debug_buff,"Pressed time: %d\r\n", (int)pressed_time);
		DEBUG_TX(debug_buff);
#ifdef CB_DEMO_0
		if (ecog_write_inverse())
		{
			ecog_cls();
			screen_num++;
			if (screen_num == MAX_SCREEN)
				screen_num = 0;
			Set_Screen(&screen[screen_num]);
			ecog_update_display(1);
		}
#else
		if (node_state == initial)
		{
			if (pressed_time > T_START_PRESS)
			{
				Network_Include();
			}
			else
				Enable_IRQ();
		}
		else if(node_state == normal)
		{
			DEBUG_TX("node_state normal pressed\r\n");
			set_display(1);
			alert_id[0] = 0x00; alert_id[1] = 0x00;
			DEBUG_TX("Sending pressed alert\r\n");
			Send_Message(f_alert, 2, alert_id, 1);
			DEBUG_TX("Sent pressed alert\r\n");
			node_state = pressed;
			Enable_IRQ();
		}
		else if(node_state == pressed)
		{
			DEBUG_TX("node_state pressed\r\n");
			if (pressed_time > T_REVERT_PRESS)
			{
				set_display(0);
				alert_id[0] = 0x01; alert_id[1] = 0x00;
				DEBUG_TX("Sending cleared alert\r\n");
				Send_Message(f_alert, 2, alert_id, 1);
				DEBUG_TX("Sent cleared alert\r\n");
				node_state = normal;
				Enable_IRQ();
			}
			Enable_IRQ();
		}
		else
			Enable_IRQ();
  }
  else
  {
    DEBUG_TX( "Other button pressed: \r\n" );
  }
#endif
}

void Enable_IRQ(void)
{
	// This is the recommended way of clearing IRQs
	while (HAL_NVIC_GetPendingIRQ(EXTI3_IRQn))
	{
		__HAL_GPIO_EXTI_CLEAR_IT(EXTI3_IRQn);
		HAL_NVIC_ClearPendingIRQ(EXTI3_IRQn);
	}
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

void Network_Include(void)
{
	uint8_t equal = 1;
	int i;
	DEBUG_TX("Network_Include\r\n");
	if(include_state == 0)
	{
		set_display(DISPLAY_CONNECTING);
		include_state = 1;
	}
	else if(include_state == 1)
	{
		set_display(DISPLAY_PROBLEM);
		include_state = 2;
	}
	Radio_On();
	if(Message_Search(beacon_address, Rx_Buffer) == SEARCH_OK)
	{
		bridge_address[0] = Rx_Buffer[2]; bridge_address[1] = Rx_Buffer[3];
		Send_Message(f_include_req, 4, node_id, 0);
	}
	else
	{
		switch(include_state)
		{
			case 1:
				Radio_Off();
				RTC_Delay(10*60);
				break;
			case 2:
				Radio_Off();
				RTC_Delay(60*60);
				break;
			default:
				Radio_Off();
				RTC_Delay(60*60);
		}
		return;
	}
	if(Message_Search(grant_address, Rx_Buffer) == SEARCH_OK)
	{
		DEBUG_TX("Received grant\r\n");
		equal = 1;
		for(i=0; i<4; i++)
			if(Rx_Buffer[i+12] != node_id[i])
			{
				equal = 0;
				break;
			}
		if(equal)
		{
			node_address[0] = Rx_Buffer[16]; node_address[1] = Rx_Buffer[17];
		}
	}
	uint8_t data[] = {0x00, 0x00};
	Send_Message(f_ack, 0, data, 0);
	Send_Message(f_woken_up, 0, data, 1);
	set_display(0);
	Radio_Off();
	Enable_IRQ();
	include_state = 0;
	node_state = normal;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3)
	{
		DEBUG_TX("HAL_UART_TxCpltCallback\r\n");
		radio_ready = SET;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3)
	{
		DEBUG_TX("HAL_UART_RxCpltCallback\r\n");
		radio_ready = SET;
	}
}

void Radio_Tx_IT(UART_HandleTypeDef *uart, uint8_t *buffer, uint16_t buffer_size)
{
	radio_ready = RESET;
	if(HAL_UART_Transmit_IT(uart, (uint8_t *)buffer, buffer_size) != HAL_OK)
	{
		DEBUG_TX("Radio Tx Error\r\n");
	}
	//while (radio_ready != SET)
	//{
	//}
}

void Radio_Tx(UART_HandleTypeDef *uart, uint8_t *buffer, uint16_t buffer_size)
{
	radio_ready = RESET;
	if(HAL_UART_Transmit(uart, (uint8_t *)buffer, buffer_size, 1000) != HAL_OK)
	{
		DEBUG_TX("Radio Tx Error\r\n");
	}
}

void Radio_Rx_IT(UART_HandleTypeDef *uart, uint8_t *buffer, uint16_t buffer_size)
{
	radio_ready = RESET;
	if(HAL_UART_Receive_IT(uart, (uint8_t *)buffer, buffer_size) != HAL_OK)
	{
		DEBUG_TX("UART Rx Error\r\n");
	}
	else
		DEBUG_TX("Radio_Rx_IT OK\r\n");
}

uint16_t Radio_Rx(UART_HandleTypeDef *uart, uint8_t *buffer, uint16_t buffer_size, uint16_t timeout)
{
	HAL_StatusTypeDef status = HAL_UART_Receive(uart, (uint8_t *)buffer, buffer_size, timeout);
	return status;
}

void Radio_On(void)
{
	DEBUG_TX("Radio_On\r\n");
    HAL_GPIO_WritePin(RADIO_POWER_GPIO_Port, RADIO_POWER_Pin, GPIO_PIN_SET);
    Delay_ms(500);
}

void Radio_Off(void)
{
	DEBUG_TX("Radio_Off\r\n");
    HAL_GPIO_WritePin(RADIO_POWER_GPIO_Port, RADIO_POWER_Pin, GPIO_PIN_RESET);
}

void Send_Message(uint8_t function, uint8_t data_length, uint8_t *data, uint8_t ack)
{
	int i;
	DEBUG_TX("Send_Message\r\n");
	tx_message[0] = bridge_address[0]; tx_message[1] = bridge_address[1];
	tx_message[2] = node_address[0]; tx_message[3] = node_address[1];
	tx_message[4] = function;
	tx_message[5] = 0x00; tx_message[6] = 0x00; tx_message[7] = 0x00; tx_message[8] = 0x00;  // Time = 0 for now
	tx_message[9] = 10 + data_length;
	if(data_length != 0)
	{
		for(i=0; i<data_length; i++)
			tx_message[i+10] = data[i];
	}
	tx_length = 10 + data_length;
	sprintf(debug_buff, "Send_Message: %02x %02x %02x %02x %02x %02x, data length: %d\r\n", tx_message[0], tx_message[1], tx_message[2], tx_message[3], tx_message[4], tx_message[5], data_length);
	DEBUG_TX(debug_buff);
	send_attempt = 0;
	Manage_Send(ack);
}

void Manage_Send(uint8_t ack)
{
	sprintf(debug_buff, "Manage_Send, send_attempt: %d                          \r\n", send_attempt);
	DEBUG_TX(debug_buff);
	if((send_attempt == 0) | (send_attempt == 4))
		Radio_On();
	send_attempt++;
	while(HAL_GPIO_ReadPin(GPIOB, RADIO_BUSY_Pin == GPIO_PIN_SET))
	{
	}
	if(Message_Search(beacon_address, Rx_Buffer) == SEARCH_OK)
	{
		DEBUG_TX("Manage_Send found beacon\r\n");
		RADIO_TXS(tx_message, tx_length);
		DEBUG_TX("Manage_Send sent message\r\n");
	}
	else
	{
		DEBUG_TX("Manage_Send did not find beacon\r\n");
		return;
	}
	if(ack)
	{
		DEBUG_TX("Manage_Send waiting for ack\r\n");
		if(Message_Search(node_address, Rx_Buffer) == SEARCH_OK)
		{
			if(Rx_Buffer[4] == f_ack)
			{
				send_attempt = 0;
				DEBUG_TX("Manage_Send ack received\r\n");
				Set_Wakeup(0);
				return;
			}
			else
			{
				// If we get a message that is not an ack when expecting an ack, clear it by sending ack back.
				DEBUG_TX("Manage_Send another message received when expecting ack\r\n");
				uint8_t data[] = {0x00, 0x00};
				Send_Message(f_ack, 0, data, 0);
				Set_Wakeup(1);
			}
		}
		else
		{
			DEBUG_TX("Manage_Send no ack 1\r\n");
			switch(send_attempt)
			{
				case 1:
				case 2:
				case 3:
				case 5:
				case 6:
					Manage_Send(1);
					break;
				case 4:
					Radio_Off();
					RTC_Delay(30000);
					break;
				case 7:
					DEBUG_TX("Manage_Send permanent failure\r\n");
					Radio_Off();
			}
		}
	}
}

void Listen_Radio(void)
{
	DEBUG_TX("Listen_Radio\r\n");
	if(Message_Search(node_address, Rx_Buffer) == SEARCH_OK)
	{
		uint8_t data[] = {0x00, 0x00};
		if(Rx_Buffer[4] == f_config)
		{
			DEBUG_TX("Listen_Radio. Config message received\r\n");
			Store_Config();
			Send_Message(f_ack, 0, data, 0);
			Set_Wakeup(0);
		}
		else if(Rx_Buffer[4] == f_ack)
		{
			DEBUG_TX("Listen_Radio. Ack received\r\n");
			Set_Wakeup(0);
		}
		else
		{
			DEBUG_TX("Listen_Radio. Sending ack\r\n");
			Send_Message(f_ack, 0, data, 0);
			Set_Wakeup(0);
		}
	}
	else
		DEBUG_TX("Listen_Radio. Search failed\r\n");

}

void Set_Wakeup(uint8_t force_awake)
{
	DEBUG_TX("Set_Wakeup\r\n");
	if(force_awake)
	{
		Listen_Radio();
	}
	else
	{
		uint32_t wakeup = ((Rx_Buffer[10] << 8) | Rx_Buffer[11]) << 1;
		if(wakeup > ONE_DAY)
			wakeup = ONE_DAY;
		if(wakeup == 0)
			Listen_Radio();
		else
		{
			Radio_Off();
			sleeping = 1;
			RTC_Delay(wakeup);
		}
	}
}

void Wakeup(void)
{
	DEBUG_TX("Woken up\r\n");
	sleeping = 0;
	uint8_t data[] = {0x00, 0x00};
	Send_Message(f_woken_up, 0, data, 1);
}

void Store_Config(void)
{
	uint8_t s, r, i = 0;
	uint8_t pos = 12;
	if(strncmp(Rx_Buffer+pos, "S", 1) == 0)
		s = Rx_Buffer[pos+1];
	if(strncmp(Rx_Buffer+pos+2, "R", 1) == 0)
		r = Rx_Buffer[pos+3];
	sprintf(debug_buff, "Screen: %d, region: %d\r\n", s, r);
	DEBUG_TX(debug_buff);
	for(i=0; i<64; i++)
		debug_buff[i] = 0;
	for(i=0; i<Rx_Buffer[5]; i++)
	{
		screens[s][r][i] = Rx_Buffer[i+pos+4];
		//sprintf(debug_buff, "%c", screens[s][r][i]);
		//DEBUG_TX(debug_buff);
	}
	//DEBUG_TX("\r\n");
	//sprintf(debug_buff, "Store_Config, screen: %d, reg: %d, %02x %02x %02x %02x %02x %02x\r\n", s, r, screens[s][r][0], screens[s][r][1], screens[s][r][2], screens[s][r][3], screens[s][r][4], screens[s][r][5]);
	//DEBUG_TX(debug_buff);
	config_stored++;
	sprintf(debug_buff, "Store_Config, config_stored: %d\r\n", config_stored);
	DEBUG_TX(debug_buff);
}

uint8_t Message_Search(uint8_t *address, uint8_t *Rx_Buffer)
{
	int search_start = Cbr_Now();
	sprintf(debug_buff, "Message_Search, address: %02x %02x\r\n", address[0], address[1]);
	DEBUG_TX(debug_buff);
	__HAL_UART_FLUSH_DRREGISTER(&huart3);
	while (1)
	{
		status = Rx_Message(Rx_Buffer, &length, 6000);
		if (status == HAL_OK)
		{
			sprintf(debug_buff, "MS Rx: %02x %02x %02x %02x %02x %02x, Length: %d\r\n", Rx_Buffer[0], Rx_Buffer[1], Rx_Buffer[2], Rx_Buffer[3], Rx_Buffer[4], Rx_Buffer[5], length);
			DEBUG_TX(debug_buff);
			if((Rx_Buffer[0] == address[0]) & (Rx_Buffer[1] == address[1]))
			{
				return SEARCH_OK;
			}
			if ((Cbr_Now() - search_start) > MAX_SEARCH_TIME)
			{
				DEBUG_TX("Message_Search timeout\r\n");
				return SEARCH_TIMEOUT;
			}
		}
		else
		{
			__HAL_UART_FLUSH_DRREGISTER(&huart3);
			DEBUG_TX("Message_Search status != HAL_OK\r\n");
			if ((Cbr_Now() - search_start) > MAX_SEARCH_TIME)
			{
				DEBUG_TX("Message_Search error\r\n");
				return SEARCH_ERROR;
			}
		}
	}
}

HAL_StatusTypeDef Rx_Message(uint8_t *buffer, int *length, uint16_t timeout)
{
	uint8_t Rx_Data[2];
	HAL_StatusTypeDef status;
	int len = 0;
	//DEBUG_TX("In Rx_Message\r\n");
	status = RADIO_RX(Rx_Data, 1, timeout);
	//DEBUG_TX("Received byte 1\r\n");
	if (status != HAL_OK)
		return status;
	else
	{
		buffer[0] = Rx_Data[0];
		//DEBUG_TX("Rx: ");
		while (1)
		{
			len++;
			status = RADIO_RX(Rx_Data, 1, 5);
			if (status == HAL_OK)
			{
				//DEBUG_TXS(Rx_Data, 1);
				buffer[len] = Rx_Data[0];
			}
			else if (status == HAL_TIMEOUT)
			{
				//char buffer1[64] = {" "};
				//sprintf(buffer1, "\r\nLength %d, Received: %s\r\n", len, buffer);
				//DEBUG_TX(buffer1);
				*length = len;
				return HAL_OK;
			}
			else
			{
				DEBUG_TX("Radio_Rx error 2");
				return status;
			}
		}
	}
}

static void RTC_TimeShow(void)
{
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;

  /* Get the RTC current Time */
  HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
  /* Display time Format : hh:mm:ss */
  sprintf(debug_buff,"%02d:%02d:%02d\r\n",stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
  DEBUG_TX(debug_buff);
}

uint32_t Cbr_Now(void)
{
	RTC_TimeTypeDef rtc_time;
	HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
	//sprintf(debug_buff,"Cbr_Now: %02d:%02d:%02\r\n",rtc_time.Hours, rtc_time.Minutes, rtc_time.Seconds);
	//DEBUG_TX(debug_buff);
	return Cbr_Time(rtc_time.Hours, rtc_time.Minutes, rtc_time.Seconds);
}

uint32_t Cbr_Time(uint8_t h, uint8_t m, uint8_t s)
{
	return h*3600 + m*60 +s;
}

void RTC_Delay(uint32_t delay)
{
	RTC_TimeTypeDef rtcTime;
	RTC_DateTypeDef sdatestructureget;
	RTC_AlarmTypeDef sAlarm;
	uint8_t h, m, s, cs, cm;
	HMS(delay, &h, &m, &s);
	HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);  // Needed after HAL_RTC_GetTime to prevent locking
	sprintf(debug_buff,"Time nowL %02d:%02d:%02d\r\n",rtcTime.Hours, rtcTime.Minutes, rtcTime.Seconds);
	DEBUG_TX(debug_buff);
    cs = (rtcTime.Seconds + s)/60; s = (rtcTime.Seconds + s)%60;
    cm = (rtcTime.Minutes + m + cs)/60; m = (rtcTime.Minutes + m + cs)%60;
    h = (rtcTime.Hours + h + cm)%24;
	sprintf(debug_buff,"Alarm at: %02d:%02d:%02d\r\n",h ,m, s);
	DEBUG_TX(debug_buff);

	sAlarm.AlarmTime.Hours = h;
	sAlarm.AlarmTime.Minutes = m;
	sAlarm.AlarmTime.Seconds = s;
	//sAlarm.AlarmTime.SubSeconds = ss;
	//sAlarm.AlarmTime.SubSeconds = 0x0;
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

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	DEBUG_TX("\r\nHAL_RTC_AlarmAEventCallback\r\n");
	if(include_state != 0)
	{
		Network_Include();
	}
	else if(send_attempt == 4)
		Manage_Send(1);
	else if(sleeping)
		Wakeup();
}


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
