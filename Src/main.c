/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 ContinuumBridge Ltd
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
//#define CB_DEMO_0
//#define FONT_1 Verdana_Pro_SemiBold18x16
#define FONT_2 					Arial_Rounded_MT_Bold19x20
//#define FONT_2 Arial_Narrow16x20
#define FONT_3 					Arial_Rounded_MT_Bold26x27

#define STATE_SENDING			18
#define STATE_INITIAL 			19
#define STATE_CONNECTING 		20
#define STATE_CONFIG 			21
#define STATE_PROBLEM 			23
#define STATE_DEMO				25
#define STATE_NORMAL 			1
#define STATE_PRESSED 			2
#define STATE_OVERRIDE 			3

#define SEARCH_OK 				0
#define SEARCH_TIMEOUT 			1
#define SEARCH_ERROR 			2
#define SEND_OK 				0
#define SEND_TIMEOUT 			1
#define BEACON_SEARCH_TIME 		10
#define ACK_SEARCH_TIME         2
#define ONE_DAY 				(24*60*60)
#define T_LONG_PRESS          	2
#define T_RESET_PRESS         	8
#define T_MAX_RESET_PRESS		20  // To catch failure case

#define MAX_SCREEN 32
#define REGIONS 2

// Function codes:
#define  f_include_req        	0x00
#define  f_s_include_req      	0x01
#define  f_include_grant      	0x02
#define  f_reinclude          	0x04
#define  f_config             	0x05
#define  f_send_battery       	0x06
#define  f_alert              	0x09
#define  f_woken_up           	0x07
#define  f_ack                	0x08
#define  f_beacon             	0x0A
#define  f_start             	0x0B
#define	 f_unknown				0x0C

#define RIGHT_SIDE			  	0
#define LEFT_SIDE			  	1

#define PRESS_LEFT_SINGLE	  	0
#define PRESS_RIGHT_SINGLE    	1
#define PRESS_LEFT_LONG		  	2
#define PRESS_RIGHT_LONG      	3
#define PRESS_LEFT_DOUBLE     	4
#define PRESS_RIGHT_DOUBLE    	5
#define PRESS_RESET           	6
#define PRESS_NONE            	7
#define PRESS_DEMO	  		  	8

#define OPERATIONAL			  	0
#define RATE				  	1
#define ROLLING_DEMO		  	2
#define ROLLING_DEMO_PIN	  	100

#define S_S						0
#define S_D						1
#define S_A						2
#define S_LD					3
#define S_LS					4
#define S_MS					5
#define S_MD					6
#define S_RS					7
#define S_RD					8
#define S_XV					9
#define S_XS					10
#define S_W						11
#define S_WS					12

#define MODE				  	OPERATIONAL

//uint8_t 			node_id[] 			= {0x00, 0x00, 0x00, 0x2F};    // Battery 47
//uint8_t 			node_id[] 			= {0x00, 0x00, 0x00, 0x0A};  // Development
//uint8_t 			node_id[] 			= {0x00, 0x00, 0x00, 0x10};  // Brexit
//uint8_t 			node_id[] 			= {0x00, 0x00, 0x00, 0x11};  // 17
//uint8_t 			node_id[] 			= {0x00, 0x00, 0x00, 0x12};  // 18
//uint8_t 			node_id[] 			= {0x00, 0x00, 0x00, 0x13};  // Smart IoT 19 Martin's
//uint8_t 			node_id[] 			= {0x00, 0x00, 0x00, 0x14};  // Smart IoT 20
//uint8_t 			node_id[] 			= {0x00, 0x00, 0x00, 0x15};  // Smart IoT 21
//uint8_t 			node_id[] 			= {0x00, 0x00, 0x00, 0x16};  // Smart IoT 22
//uint8_t 			node_id[] 			= {0x00, 0x00, 0x00, 0x17};  // 23
uint8_t 			node_id[] 			= {0x00, 0x00, 0x00, 0x18};  // 24

char 				debug_buff[64] 		= {0};
char 				screens[MAX_SCREEN][1][194];
uint8_t				states[24][16]      = {0xFF};

HAL_StatusTypeDef 	status;
int 				length;
uint8_t 			Rx_Buffer[256];
uint8_t 			tx_message[64];
uint8_t 			tx_length;
uint8_t 			bridge_address[2] 		= {0xFF, 0xFF};
uint8_t 			node_address[2] 		= {0x00, 0x00};
uint8_t 			beacon_address[] 		= {0xBB, 0xBB};
uint8_t 			grant_address[] 		= {0xBB, 0x00};

int 				radio_ready   			= SET;
int 				screen_num 				= 0;
uint8_t 			button_irq				= 0;
RTC_HandleTypeDef 	hrtc;
uint8_t 			include_state 			= 0;
uint8_t 			send_attempt 			= 0;
uint8_t 			config_stored 			= 0;
uint8_t 			app_value 				= 0;
uint8_t 			current_screen 			= 0;
uint8_t 			rtc_irq					= 0;
uint16_t 			pressed_button;
uint8_t 			running 				= 0;
GPIO_PinState  		button_state;
uint8_t				current_state			= STATE_INITIAL;
uint32_t			button_press_time[2] 	= {0, 0};

typedef enum {initial, normal, pressed, search, search_failed, reverting, demo} NodeState;
NodeState         node_state           = initial;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Set_Display(uint8_t screen_num, uint8_t turn_on_radio);
HAL_StatusTypeDef Rx_Message(uint8_t *buffer, int *length, uint16_t timeout);
void Radio_On(void);
void Radio_Off(void);
uint8_t Message_Search(uint8_t *address, uint8_t *Rx_Buffer, uint16_t max_search_time);
void Send_Message(uint8_t function, uint8_t data_length, uint8_t *data, uint8_t ack, uint8_t beacon);
void Manage_Send(uint8_t ack, uint8_t beacon, uint8_t function);
void Network_Include(void);
void Listen_Radio(void);
void Set_Wakeup(uint8_t force_awake);
void Build_Screen(uint8_t screen_num);
void Store_Config(void);
void Load_Normal_Screens(void);
void Load_Demo_Screens(void);
static void SYSCLKConfig_STOP(void);
static void SystemPower_Config(void);
void Send_Delay(void);
void Power_Down(void);
void On_RTC_IRQ(void);
void On_Button_IRQ(uint16_t GPIO_Pin, GPIO_PinState button_state);
void Initialise_States(void);
void On_NewState(void);
void Send_Battery();

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  int  i;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  SystemPower_Config();

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

  HAL_GPIO_WritePin(GPIOB, DISPLAY_ON_PB6_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, _RESET_DISPLAY_Pin, GPIO_PIN_SET);
  Delay_ms(5);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(_SPI_CS_GPIO_Port, _SPI_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, DISPLAY_DISCHARGE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, DISPLAY_PWM_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB, HOST_READY_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RADIO_POWER_GPIO_Port, RADIO_POWER_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, BATT_READ_Pin, GPIO_PIN_RESET);
  HAL_ADC_DeInit(&hadc);
  HAL_DBGMCU_DisableDBGStopMode();

  DEBUG_TX("Hello Computer\r\n");
  HAL_GPIO_WritePin(GPIOB, BATT_READ_Pin, GPIO_PIN_SET);
  Load_Normal_Screens();
  Initialise_States();
  ecog_init();

  Radio_On();
  /*
   RADIO_TXS("ER_CMD#D0", 9);
   status = Rx_Message(Rx_Buffer, &length, 3000);
   DEBUG_TX("Received: ");
   DEBUG_TXS(Rx_Buffer, 9);
   DEBUG_TX("\r\n");
   Delay_ms(100);
   RADIO_TXS("ACK", 3);
   Delay_ms(200);
   RADIO_TXS("ER_CMD#d0", 9);
   status = Rx_Message(Rx_Buffer, &length, 3000);
   DEBUG_TX("Received: ");
   DEBUG_TXS(Rx_Buffer, 9);
   DEBUG_TX("\r\n");
   Delay_ms(100);
   RADIO_TXS("ACK", 3);
   Delay_ms(200);
  */
  // Reset radio
  RADIO_TXS("ER_CMD#R0", 9);
  status = Rx_Message(Rx_Buffer, &length, 3000);
  DEBUG_TX("Received: ");
  DEBUG_TXS(Rx_Buffer, 9);
  DEBUG_TX("\r\n");
  Delay_ms(100);
  RADIO_TXS("ACK", 3);
  Delay_ms(200);

  // Set OTA data rate to 1200 bps
  RADIO_TXS("ER_CMD#B0", 9);
  status = Rx_Message(Rx_Buffer, &length, 3000);
  DEBUG_TX("Received: ");
  DEBUG_TXS(Rx_Buffer, 9);
  DEBUG_TX("\r\n");
  Delay_ms(100);
  RADIO_TXS("ACK", 3);
  Delay_ms(200);

  Radio_On();
  RADIO_TXS("Hello World", 11);
  DEBUG_TX("Sent Hello World\r\n");
  Delay_ms(50);
  for (i=0; i<128; i++)
	  Rx_Buffer[i] = 45;
  status = Rx_Message(Rx_Buffer, &length, 5000);
  if (status == HAL_OK)
  {
	  DEBUG_TX("Received: ");
	  DEBUG_TXS(Rx_Buffer, 32);
	  DEBUG_TX("\r\n");
	  sprintf(debug_buff, "Length %d\r\n", length);
	  DEBUG_TX(debug_buff);
  }
  else
  	  DEBUG_TX("Receive problem\r\n");

  Radio_Off();
  Set_Display(STATE_INITIAL, 0);
  HAL_UART_MspDeInit(&huart1);
  //HAL_SuspendTick();
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */

  while(1)
  {
	  /* Configures system clock after wake-up from STOP: enable HSE, PLL and select
	  PLL as system clock source (HSE and PLL are disabled in STOP mode) */
	  SYSCLKConfig_STOP();
	  HAL_UART_MspInit(&huart1);
	  DEBUG_TX("Main loop\r\n");
	  if(button_irq)
	  {
		  //DEBUG_TX("Button IRQ\r\n");
		  On_Button_IRQ(pressed_button, button_state);
	  }
	  else if(rtc_irq)
	  {
		  rtc_irq = 0;
		  DEBUG_TX("RTC IRQ\r\n");
		  if(MODE == ROLLING_DEMO)
			  On_Button_IRQ(ROLLING_DEMO_PIN, button_state);
		  else
			  On_RTC_IRQ();
	  }
	  button_irq = 0;
	  HAL_UART_MspDeInit(&huart1);
	  //HAL_PWR_EnterSTANDBYMode();
	  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
  }

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}
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

static void SYSCLKConfig_STOP(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  uint32_t pFLatency = 0;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* Get the Oscillators configuration according to the internal RCC registers */
  HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

  /* After wake-up from STOP reconfigure the system clock: Enable HSI and PLL */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
	  DEBUG_TX("Sysclock restart error 1\r\n");
  }

  /* Get the Clocks configuration according to the internal RCC registers */
  HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pFLatency);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType     = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource  = RCC_SYSCLKSOURCE_PLLCLK;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, pFLatency) != HAL_OK)
  {
	  DEBUG_TX("Sysclock restart error 2\r\n");
  }
}

static void SystemPower_Config(void)
{
  //GPIO_InitTypeDef GPIO_InitStructure = {0};
  /* Enable Ultra low power mode */
  HAL_PWREx_EnableUltraLowPower();
  /* Enable the fast wake up from Ultra low power mode */
  HAL_PWREx_EnableFastWakeUp();
  /* Enable GPIOs clock */
  /*
   __HAL_RCC_GPIOA_CLK_ENABLE();
   __HAL_RCC_GPIOB_CLK_ENABLE();
   __HAL_RCC_GPIOC_CLK_ENABLE();
   __HAL_RCC_GPIOD_CLK_ENABLE();
   __HAL_RCC_GPIOH_CLK_ENABLE();
   */
   /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
  /*
   GPIO_InitStructure.Pin = GPIO_PIN_All;
   GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
   GPIO_InitStructure.Pull = GPIO_NOPULL;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
   HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
   HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
   HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
   HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);
   */
   /* Disable GPIOs clock */
  /*
   __HAL_RCC_GPIOA_CLK_DISABLE();
   __HAL_RCC_GPIOB_CLK_DISABLE();
   __HAL_RCC_GPIOC_CLK_DISABLE();
   __HAL_RCC_GPIOD_CLK_DISABLE();
   __HAL_RCC_GPIOH_CLK_DISABLE();
   */
}


void Set_Display(uint8_t screen_num, uint8_t turn_on_radio)
{
	if (ecog_write_inverse(0))
	{
		ecog_cls();
		Build_Screen(screen_num);
		ecog_update_display(1, turn_on_radio);
		current_screen = screen_num;
	}
}

void Build_Screen(uint8_t screen_num)
{
	int pos = 0;
	int len;
	uint8_t f, x, y = 0;
	uint8_t *font = FONT_2;
	uint8_t w, h = 0;
	uint8_t region = 0;
	uint8_t parsing = 1;
	uint8_t loops = 0;

	sprintf(debug_buff,"Build_Screen %d\r\n", screen_num);
	DEBUG_TX(debug_buff);
	while(parsing)
	{
		if(strncmp(screens[screen_num][region]+pos, "S", 1) == 0)
		{
			pos += 2;
			//DEBUG_TX("Screen\r\n");

		}
		else if(strncmp(screens[screen_num][region]+pos, "R", 1) == 0)
		{
			pos += 2;
			//DEBUG_TX("Region\r\n");
		}
		else if(strncmp(screens[screen_num][region]+pos, "F", 1) == 0)
		{
			pos++; f = screens[screen_num][region][pos]; pos++;
			//sprintf(debug_buff,"Build_Screen, font: %d                             \r\n", f);
			//DEBUG_TX(debug_buff);
			switch(f)
			{
				case 1:
					font = FONT_2; break;
				case 2:
					font = FONT_2; break;
				case 3:
					font = FONT_3; break;
			}
		}
		else if(strncmp(screens[screen_num][region]+pos, "X", 1) == 0)
		{
			pos++; x = screens[screen_num][region][pos]; pos++;
			//sprintf(debug_buff,"Build_Screen, x: %d                             \r\n", x);
			//DEBUG_TX(debug_buff);
		}
		else if(strncmp(screens[screen_num][region]+pos, "Y", 1) == 0)
		{
			pos++; y = screens[screen_num][region][pos]; pos++;
			//sprintf(debug_buff,"Build_Screen, y: %d                             \r\n", y);
			//DEBUG_TX(debug_buff);
		}
		else if(strncmp(screens[screen_num][region]+pos, "B", 1) == 0)
		{
			pos++; w = screens[screen_num][region][pos]; pos++;
			h = screens[screen_num][region][pos]; pos++;
			//sprintf(debug_buff,"Build_Screen, box. w: %d, h: %d     \r\n", w, h);
			//DEBUG_TX(debug_buff);
			ecog_box(x, y, w, h,ECOG_ON);
		}
		else if(strncmp(screens[screen_num][region]+pos, "C", 1) == 0)
		{
			pos++; len = screens[screen_num][region][pos]; pos++;
			//sprintf(debug_buff,"Build_Screen, centred text, pos: %d, len: %d   \r\n", pos, len);
			//DEBUG_TX(debug_buff);
			ecog_printfc(font, y, screens[screen_num][region] + pos);
			//sprintf(debug_buff,"Build_Screen, text: %20s\r\n", screens[screen_num][region] + pos);
			//DEBUG_TX(debug_buff);
			pos += len + 1;
			//sprintf(debug_buff,"Build_Screen, pos: %d, next 3 chars: %3s\r\n", pos, screens[screen_num][region] + pos);
			//DEBUG_TX(debug_buff);
		}
		else if(strncmp(screens[screen_num][region]+pos, "l", 1) == 0)
		{
			pos++; len = screens[screen_num][region][pos]; pos++;
			//sprintf(debug_buff,"Build_Screen, centred left text, pos: %d, len: %d   \r\n", pos, len);
			//DEBUG_TX(debug_buff);
			ecog_printfl(font, y, screens[screen_num][region] + pos);
			//sprintf(debug_buff,"Build_Screen, text left: %20s\r\n", screens[screen_num][region] + pos);
			//DEBUG_TX(debug_buff);
			pos += len + 1;
			//sprintf(debug_buff,"Build_Screen, pos: %d, next 3 chars: %3s\r\n", pos, screens[screen_num][region] + pos);
			//DEBUG_TX(debug_buff);
		}
		else if(strncmp(screens[screen_num][region]+pos, "r", 1) == 0)
		{
			pos++; len = screens[screen_num][region][pos]; pos++;
			//sprintf(debug_buff,"Build_Screen, centred right text, pos: %d, len: %d   \r\n", pos, len);
			//DEBUG_TX(debug_buff);
			ecog_printfr(font, y, screens[screen_num][region] + pos);
			//sprintf(debug_buff,"Build_Screen, right text: %20s\r\n", screens[screen_num][region] + pos);
			//DEBUG_TX(debug_buff);
			pos += len + 1;
			//sprintf(debug_buff,"Build_Screen, pos: %d, next 3 chars: %3s\r\n", pos, screens[screen_num][region] + pos);
			//DEBUG_TX(debug_buff);
		}
		else if(strncmp(screens[screen_num][region]+pos, "T", 1) == 0)
		{
			ecog_position(x, y);
			pos++; len = screens[screen_num][region][pos]; pos++;
			//sprintf(debug_buff,"Build_Screen, text, pos: %d, len: %d   \r\n", pos, len);
			//DEBUG_TX(debug_buff);
			ecog_printf(font, screens[screen_num][region] + pos);
			//sprintf(debug_buff,"Build_Screen, text: %20s\r\n", screens[screen_num][region] + pos);
			//DEBUG_TX(debug_buff);
			pos += len + 1;
			//sprintf(debug_buff,"Build_Screen, pos: %d, next 3 chars: %3s\r\n", pos, screens[screen_num][region] + pos);
			//DEBUG_TX(debug_buff);
		}
		else if(strncmp(screens[screen_num][region]+pos, "E", 1) == 0)
		{
			//DEBUG_TX("Build_Screen found E\r\n");
			pos++;
			if(strncmp(screens[screen_num][region]+pos, "R", 1) == 0)
			{
				pos++; region++;
				loops = 0;
				//DEBUG_TX("Build_Screen end region\r\n");
			}
			else if(strncmp(screens[screen_num][region]+pos, "S", 1) == 0)
			{
				parsing = 0;
				//DEBUG_TX("Build_Screen end screen\r\n");
			}
        }
		loops++;
		if(loops >127)
		{
			parsing = 0;
			//DEBUG_TX("Build_Screen did not find end of screen\r\n");
		}
	}
}

uint8_t On_Button_Press(uint16_t GPIO_Pin, 	GPIO_PinState button_state)
{
	int 				side 					= LEFT_SIDE;
	uint32_t 			pressed_time 			= 0;
	uint32_t 			now;
	int 				i;
	int					en_double				= 0;

	if(current_state != STATE_DEMO)
		if((states[current_state][S_LD] != 0xFF) | (states[current_state][S_RD] != 0xFF) | (states[current_state][S_MD] != 0xFF))
		{
			en_double = 1;
			DEBUG_TX("Double enabled\r\n");
		}
	if (GPIO_Pin == PUSH_RIGHT_Pin)
	{
		side = RIGHT_SIDE;
		//sprintf(debug_buff,"Right button event: %d\r\n", (int)Cbr_Now());
		//DEBUG_TX(debug_buff);
	}
	else if (GPIO_Pin == LEFT_PUSH_Pin)
	{
		side = LEFT_SIDE;
		//sprintf(debug_buff,"Left button event: %d\r\n", (int)Cbr_Now());
		//DEBUG_TX(debug_buff);
	}
	else
	{
		//DEBUG_TX("Unknown button event\r\n");
		return PRESS_NONE;
	}

	now = Cbr_Now();
	button_state = HAL_GPIO_ReadPin(GPIOA, GPIO_Pin);
	if (button_state == GPIO_PIN_RESET)
	{
		if(en_double)
		{
			if((now - button_press_time[side]) < 2)
			{
				if(side == LEFT_SIDE)
				{
					//button_press_time[side] = now;  // So that we can catch long presses
					return PRESS_LEFT_DOUBLE;
				}
				else
				{
					//button_press_time[side] = now;  // So that we can catch long presses
					return PRESS_RIGHT_DOUBLE;
				}
			}
			button_press_time[side] = now;
			return PRESS_NONE;
		}
		else
		{
			button_press_time[side] = now;
			if(side == LEFT_SIDE)
				return PRESS_LEFT_SINGLE;
			else
				return PRESS_RIGHT_SINGLE;
		}
	}
	else if(button_state == GPIO_PIN_SET)
	{
		pressed_time = now - button_press_time[side];
		if((pressed_time > T_RESET_PRESS) & (pressed_time < T_MAX_RESET_PRESS))
		{
			DEBUG_TX("System reset\r\n");
			NVIC_SystemReset();
		}
		else if(pressed_time > T_LONG_PRESS)
		{
			if(side == LEFT_SIDE)
				return PRESS_LEFT_LONG;
			else
				return PRESS_RIGHT_LONG;
		}
	}
	return PRESS_NONE; // Should not ever get here, but just in case
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	pressed_button = GPIO_Pin;
	button_state = __HAL_GPIO_EXTI_GET_IT(GPIO_Pin);
	button_irq = 1;
	return;
}

void On_Button_IRQ(uint16_t GPIO_Pin, GPIO_PinState button_state)
{
	uint8_t button_press;

	if(GPIO_Pin == ROLLING_DEMO_PIN)
		button_press = PRESS_DEMO;
	else
	{
		button_press = On_Button_Press(GPIO_Pin, button_state);
		//sprintf(debug_buff, "button_press: %d\r\n", button_press);
		//DEBUG_TX(debug_buff);
	}

	if(button_press == PRESS_NONE)
	{
		return;
	}
	else if(current_state == STATE_INITIAL)
	{
		DEBUG_TX("node_state is initial\r\n");
		if ((button_press == PRESS_RIGHT_LONG) | (button_press == PRESS_LEFT_LONG))
		{
			Network_Include();
		}
		else
		{
			DEBUG_TX("Starting demo mode\r\n");
			Radio_Off();
			Load_Demo_Screens();
			screen_num = 0;
			button_irq = 0;
			Set_Display(0, 0);
			current_state = STATE_DEMO;
			return;
		}
	}
	else if(current_state == STATE_DEMO)
	{
		DEBUG_TX("Node state demo\r\n");
		if ((button_press == PRESS_RIGHT_LONG) || (button_press == PRESS_LEFT_LONG)
			|| (button_press == PRESS_RIGHT_DOUBLE) || (button_press == PRESS_LEFT_DOUBLE))
			screen_num = 0;
		else
			screen_num++;
		if (screen_num == MAX_SCREEN)
			screen_num = 0;
		button_irq = 0;
		Set_Display(screen_num, 0);
		if ((screen_num == 9) || (screen_num == 16) || (screen_num == 23))
		{
			Delay_ms(2500);
			screen_num++;
			Set_Display(screen_num, 0);
		}
		if(MODE == ROLLING_DEMO)
			RTC_Delay(5);
	}
	else
	{
		switch(button_press)
		{
			case PRESS_LEFT_SINGLE:
				if(states[current_state][S_LS] != 0xFF)
					current_state = states[current_state][S_LS];
				break;
			case PRESS_LEFT_DOUBLE:
				if(states[current_state][S_LD] != 0xFF)
					current_state = states[current_state][S_LD];
				break;
			case PRESS_RIGHT_SINGLE:
				if(states[current_state][S_RS] != 0xFF)
					current_state = states[current_state][S_RS];
				break;
			case PRESS_RIGHT_DOUBLE:
				if(states[current_state][S_RD] != 0xFF)
					current_state = states[current_state][S_RD];
				break;
			default:
				DEBUG_TX("Warning. Undefined button_press\r\n");
				current_state = states[current_state][0];
		}
		On_NewState();
	}
}

void On_NewState(void)
{
	uint8_t alert_id[] = {0x00, 0x00};
	sprintf(debug_buff, "On_NewState, state: %d       \r\n", current_state);
	DEBUG_TX(debug_buff);
	if(states[current_state][S_A] != 0xFF)
	{
		Radio_On();
		button_irq = 0;  // So other functions can check if it's been pressed again
		alert_id[1] = states[current_state][S_A]; alert_id[0] = 0x00;
		sprintf(debug_buff, "Sending alert: %x %x       \r\n", alert_id[0], alert_id[1]);
		DEBUG_TX(debug_buff);
		Send_Message(f_alert, 2, alert_id, 1, 0);
	}
	else  // If a message is sent, display is changed as part of Manage_Send(), otherwise change here
	{
		if(states[current_state][S_D] != 0xFF)
			Set_Display(states[current_state][S_D], 0);
	}
}

void Power_Down(void)
{
	DEBUG_TX("Powering down\r\n");
	Delay_ms(100);
	/*
	__HAL_RCC_GPIOA_CLK_DISABLE();
	__HAL_RCC_GPIOB_CLK_DISABLE();
	__HAL_RCC_GPIOC_CLK_DISABLE();
	__HAL_RCC_GPIOD_CLK_DISABLE();
	__HAL_RCC_GPIOH_CLK_DISABLE();
	*/
	HAL_UART_MspDeInit(&huart1);
	//HAL_SuspendTick();
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	DEBUG_TX("Powered down\r\n");
}

void Network_Include(void)
{
	uint8_t equal = 1;
	int i;
	DEBUG_TX("Network_Include\r\n");
	if(include_state == 0)
	{
		Set_Display(STATE_CONNECTING, 1);
		include_state = 1;
	}
	else if(include_state == 1)
	{
		Set_Display(STATE_PROBLEM, 1);
		include_state = 2;
	}
	Radio_On();
	if(Message_Search(beacon_address, Rx_Buffer, BEACON_SEARCH_TIME) == SEARCH_OK)
	{
		bridge_address[0] = Rx_Buffer[2]; bridge_address[1] = Rx_Buffer[3];
		Send_Message(f_include_req, 4, node_id, 0, 0);
	}
	else
	{
		switch(include_state)
		{
			case 1:
				Radio_Off();
				//RTC_Delay(10*60);
				RTC_Delay(30);
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
	if(Message_Search(grant_address, Rx_Buffer, BEACON_SEARCH_TIME) == SEARCH_OK)
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
			uint8_t data[] = {0x00, 0x00};
			Send_Message(f_ack, 0, data, 0, 0);
			DEBUG_TX("Sent ack for grant\r\n");
			include_state = 0;
			current_state = STATE_CONFIG;
			On_NewState();
			DEBUG_TX("Sending woken_up after grant\r\n");
			Send_Message(f_woken_up, 0, data, 1, 0);
		}
	}
	else
	{
		DEBUG_TX("Network_Include. Did not receive grant\r\n");
		RTC_Delay(30);
	}
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
	while (radio_ready != SET)
	{
	}
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
    HAL_GPIO_WritePin(GPIOB, HOST_READY_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RADIO_POWER_GPIO_Port, RADIO_POWER_Pin, GPIO_PIN_SET);
    HAL_UART_MspInit(&huart3);
    Delay_ms(400);
    HAL_GPIO_WritePin(GPIOB, HOST_READY_Pin, GPIO_PIN_RESET);
}

void Radio_Off(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	DEBUG_TX("Radio_Off\r\n");
	HAL_UART_MspDeInit(&huart3);
    HAL_GPIO_WritePin(RADIO_POWER_GPIO_Port, RADIO_POWER_Pin, GPIO_PIN_RESET);
    Delay_ms(5);
	__GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = RADIO_BUSY_Pin|RADIO_CARRIER_DETECT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	//__GPIOB_CLK_DISABLE();
}

void Send_Message(uint8_t function, uint8_t data_length, uint8_t *data, uint8_t ack, uint8_t beacon)
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
	Manage_Send(ack, beacon, function);
}

void Manage_Send(uint8_t ack, uint8_t beacon, uint8_t function)
{
	static uint8_t local_function;
	sprintf(debug_buff, "Manage_Send, send_attempt: %d                          \r\n", send_attempt);
	DEBUG_TX(debug_buff);
	if(send_attempt == 4)
		Radio_On();
	send_attempt++;
	if(function != f_unknown)
		local_function = function;
	while(HAL_GPIO_ReadPin(GPIOB, RADIO_BUSY_Pin == GPIO_PIN_SET))
	{
	}
	if(beacon)
	{
		if(Message_Search(beacon_address, Rx_Buffer, BEACON_SEARCH_TIME) == SEARCH_OK)
		{
			DEBUG_TX("Manage_Send found beacon\r\n");
			Send_Delay();
		}
		else
		{
			DEBUG_TX("Manage_Send did not find beacon\r\n");
			// *** Left in high power state but changed to add RTC_Delay
			RTC_Delay(10);
			return;
		}
	}
	RADIO_TXS(tx_message, tx_length);
	DEBUG_TX("Manage_Send sent message\r\n");
	if(ack)
	{
		DEBUG_TX("Manage_Send waiting for ack\r\n");
		if(Message_Search(node_address, Rx_Buffer, ACK_SEARCH_TIME) == SEARCH_OK)
		{
			if(Rx_Buffer[4] == f_ack)
			{
				send_attempt = 0;
				DEBUG_TX("Manage_Send ack received\r\n");
				if((states[current_state][S_D] != 0xFF) & (local_function != f_woken_up))
				{
					uint32_t wakeup = ((Rx_Buffer[10] << 8) | Rx_Buffer[11]) << 1;
					if(wakeup != 0)
						Radio_Off();  // Turn radio off ASAP, before updating display
					Set_Display(states[current_state][S_D], 0);
				}
				Set_Wakeup(0);
			}
			else
			{
				// If we get a message that is not an ack when expecting an ack, clear it by sending ack back.
				DEBUG_TX("Manage_Send another message received when expecting ack\r\n");
				uint8_t data[] = {0x00, 0x00};
				Send_Message(f_ack, 0, data, 0, 0);
				Set_Wakeup(1);
			}
		}
		else
		{
			DEBUG_TX("Manage_Send no ack 1\r\n");
			switch(send_attempt)
			{
				case 1:
					if (function != f_woken_up)
						Set_Display(STATE_SENDING, 1);
				case 2:
				case 3:
				case 5:
					Send_Delay();
				case 6:
					Manage_Send(1, beacon, local_function);
					break;
				case 4:
					Radio_Off();
					RTC_Delay(30);
					break;
				case 7:
					DEBUG_TX("Manage_Send permanent failure\r\n");
					Set_Display(STATE_PROBLEM, 0);
					Radio_Off();
					RTC_Delay(600);
			}
		}
	}
	else        // Don't wait for an ack, but give time for message to be sent
	{
		Delay_ms(200);
	}
}

void Listen_Radio(void)
{
	static uint8_t fail_count = 0;
	DEBUG_TX("Listen_Radio\r\n");
	if(Message_Search(node_address, Rx_Buffer, ACK_SEARCH_TIME) == SEARCH_OK)
	{
		fail_count = 0;
		uint8_t data[] = {0x00, 0x00};
		if(Rx_Buffer[4] == f_config)
		{
			DEBUG_TX("Listen_Radio. Config message received\r\n");
			if(MODE != RATE)
				Store_Config();
			Send_Message(f_ack, 0, data, 0, 0);
			Set_Wakeup(0);
		}
		else if(Rx_Buffer[4] == f_start)
		{
			DEBUG_TX("Listen_Radio. Start received\r\n");
			current_state = 0;
			On_NewState();
			Send_Message(f_ack, 0, data, 0, 0);
			Set_Wakeup(0);
		}
		else if(Rx_Buffer[4] == f_send_battery)
		{
			DEBUG_TX("Listen_Radio. Send battery received\r\n");
			Send_Message(f_ack, 0, data, 0, 0);
			Send_Battery();
			Set_Wakeup(0);
		}
		else if(Rx_Buffer[4] == f_ack)
		{
			DEBUG_TX("Listen_Radio. Ack received\r\n");
			Set_Wakeup(0);
		}
		else
		{
			DEBUG_TX("Listen_Radio. Unknown message, sending ack\r\n");
			Send_Message(f_ack, 0, data, 0, 0);
			Set_Wakeup(1);
		}
	}
	else
	{
		DEBUG_TX("Listen_Radio. Search failed\r\n");
		if(fail_count < 4)
		{
			fail_count++;
			Set_Wakeup(0);
		}
		else  // Catches case where we miss an ack and last wakeup was 0
		{
			fail_count = 0;
			Radio_Off();
			RTC_Delay(30);
		}
	}
}

void Set_Wakeup(uint8_t error)
{
	DEBUG_TX("Set_Wakeup\r\n");
	if(error)
	{
		DEBUG_TX("Set_Wakeup, error\r\n");
		Radio_Off();    // Just in case
		RTC_Delay(30);  // On error wake up in 30 secs, when woken_up will be sent, resetting communications
	}
	else
	{
		uint32_t wakeup = ((Rx_Buffer[10] << 8) | Rx_Buffer[11]) << 1;
		if(wakeup > ONE_DAY)
			wakeup = ONE_DAY;
		if(wakeup == 0)
		{
			DEBUG_TX("Set_Wakeup, calling Listen_Radio\r\n");
			Listen_Radio();
		}
		else
		{
			sprintf(debug_buff, "Set_Wakeup for %d\r\n", (int)wakeup);
			DEBUG_TX(debug_buff);
			Radio_Off();
			RTC_Delay(wakeup);
		}
	}
}

void Store_Config(void)
{
	uint8_t s, r, i = 0;
	uint8_t pos = 12;
	DEBUG_TX("Store_Config\r\n");
	if(strncmp(Rx_Buffer+pos, "S", 1) == 0)
	{
		s = Rx_Buffer[pos+1];
		if(strncmp(Rx_Buffer+pos+2, "R", 1) == 0)
			r = Rx_Buffer[pos+3];
		//sprintf(debug_buff, "Screen: %d, region: %d\r\n", s, r);
		//DEBUG_TX(debug_buff);
		//for(i=0; i<64; i++)
		//	debug_buff[i] = 0;
		for(i=0; i<Rx_Buffer[5]; i++)
		{
			screens[s][r][i] = Rx_Buffer[i+pos+4];
			//sprintf(debug_buff, "%c", screens[s][r][i]);
			//DEBUG_TX(debug_buff);
		}
		DEBUG_TX("\r\n");
		//sprintf(debug_buff, "Store_Config, screen: %d, reg: %d, %02x %02x %02x %02x %02x %02x\r\n", s, r, screens[s][r][0], screens[s][r][1], screens[s][r][2], screens[s][r][3], screens[s][r][4], screens[s][r][5]);
		//DEBUG_TX(debug_buff);
		config_stored++;
		//sprintf(debug_buff, "Store_Config, config_stored: %d\r\n", config_stored);
		//DEBUG_TX(debug_buff);
		if(s == current_screen)
		{
			DEBUG_TX("Store_Config. Current screen updated\r\n");
			Set_Display(current_screen, 0);
		}
	}
	else if(strncmp(Rx_Buffer+pos, "C", 1) == 0)
	{
		app_value = Rx_Buffer[pos+1];
		sprintf(debug_buff, "Application value updated:%d\r\n", app_value);
		DEBUG_TX(debug_buff);
		if(states[current_state][S_XV] == app_value)
		{
			current_state = states[current_state][S_XS];
			On_NewState();
		}
	}
	else if(strncmp(Rx_Buffer+pos, "M", 1) == 0)
	{
		DEBUG_TX("State config\r\n");
		for(i=0; i<64; i++)
			debug_buff[i] = 0x20;
		s = Rx_Buffer[pos+1];
		for(i=0; i<16; i++)
		{
			states[s][i] = Rx_Buffer[i+pos+1];
		}
		/*
		for(i=0; i<16; i++)
		{
			sprintf(debug_buff, "%x ", states[s][i]);
		 	DEBUG_TX(debug_buff);
		 	DEBUG_TX("\r\n");
		}
		*/
	}
}

void Send_Battery()
{
	uint32_t adc_value = 0x55;
	uint8_t alert[] = {0x02, 0x00};
	MX_ADC_Init();
	HAL_GPIO_WritePin(GPIOB, BATT_READ_Pin, GPIO_PIN_SET);
	Delay_ms(100);
	if(HAL_ADC_Start(&hadc) != HAL_OK)
	{
		DEBUG_TX("ADC start error\r\n");
		HAL_ADC_DeInit(&hadc);
		HAL_GPIO_WritePin(GPIOB, BATT_READ_Pin, GPIO_PIN_RESET);
		return;
	}
	if(HAL_ADC_PollForConversion(&hadc, 50) != HAL_OK)
	{
		DEBUG_TX("ADC conversion error\r\n");
		HAL_ADC_Stop(&hadc);
		HAL_ADC_DeInit(&hadc);
		HAL_GPIO_WritePin(GPIOB, BATT_READ_Pin, GPIO_PIN_RESET);
	}
	else
	{
		adc_value = HAL_ADC_GetValue(&hadc);
		HAL_ADC_Stop(&hadc);
		HAL_ADC_DeInit(&hadc);
		HAL_GPIO_WritePin(GPIOB, BATT_READ_Pin, GPIO_PIN_RESET);
		sprintf(debug_buff, "ADC battery value: %x\r\n", (int)adc_value);
		DEBUG_TX(debug_buff);
		alert[1] = adc_value >> 8;
		sprintf(debug_buff, "Sending alert: %x %x       \r\n", alert[0], alert[1]);
		DEBUG_TX(debug_buff);
		Send_Message(f_alert, 2, alert, 1, 0);
	}

	return;
}


uint8_t Message_Search(uint8_t *address, uint8_t *Rx_Buffer, uint16_t max_search_time)
{
	int search_start = Cbr_Now();
	sprintf(debug_buff, "Message_Search, address: %02x %02x\r\n", address[0], address[1]);
	DEBUG_TX(debug_buff);
	while (1)
	{
		__HAL_UART_FLUSH_DRREGISTER(&huart3);
		if(current_state != STATE_INITIAL) // So we can display "sending" message if there is no immediate ack
			status = Rx_Message(Rx_Buffer, &length, 1500);
		else
			status = Rx_Message(Rx_Buffer, &length, 6000);
		if (status == HAL_OK)
		{
			//sprintf(debug_buff, "MS Rx: %02x %02x %02x %02x %02x %02x %02x %02x, Length: %d\r\n", Rx_Buffer[0], Rx_Buffer[1],
			//					Rx_Buffer[2], Rx_Buffer[3], Rx_Buffer[4], Rx_Buffer[5], Rx_Buffer[10], Rx_Buffer[11], length);
			//DEBUG_TX(debug_buff);
			if((Rx_Buffer[0] == address[0]) & (Rx_Buffer[1] == address[1]))
			{
				return SEARCH_OK;
			}
			if ((Cbr_Now() - search_start) > max_search_time)
			{
				DEBUG_TX("Message_Search timeout\r\n");
				return SEARCH_TIMEOUT;
			}
		}
		else
		{
			__HAL_UART_FLUSH_DRREGISTER(&huart3);
			DEBUG_TX("Message_Search status != HAL_OK\r\n");
			if ((Cbr_Now() - search_start) > max_search_time)
			{
				DEBUG_TX("Message_Search 2nd timeout error\r\n");
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

void RTC_TimeShow(void)
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

void Send_Delay(void)
{
	/*
	At 1 Kbps it takes 16*8 = 128 ms to send a message.
	Delay between 0 and 3 x 128 ms.
	*/
	uint32_t delay = (Cbr_Now() & 0x3) * 128;
	sprintf(debug_buff,"Send_Delay: %d\r\n", (int)delay);
	DEBUG_TX(debug_buff);
	Delay_ms(delay);
}


void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	rtc_irq = 1;
	return;
}

void On_RTC_IRQ(void)
{
	DEBUG_TX("\rRTC IRQ\r\n");
	if(include_state != 0)
	{
		Network_Include();
	}
	else if(send_attempt == 4)
		Manage_Send(1, 0, f_unknown);
	else
	{
		DEBUG_TX("Woken up\r\n");
		Radio_On();
		uint8_t data[] = {0x00, 0x00};
		Send_Message(f_woken_up, 0, data, 1, 0);
	}
}

void Load_Normal_Screens(void)
{
	// Not very elegant, but it does the job and it shouldn't need to change:
	int i, s;
	strcpy(screens[18][0], "F\x03" "Y\x10" "C\x07" "Sending\xFF" "Y\x32" "C\x0F" "Please wait ...\xFF" "ES");
	strcpy(screens[19][0], "F\x02" "Y\x04" "C\x0F" "Welcome to Spur\xFF" "Y\x1A" "C\x0F" "Push here for 3\xFF"
			              "Y\x30" "C\x12" "seconds to connect\xFF" "Y\x46" "C\x0A" "to network\xFF" "ES");
	strcpy(screens[20][0], "F\x03" "Y\x05" "C\x0A" "Connecting\xFF" "Y\x20" "C\x0A" "to network\xFF" "Y\x3D"
						   "C\x0B" "Please wait\xFF" "ES");
	strcpy(screens[21][0], "F\x02" "Y\x05" "C\x09" "Connected\xFF" "Y\x20" "C\x0B" "Configuring\xFF" "Y\x3D"
			              	 "C\x0B" "Please wait\xFF" "ES");
	strcpy(screens[22][0], "F\x02" "Y\x10" "C\x0B" "Spur button\xFF" "Y\x32" "C\x0E" "name not known\xFF" "ES");
	strcpy(screens[23][0], "F\x03" "Y\x05" "C\x0D" "Communication\xFF" "Y\x20" "C\x07" "problem\xFF" "Y\x3D"
		              	  "C\x0A" "Not in use\xFF" "ES");
	for(s=18; s<24; s++)
		for(i=0; i<128; i++)
			if(screens[s][0][i] == 0xFF)
				screens[s][0][i] = 0x00;
}

void Load_Demo_Screens(void)
{
	int i, s;
	strcpy(screens[0][0], "F\x03" "Y\x05" "C\x0B" "This is the\xFF" "Y\x20" "C\x09" "Spur demo\xFF"
	                	   "Y\x3D" "C\x1D" "push to begin\xFF" "ES");
	strcpy(screens[1][0], "F\x03" "Y\x10" "C\x0C" "Push here to\xFF" "Y\x32" "C\x0E" "report a fault\xFF" "ES");
	strcpy(screens[2][0], "F\x03" "Y\x05" "C\x0F" "Fault with this\xFF" "Y\x20" "C\x0D" "appliance has\xFF"
	                      "Y\x3D" "C\x0D" "been reported\xFF" "ES");
	strcpy(screens[3][0], "F\x03" "Y\x05" "C\x09" "Push here\xFF" "Y\x20" "C\x09" "for table\xFF" "Y\x3D" "C\x07" "service\xFF" "ES");
	strcpy(screens[4][0], "F\x03" "Y\x05" "C\x08" "A waiter\xFF" "Y\x20" "C\x0C" "will be with\xFF" "Y\x3D" "C\x08" "you soon\xFF" "ES");
	strcpy(screens[5][0], "F\x02" "X\x18" "Y\x0E" "T\x04" "Push\xFF" "X\x08" "Y\x24" "T\x08" "here for\xFF"
	                      "X\x20" "Y\x3C" "T\x04" "bill\xFF" "F\x02" "X\x7D" "Y\x0E" "T\x04" "Push\xFF" "X\x6D" "Y\x24"
	                      "T\x08" "here for\xFF" "X\x70" "Y\x3C" "T\x07" "service\xFF"
			              "X\x02" "Y\x02" "B\x5A\x5C" "X\x03" "Y\x03" "B\x58\x5A"
	          	  	  	  "X\x68" "Y\x02" "B\x5A\x5C" "X\x69" "Y\x03" "B\x58\x5A"
			              "ES");
	strcpy(screens[6][0], "F\x02" "Y\2" "C\x0D" "Do you have a\xFF" "Y\x16" "C\x0D" "loyalty card?\xFF"
	 		               "F\x03" "X\x1C" "Y\x39" "T\x02" "No\xFF"
	                       "F\x03" "X\x7A" "Y\x39" "T\x03" "Yes\xFF"
	 		               "X\x02" "Y\x2D" "B\x5A\x32" "X\x03" "Y\x2E" "B\x5A\x32"
	 		               "X\x68" "Y\x2D" "B\x5A\x32" "X\x69" "Y\x2E" "B\x5A\x32" "ES");
	strcpy(screens[7][0], "F\x03" "Y\x05" "C\x08" "A waiter\xFF" "Y\x20" "C\x0C" "will be with\xFF" "Y\x3D" "C\x08" "you soon\xFF" "ES");
	strcpy(screens[8][0], "F\x02" "Y\2" "C\x0B" "How was our\xFF" "Y\x16" "C\x0E" "service today?\xFF"
			              "X\x08" "Y\x31" "T\x08" "Push for\xFF"
			  	  	  	  "X\x1A" "Y\x47" "T\x03" "bad\xFF" "X\x6D" "Y\x31" "T\x08" "Push for\xFF" "X\x7E"
			              "Y\x47" "T\x04" "good\xFF" "X\x02" "Y\x2D" "B\x5A\x32" "X\x03" "Y\x2E" "B\x5A\x32"
			              "X\x68" "Y\x2D" "B\x5A\x32" "X\x69" "Y\x2E" "B\x5A\x32" "ES");
	strcpy(screens[9][0], "F\x03" "Y\x10" "C\x0D" "Thank you for\xFF" "Y\x32" "C\x0D" "your feedback\xFF" "ES");
	strcpy(screens[10][0], "F\x02" "Y\2" "C\x0B" "How was our\xFF" "Y\x16" "C\x0E" "service today?\xFF"
			               "X\x08" "Y\x31" "T\x08" "Push for\xFF"
			  	  	   	   "X\x1A" "Y\x47" "T\x03" "bad\xFF" "X\x6D" "Y\x31" "T\x08" "Push for\xFF" "X\x7E"
			               "Y\x47" "T\x04" "good\xFF" "X\x02" "Y\x2D" "B\x5A\x32" "X\x03" "Y\x2E" "B\x5A\x32"
			               "X\x68" "Y\x2D" "B\x5A\x32" "X\x69" "Y\x2E" "B\x5A\x32" "ES");
	strcpy(screens[11][0], "F\x02" "Y\x05" "C\x13" "Push here to report\xFF" "Y\x20" "C\x0E" "a problem with\xFF"
	  		               "Y\x3D" "C\x10" "these facilities\xFF" "ES");
	strcpy(screens[12][0], "F\x02" "Y\x05" "C\x0C" "Problem with\xFF" "Y\x20" "C\x10" "these facilities\xFF"
	    		           "Y\x3D" "C\x11" "has been reported\xFF" "ES");
	strcpy(screens[13][0], "F\x03" "Y\x05" "C\x09" "Push here\xFF" "Y\x20" "C\x0A" "to request\xFF"
	    		           "Y\x3D" "C\x06" "a taxi\xFF" "ES");
	strcpy(screens[14][0], "F\x02" "Y\x04" "C\x0E" "Taxi requested\xFF" "Y\x1A" "C\x11" "Taxi will be here\xFF"
			               "Y\x30" "C\x0F" "at approx 11:35\xFF" "Y\x46" "C\x14" "Push again to cancel\xFF" "ES");
	strcpy(screens[15][0], "F\x02" "Y\x05" "C\x0F" "Push to send an\xFF" "Y\x22" "C\x10" "AV technician to\xFF"
	    		           "Y\x3D" "C\x0E" "meeting room 1\xFF" "ES");
	strcpy(screens[16][0], "F\x03" "Y\x10" "C\x0D" "AV technician\xFF" "Y\x32" "C\x09" "requested\xFF" "ES");
	strcpy(screens[17][0], "F\x03" "Y\x05" "C\x05" "An AV\xFF" "Y\x20" "C\x0D" "technician is\xFF"
			               "Y\x3D" "C\x0C" "on their way\xFF" "ES");
	strcpy(screens[18][0], "F\x03" "Y\x10" "C\x0C" "Push here if\xFF" "Y\x32" "C\x09" "you're OK\xFF" "ES");
	strcpy(screens[19][0], "F\x03" "Y\x05" "C\x0C" "Thanks. I'll\xFF" "Y\x20" "C\x0D" "call you this\xFF" "Y\x3D" "C\x0A" "PM - Peter\xFF" "ES");
	strcpy(screens[20][0], "F\x02" "Y\x05" "C\x0C" "Push here to\xFF" "Y\x20" "C\x0C" "request more\xFF"
	      		           "Y\x3D" "C\x0F" "coffee capsules\xFF" "ES");
	strcpy(screens[21][0], "F\x02" "Y\x05" "C\x0B" "More coffee\xFF" "Y\x20" "C\x0D" "capsules have\xFF"
	      		           "Y\x3D" "C\x0E" "been requested\xFF" "ES");
	strcpy(screens[22][0], "F\x02" "Y\x05" "C\x0C" "Push here if\xFF" "Y\x20" "C\x10" "you want a carer\xFF"
	       		           "Y\x3D" "C\x0C" "to visit you\xFF" "ES");
	strcpy(screens[23][0], "F\x03" "Y\x10" "C\x0C" "Your request\xFF" "Y\x32" "C\x0D" "has been sent\xFF" "ES");
	strcpy(screens[24][0], "F\x02" "Y\x05" "C\x0C" "A carer will\xFF" "Y\x20" "C\x11" "visit you between\xFF"
	         		       "Y\x3D" "C\x0F" "11:00 and 12:00\xFF" "ES");
	strcpy(screens[25][0], "F\x02" "Y\2" "C\x0B" "Are you OK?\xFF" "Y\x16" "C\x11" "Please push below\xFF"
	 		               "F\x03" "X\x1C" "Y\x39" "T\x02" "No\xFF"
	                       "F\x03" "X\x7A" "Y\x39" "T\x03" "Yes\xFF"
	 		               "X\x02" "Y\x2D" "B\x5A\x32" "X\x03" "Y\x2E" "B\x5A\x32"
	 		               "X\x68" "Y\x2D" "B\x5A\x32" "X\x69" "Y\x2E" "B\x5A\x32" "ES");
	strcpy(screens[26][0], "F\x02" "Y\x05" "C\x0C" "Push here if\xFF" "Y\x20" "C\x10" "printer supplies\xFF"
	        		       "Y\x3D" "C\x07" "are low\xFF" "ES");
	strcpy(screens[27][0], "F\x02" "X\x18" "Y\x0E" "T\x04" "Push\xFF" "X\x08" "Y\x24" "T\x08" "here for\xFF"
	 		               "X\x16" "Y\x3C" "T\x05" "paper\xFF" "X\x7D" "Y\x0E" "T\x04" "Push\xFF" "X\x6D" "Y\x24"
	 		               "T\x08" "here for\xFF" "X\x7B" "Y\x3C" "T\x05" "toner\xFF"
	 		               "X\x02" "Y\x02" "B\x5A\x5C" "X\x03" "Y\x03" "B\x58\x5A"
	           	  	  	   "X\x68" "Y\x02" "B\x5A\x5C" "X\x69" "Y\x03" "B\x58\x5A"
	 		               "ES");
	strcpy(screens[28][0], "F\x02" "X\x18" "Y\x0E" "T\x04" "Push\xFF" "X\x08" "Y\x24" "T\x08" "here for\xFF"
	  		               "X\x16" "Y\x3C" "T\x05" "black\xFF" "X\x7D" "Y\x0E" "T\x04" "Push\xFF" "X\x6D" "Y\x24"
	  		               "T\x08" "here for\xFF" "X\x6D" "Y\x3C" "T\x07" "Y/Cy/Mg\xFF"
	  		               "X\x02" "Y\x02" "B\x5A\x5C" "X\x03" "Y\x03" "B\x58\x5A"
	            	  	   "X\x68" "Y\x02" "B\x5A\x5C" "X\x69" "Y\x03" "B\x58\x5A"
	  		               "ES");
	strcpy(screens[29][0], "F\x02" "Y\x04" "C\x0F" "Low black toner\xFF" "Y\x1A" "C\x08" "reported\xFF"
	 		               "Y\x30" "C\x12" "Push here if other\xFF" "Y\x46" "C\x14" "printer supplies low\xFF" "ES");
	strcpy(screens[30][0], "F\x03" "Y\x10" "C\x0C" "Push here if\xFF" "Y\x32" "C\x0D" "you need help\xFF" "ES");
	strcpy(screens[31][0], "F\x03" "Y\x10" "C\x0D" "I'm on my way\xFF" "Y\x32" "C\x07" "- Sarah\xFF" "ES");

	for(s=0; s<32; s++)
		for(i=0; i<128; i++)
			if(screens[s][0][i] == 0xFF)
				screens[s][0][i] = 0x00;
}

void Initialise_States(void)
{
    //                               S   D    A   LD    LS   MS  MD   RS   RD   XV   XS    W   WS
	static const uint8_t init[16] = {19, 19, 255,  20, 255, 255,  20, 255,  20, 255, 255, 255, 255}; // Push to Connect
	static const uint8_t conn[16] = {20, 20, 255, 255, 255, 255, 255, 255, 255,   2,  21, 255, 255}; // Connecting
	static const uint8_t conf[16] = {21, 21, 255, 255, 255, 255, 255, 255, 255,  13,   0, 255, 255}; // Configuring
	static const uint8_t prob[16] = {23, 23, 255,  19, 255, 255,  19, 255,  19,  13,   0, 255, 255}; // Comms Problem
	memcpy(states[STATE_INITIAL], init, sizeof(init));
	memcpy(states[STATE_CONNECTING], conn, sizeof(conn));
	memcpy(states[STATE_CONFIG], conf, sizeof(conf));
	memcpy(states[STATE_PROBLEM], prob, sizeof(prob));
}

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT ContinuumBridge Ltd *****END OF FILE****/
