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
#include "load_screens.h"
#include "nodeid.h"
#include "ecog_driver.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//#define CB_DEMO_0
//#define FONT_2 					Arial_Rounded_MT_Bold19x20
//#define FONT_2 						Arial_Narrow16x23
#define FONT_2 					Arial_Narrow14x20
//#define FONT_2 Arial_Narrow16x20
//#define FONT_3 					Arial_Rounded_MT_Bold26x27
#define FONT_3 					Arial_Narrow18x26

#define STATE_TEST				16
#define STATE_INITIAL_TEST		26
#define STATE_SENDING			18
#define STATE_INITIAL 			19
#define STATE_CONNECTING 		20
#define STATE_CONFIG 			21
#define STATE_START 			22
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
#define BEACON_SEARCH_TIME 		10   // Units: 1 second
#define FIRST_ACK_SEARCH_TIME   1    // Units: 1 second
#define ACK_SEARCH_TIME         2    // Units: 1 second
#define ONE_DAY 				(24*60*60)
#define T_LONG_PRESS          	2    // Units: 1 second
#define T_DOUBLE_PRESS_16		8    // Units: 1/16 second
#define T_RESET_PRESS         	8    // Units: 1 second
#define T_MAX_RESET_PRESS		20   // To catch failure case

#define MAX_SCREEN 				38
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
#define BUTTON_NOT_PRESSED		0
#define BUTTON_PRESSED			1

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

uint8_t 			node_id[] 				= {0x00, 0x00, 0x00, 0x00};

char 				debug_buff[128] 		= {0};
char 				screens[MAX_SCREEN][1][194];
uint8_t				states[24][16]     		= {0xFF};

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
GPIO_PinState 		button_state;
int					side					= 0;
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
uint8_t				current_state			= STATE_INITIAL_TEST;
int32_t				button_press_time[2] 	= {-100, -100};
uint32_t			last_press_sixteenths[2] = {0, 0};
uint8_t				check_long[2]   		= {0, 0};
uint8_t				stop_mode				= 1;
uint8_t				start_from_reset		= 1;

typedef enum {initial, normal, pressed, search, search_failed, reverting, demo} NodeState;
NodeState         node_state           = initial;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Set_Display(uint8_t screen_num);
HAL_StatusTypeDef Rx_Message(uint8_t *buffer, int *length, uint16_t timeout);
void Radio_On(int delay);
void Host_Ready(void);
void Radio_Off(void);
uint8_t Message_Search(uint8_t *address, uint8_t *Rx_Buffer, uint16_t max_search_time);
void Send_Message(uint8_t function, uint8_t data_length, uint8_t *data, uint8_t ack, uint8_t beacon);
void Manage_Send(uint8_t ack, uint8_t beacon, uint8_t function);
void Network_Include(void);
void Listen_Radio(uint8_t reset_fail_count);
void Set_Wakeup(uint8_t force_awake);
void Build_Screen(uint8_t screen_num);
void Store_Config(void);
static void SYSCLKConfig_STOP(void);
static void SystemPower_Config(void);
void Send_Delay(void);
void Power_Down(void);
void On_RTC_IRQ(void);
void On_Button_IRQ(uint16_t button_pressed, uint16_t GPIO_Pin, GPIO_PinState button_state);
void Initialise_States(void);
void On_NewState(void);
int Read_Battery(uint8_t send);
void Configure_And_Test(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  GPIO_PinState gpio_test;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  SYSCLKConfig_STOP();
  SystemPower_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_RTC_Init();
  //MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  DWT_Init();

  /* USER CODE BEGIN 2 */

  Set_Node_ID(node_id);
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
  HAL_DBGMCU_DisableDBGStopMode();

  DEBUG_TX("Hello Computer\r\n\0");
  /*
  while(1)
  {
	  Delay_ms(62);
	  uint32_t sixteenths_now = SIXTEENTHS_NOW;
	  sprintf(debug_buff,"now: %d\r\n\0", (int)sixteenths_now);
	  DEBUG_TX(debug_buff);
	  Delay_ms(63);
	  sprintf(debug_buff,"now: %d\r\n\0", (int)sixteenths_now);
	  DEBUG_TX(debug_buff);
  }
  */
  Load_Normal_Screens();
  Initialise_States();
  ecog_init();

  // Reset radio
  Configure_And_Test();
  Radio_Off();
  DEBUG_TX("Waiting\r\n\0");
  HAL_UART_MspDeInit(&huart1);
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */

  while(1)
  {
	  /* Configures system clock after wake-up from STOP: enable HSE, PLL and select
	  PLL as system clock source (HSE and PLL are disabled in STOP mode) */
	  if(stop_mode)
	  {
		  //SysTick->CTRL  |= SysTick_CTRL_TICKINT_Msk;            // systick IRQ on
		  SYSCLKConfig_STOP();
		  HAL_UART_MspInit(&huart1);
		  DEBUG_TX("*** Main ***\r\n\0");
	  }
	  if(button_irq)
	  {
		  HAL_NVIC_DisableIRQ(EXTI3_IRQn);
		  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
		  Delay_ms(20);
		  button_state = HAL_GPIO_ReadPin(GPIOA, pressed_button);
		  sprintf(debug_buff, "Button IRQ: state: %d\r\n", button_state);
		  DEBUG_TX(debug_buff);
		  On_Button_IRQ(BUTTON_PRESSED, pressed_button, button_state);
		  Enable_IRQ();
		  button_irq = 0;
	  }
	  else if(rtc_irq)
	  {
		  rtc_irq = 0;
		  DEBUG_TX("RTC IRQ\r\n\0");
		  On_RTC_IRQ();
	  }
	  else if(!stop_mode)
	  {
		  uint32_t sixteenths_now = SIXTEENTHS_NOW;
		  for(side=0; side<2; side++)
		  {
			  if(last_press_sixteenths[side] != 0)
			  {
				  //sprintf(debug_buff,"Timeout: now %d: %u, last_press: %u, irq: %d\r\n", side, sixteenths_now, last_press_sixteenths[side], button_irq);
				  //DEBUG_TX(debug_buff);
				  if(sixteenths_now - last_press_sixteenths[side] > T_DOUBLE_PRESS_16)
				  {
					  On_Button_IRQ(BUTTON_NOT_PRESSED, pressed_button, button_state);
				  }
			  }
		  }
	  }
	  if(stop_mode)
	  {
		  Radio_Off();
		  for(side=0; side<2; side++)
			  last_press_sixteenths[side] = 0;
		  button_irq = 0;  // Seemed to be missed above
		  DEBUG_TX("Stopping\r\n\0");
		  //Delay_ms(20);
		  HAL_UART_MspDeInit(&huart1);
		  //HAL_PWR_EnterSTANDBYMode();
		  //SysTick->CTRL  &= ~SysTick_CTRL_TICKINT_Msk;        // systick IRQ off
		  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	  }
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
	  DEBUG_TX("Sysclock restart error 1\r\n\0");
  }

  /* Get the Clocks configuration according to the internal RCC registers */
  HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pFLatency);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType     = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource  = RCC_SYSCLKSOURCE_PLLCLK;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, pFLatency) != HAL_OK)
  {
	  DEBUG_TX("Sysclock restart error 2\r\n\0");
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


void Set_Display(uint8_t screen_num)
{
	if (ecog_write_inverse(0))
	{
		ecog_cls();
		Build_Screen(screen_num);
		ecog_update_display(1);
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
			//sprintf(debug_buff,"Build_Screen, font: %d\r\n", f);
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
			//sprintf(debug_buff,"Build_Screen, y: %d\r\n", y);
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
			//sprintf(debug_buff,"Build_Screen, centred text, pos: %d, len: %d\r\n", pos, len);
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

uint8_t On_Button_Press(uint16_t button_pressed, uint16_t GPIO_Pin, GPIO_PinState button_state)
{
	int 				side 					= LEFT_SIDE;
	uint32_t 			pressed_time 			= 0;
	uint32_t 			now;
	static uint8_t		check_double[2]			= {0, 0};
	static uint8_t		double_press			= 0;

	if (GPIO_Pin == PUSH_RIGHT_Pin)
		side = RIGHT_SIDE;
	else if (GPIO_Pin == LEFT_PUSH_Pin)
		side = LEFT_SIDE;
	else
		return PRESS_NONE;

	now = Cbr_Now();
	//check_double = 1;
	//sprintf(debug_buff,"16th now: %u, last %u, side: %d, state: %d\r\n", SIXTEENTHS_NOW, last_press_sixteenths[side], (int)side, (int)button_state);
	//DEBUG_TX(debug_buff);
	//button_state = HAL_GPIO_ReadPin(GPIOA, GPIO_Pin);
	//sprintf(debug_buff, "side: %d, doub: %d:%d, long: %d:%d, state: %d, pressed: %d\r\n",
	//		side, check_double[0], check_double[1], check_long[0], check_long[1],
	//		button_state, button_pressed);
	//DEBUG_TX(debug_buff);
	if(button_pressed && (button_state == GPIO_PIN_RESET))
	{
		check_long[0] = 0;  check_long[1] = 0; // Because we need to go through !button_pressed on the way to checking for longs
		if(check_double[side])
		{
			check_double[0] = 0; check_double[1] = 0;
			double_press = 1;
			stop_mode = 1;
			if(side == LEFT_SIDE) DEBUG_TX("AL\r\n\0"); else DEBUG_TX("AR\r\n\0");
			if(side == LEFT_SIDE)
				return PRESS_LEFT_DOUBLE;
			else
				return PRESS_RIGHT_DOUBLE;
		}
	   	else
	   	{
	   		last_press_sixteenths[side] = SIXTEENTHS_NOW;
	   		if(current_state != STATE_DEMO)
	   			Radio_On(0);  // Radio needs about 400 ms to turn on, so start now.
	   		else
	   			Radio_Off();
	   		double_press = 0;
	   		stop_mode = 0;
	   		if(side == LEFT_SIDE) DEBUG_TX("BL\r\n\0"); else DEBUG_TX("BR\r\n\0");
	   		return PRESS_NONE;
	   	}
	}
	else if(!button_pressed)
	{
  		button_press_time[side] = now;
   		check_long[side] = 1;   // Need to go through here when checking long
		check_double[0] = 0; check_double[1] = 0;
		last_press_sixteenths[side] = SIXTEENTHS_NOW;
		double_press = 0;
		stop_mode = 1;
   		if(side == LEFT_SIDE) DEBUG_TX("CL\r\n\0"); else DEBUG_TX("CR\r\n\0");
		if(side == LEFT_SIDE)
			return PRESS_LEFT_SINGLE;
		else
			return PRESS_RIGHT_SINGLE;
	}
	else if(button_state == GPIO_PIN_SET)
	{
		if(check_long[side])
		{
			check_long[0] = 0; check_long[1] = 0;
			pressed_time = now - button_press_time[side];
			//sprintf(debug_buff,"pressed_time: %d\r\n", (int)pressed_time);
			//DEBUG_TX(debug_buff);
			if((pressed_time > T_RESET_PRESS) && (pressed_time < T_MAX_RESET_PRESS) && (button_pressed))
			{
				DEBUG_TX("System reset\r\n\0");
				NVIC_SystemReset();
			}

			else if((pressed_time > T_LONG_PRESS) && (pressed_time < T_MAX_RESET_PRESS))
			{
				stop_mode = 1;
		   		if(side == LEFT_SIDE) DEBUG_TX("DL\r\n\0"); else DEBUG_TX("DR\r\n\0");
				if(side == LEFT_SIDE)
					return PRESS_LEFT_LONG;
				else
					return PRESS_RIGHT_LONG;
			}
			else
			{
		   		if(side == LEFT_SIDE) DEBUG_TX("EL\r\n\0"); else DEBUG_TX("ER\r\n\0");
		   		check_double[side] = 1;
				return PRESS_NONE;
			}
		}
		else
		{
	   		if(side == LEFT_SIDE) DEBUG_TX("FL\r\n\0"); else DEBUG_TX("FR\r\n\0");
			check_long[0] = 0; check_long[1] = 0;  // To reset the "other side"
			if(!double_press)
			{
				check_double[side] = 1;
				stop_mode = 0;
			}
			return PRESS_NONE;
		}
	}
	else
	{
		// Should not ever get here, but just in case
		stop_mode = 1;
		//DEBUG_TX("G\r\n\0");
		sprintf(debug_buff, "G side: %d, doub: %d:%d, long: %d:%d, state: %d, pressed: %d\r\n",
				side, check_double[0], check_double[1], check_long[0], check_long[1],
				button_state, button_pressed);
		DEBUG_TX(debug_buff);
		return PRESS_NONE;
	}
	return PRESS_NONE;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint32_t c = 1000;
	button_irq = 1;
	pressed_button = GPIO_Pin;
	while (HAL_NVIC_GetPendingIRQ(GPIO_Pin) && c--)
	{
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
		HAL_NVIC_ClearPendingIRQ(GPIO_Pin);
	}
	return;
}

void On_Button_IRQ(uint16_t button_pressed, uint16_t GPIO_Pin, GPIO_PinState button_state)
{
	uint8_t button_press;
	uint8_t old_state;
	uint8_t changed = 0;
	static uint8_t wait_demo;

	button_press = On_Button_Press(button_pressed, GPIO_Pin, button_state);
	sprintf(debug_buff, "P: %d\r\n", button_press);
	DEBUG_TX(debug_buff);

	if(button_press == PRESS_NONE)
	{
		return;
	}
	if(current_state == STATE_INITIAL_TEST)
	{
		current_state = STATE_INITIAL;
		Radio_On(1);             // Need this to properly
		Radio_Off();             // enter low power mode
		Set_Display(STATE_INITIAL);
	}
	else if(current_state == STATE_INITIAL)
	{
		DEBUG_TX("node_state is initial\r\n\0");
		if((button_press == PRESS_RIGHT_LONG) || (button_press == PRESS_LEFT_LONG))
		{
			wait_demo = 0;
			Network_Include();
		}
		else if(button_press == PRESS_LEFT_DOUBLE)
		{
			wait_demo = 1;
			return;
		}
		else if((button_press == PRESS_RIGHT_DOUBLE))
		{
			if(wait_demo)
			{
				DEBUG_TX("Starting demo mode\r\n\0");
				Radio_Off();
				Load_Demo_Screens();
				screen_num = 0;
				Set_Display(0);
				current_state = STATE_DEMO;
				return;
			}
		}
		else
		{
			wait_demo = 0;
			return;
		}
	}
	else if(current_state == STATE_DEMO)
	{
		DEBUG_TX("Node state demo\r\n\0");
		if ((button_press == PRESS_RIGHT_LONG) || (button_press == PRESS_LEFT_LONG))
			screen_num = 0;
		else
			screen_num++;
		if (screen_num == MAX_SCREEN)
			screen_num = 0;
		stop_mode = 1;
		Set_Display(screen_num);
	}
	else
	{
		old_state = current_state;
		switch(button_press)
		{
			case PRESS_LEFT_SINGLE:
				if(states[current_state][S_LS] != 0xFF)
				{
					changed = 1;
					current_state = states[current_state][S_LS];
				}
				break;
			case PRESS_LEFT_DOUBLE:
				if(states[current_state][S_LD] != 0xFF)
				{
					changed = 1;
					current_state = states[current_state][S_LD];
				}
				break;
			case PRESS_RIGHT_SINGLE:
				if(states[current_state][S_RS] != 0xFF)
				{
					changed = 1;
					current_state = states[current_state][S_RS];
				}
				break;
			case PRESS_RIGHT_DOUBLE:
				if(states[current_state][S_RD] != 0xFF)
				{
					changed = 1;
					current_state = states[current_state][S_RD];
				}
				break;
			case PRESS_LEFT_LONG:
			case PRESS_RIGHT_LONG:
				current_state = STATE_TEST;
				Configure_And_Test();
				return;
			default:
				DEBUG_TX("Warning. Undefined button_press\r\n\0");
				current_state = states[current_state][0];
		}
		if(current_state != old_state)
			On_NewState();
		else
			ecog_discharge_capacitors();
		if(!changed)
		{
			stop_mode = 1;
		    DEBUG_TX("Press not used in this state\r\n\0");
		}
	}
}

void On_NewState(void)
{
	uint8_t alert_id[] = {0x00, 0x00};
	sprintf(debug_buff, "On_NewState, state: %d\r\n\0", current_state);
	DEBUG_TX(debug_buff);
	if(states[current_state][S_A] != 0xFF)
	{
		Host_Ready();
		alert_id[1] = states[current_state][S_A]; alert_id[0] = 0x00;
		sprintf(debug_buff, "Sending alert: %x %x\r\n\0", alert_id[0], alert_id[1]);
		DEBUG_TX(debug_buff);
		Send_Message(f_alert, 2, alert_id, 1, 0);
	}
	else  // If a message is sent, display is changed as part of Manage_Send(), otherwise change here
	{
		if(states[current_state][S_D] != 0xFF)
			Set_Display(states[current_state][S_D]);
	}
	if(states[current_state][S_W] != 0xFF)  // Wait in this state for a fixed time
	{
		DEBUG_TX("On_NewState delay\r\n\0");
		RTC_Delay(states[current_state][S_W]);
		//current_state = states[current_state][S_WS];
		//On_NewState();
	}
}

void Power_Down(void)
{
	DEBUG_TX("Powering down\r\n\0");
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
	DEBUG_TX("Powered down\r\n\0");
}

void Network_Include(void)
{
	uint8_t equal = 1;
	int i;
	DEBUG_TX("Network_Include\r\n\0");
	if(include_state == 0)
	{
		Set_Display(STATE_CONNECTING);
		include_state = 1;
	}
	else if(include_state == 1)
	{
		include_state = 2;
	}
	else if(include_state == 2)
		include_state = 3;
	Radio_On(1);
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
				RTC_Delay(20);
				break;
			case 2:
				Radio_Off();
				RTC_Delay(60);
				break;
			case 3:
				Radio_Off();
				Set_Display(STATE_PROBLEM);
				RTC_Delay(600);
				break;
			default:
				Radio_Off();
				RTC_Delay(3600);
		}
		return;
	}
	if(Message_Search(grant_address, Rx_Buffer, BEACON_SEARCH_TIME) == SEARCH_OK)
	{
		DEBUG_TX("Received grant\r\n\0");
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
			DEBUG_TX("Sent ack for grant\r\n\0");
			include_state = 0;
			current_state = STATE_CONFIG;
			On_NewState();
			DEBUG_TX("Sending woken_up after grant\r\n\0");
			Send_Message(f_woken_up, 0, data, 1, 0);
		}
	}
	else
	{
		DEBUG_TX("Network_Include. Did not receive grant\r\n\0");
		RTC_Delay(30);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3)
	{
		DEBUG_TX("HAL_UART_TxCpltCallback\r\n\0");
		radio_ready = SET;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3)
	{
		DEBUG_TX("HAL_UART_RxCpltCallback\r\n\0");
		radio_ready = SET;
	}
}

void Radio_Tx_IT(UART_HandleTypeDef *uart, uint8_t *buffer, uint16_t buffer_size)
{
	radio_ready = RESET;
	if(HAL_UART_Transmit_IT(uart, (uint8_t *)buffer, buffer_size) != HAL_OK)
	{
		DEBUG_TX("Radio Tx Error\r\n\0");
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
		DEBUG_TX("Radio Tx Error\r\n\0");
	}
}

void Radio_Rx_IT(UART_HandleTypeDef *uart, uint8_t *buffer, uint16_t buffer_size)
{
	radio_ready = RESET;
	if(HAL_UART_Receive_IT(uart, (uint8_t *)buffer, buffer_size) != HAL_OK)
	{
		DEBUG_TX("UART Rx Error\r\n\0");
	}
	else
		DEBUG_TX("Radio_Rx_IT OK\r\n\0");
}

uint16_t Radio_Rx(UART_HandleTypeDef *uart, uint8_t *buffer, uint16_t buffer_size, uint16_t timeout)
{
	HAL_StatusTypeDef status = HAL_UART_Receive(uart, (uint8_t *)buffer, buffer_size, timeout);
	return status;
}

void Radio_On(int delay)
{
	DEBUG_TX("Radio_On\r\n\0");
    HAL_GPIO_WritePin(GPIOB, HOST_READY_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RADIO_POWER_GPIO_Port, RADIO_POWER_Pin, GPIO_PIN_SET);
    HAL_UART_MspInit(&huart3);
    if(delay)
    {
    	Delay_ms(400);
    	Host_Ready();
    	Delay_ms(50);
    }
}

void Host_Ready(void)
{
	HAL_GPIO_WritePin(GPIOB, HOST_READY_Pin, GPIO_PIN_RESET);
}

void Radio_Off(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	DEBUG_TX("Radio_Off\r\n\0");
	HAL_UART_MspDeInit(&huart3);
    HAL_GPIO_WritePin(RADIO_POWER_GPIO_Port, RADIO_POWER_Pin, GPIO_PIN_RESET);
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
	//DEBUG_TX("Send_Message\r\n\0");
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
	send_attempt = 0;
	Manage_Send(ack, beacon, function);
}

void Manage_Send(uint8_t ack, uint8_t beacon, uint8_t function)
{
	static uint8_t 		local_function;
	uint8_t 			found 					= 0;
	uint16_t 			search_time;

	sprintf(debug_buff, "Send attempt: %d\r\n", send_attempt);
	DEBUG_TX(debug_buff);
	if(send_attempt == 4)
		Radio_On(1);
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
			DEBUG_TX("Manage_Send found beacon\r\n\0");
			Send_Delay();
		}
		else
		{
			DEBUG_TX("Manage_Send did not find beacon\r\n\0");
			RTC_Delay(10);
			return;
		}
	}
	RADIO_TXS(tx_message, tx_length);
	if(ack)
	{
		if(send_attempt == 0)
			search_time = FIRST_ACK_SEARCH_TIME;
		else
			search_time = ACK_SEARCH_TIME;
		if (Message_Search(node_address, Rx_Buffer, search_time) == SEARCH_OK)
			found = 1;
		if(found)
		{
			if(Rx_Buffer[4] == f_ack)
			{
				send_attempt = 0;
				DEBUG_TX("Ack received\r\n\0");
				//sprintf(debug_buff, "S_D: %x, local_function: %x, state: %x, include: %x\r\n\0", (int)states[current_state][S_D],
				//		(int)local_function, (int)current_state, (int)include_state);
				//DEBUG_TX(debug_buff);
				uint32_t wakeup = ((Rx_Buffer[10] << 8) | Rx_Buffer[11]) << 1;
				if(wakeup != 0)
					Radio_Off();  // Turn radio off ASAP, before updating display
				if((current_state == STATE_PROBLEM) && (include_state == 0))  // Comms had failed by came back after a wake up
				{
					DEBUG_TX("Back from the dead\r\n\0");
					current_state = STATE_NORMAL;
					Set_Display(states[current_state][S_D]);
				}
				else if((states[current_state][S_D] != 0xFF) && (local_function != f_woken_up))
				{
					Set_Display(states[current_state][S_D]);
				}
				Set_Wakeup(0);
			}
			else
			{
				DEBUG_TX("Another message received when expecting ack\r\n\0");
				found = 0;
			}
		}
		if(!found)
		{
			DEBUG_TX("Manage_Send no ack 1\r\n\0");
			switch(send_attempt)
			{
				case 1:
					if (function != f_woken_up)
						Set_Display(STATE_SENDING);
				case 2:
				case 3:
					Send_Delay();
				case 5:
				case 9:
					Radio_Off();
					Delay_ms(200);
					Radio_On(1);
				case 6:
				case 8:
				case 10:
					Manage_Send(1, beacon, local_function);
					break;
				case 4:
					Radio_Off();
					RTC_Delay(30);
					break;
				case 7:
					DEBUG_TX("Manage_Send failure. Waiting 10 mins\r\n\0");
					current_state = STATE_PROBLEM;
					Radio_Off();
					On_NewState();
					RTC_Delay(600);
					break;
				case 11:
					send_attempt = 0;
					Network_Include();
					break;
			}
		}
	}
	else        // Don't wait for an ack, but give time for message to be sent
	{
		Delay_ms(200);
	}
}

void Listen_Radio(uint8_t reset_fail_count)
{
	static uint8_t fail_count = 0;
	if(reset_fail_count)
		fail_count = 0;
	sprintf(debug_buff, "Listen radio, fail: %d\r\n\0", (int)fail_count);
	DEBUG_TX(debug_buff);
	if(Message_Search(node_address, Rx_Buffer, ACK_SEARCH_TIME) == SEARCH_OK)
	{
		uint8_t data[] = {0x00, 0x00};
		if(Rx_Buffer[4] == f_config)
		{
			DEBUG_TX("Listen_Radio. Config message received\r\n\0");
			if(MODE != RATE)
				Store_Config();
			Send_Message(f_ack, 0, data, 0, 0);
			Set_Wakeup(0);
		}
		else if(Rx_Buffer[4] == f_start)
		{
			DEBUG_TX("Listen_Radio. Start received\r\n\0");
			if(start_from_reset)
			{
				start_from_reset = 0;
				current_state = STATE_START;
			}
			else
				current_state = STATE_NORMAL;
			Send_Message(f_ack, 0, data, 0, 0);
			On_NewState();
			Set_Wakeup(0);
		}
		else if(Rx_Buffer[4] == f_send_battery)
		{
			DEBUG_TX("Listen_Radio. Send battery received\r\n\0");
			Send_Message(f_ack, 0, data, 0, 0);
			Read_Battery(1);
			Set_Wakeup(0);
		}
		else if(Rx_Buffer[4] == f_ack)
		{
			DEBUG_TX("Listen_Radio. Ack received\r\n\0");
			Set_Wakeup(0);
		}
		else
		{
			DEBUG_TX("Listen_Radio. Unknown message, sending ack\r\n\0");
			Send_Message(f_ack, 0, data, 0, 0);
			Set_Wakeup(1);
		}
	}
	else
	{
		DEBUG_TX("Listen_Radio. Search failed\r\n\0");
		if(fail_count < 4)
		{
			fail_count++;
			Listen_Radio(0);
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
	DEBUG_TX("Set_Wakeup\r\n\0");
	if(error)
	{
		DEBUG_TX("Set_Wakeup, error\r\n\0");
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
			DEBUG_TX("Set_Wakeup, calling Listen_Radio\r\n\0");
			Listen_Radio(1);
		}
		else
		{
			sprintf(debug_buff, "Set_Wakeup for %d\r\n", (int)wakeup);
			DEBUG_TX(debug_buff);
			RTC_Delay(wakeup);
		}
	}
}

void Store_Config(void)
{
	uint8_t s, r, i = 0;
	uint8_t pos = 12;
	DEBUG_TX("Store_Config\r\n\0");
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
		//sprintf(debug_buff, "Store_Config, screen: %d, reg: %d, %02x %02x %02x %02x %02x %02x\r\n", s, r, screens[s][r][0], screens[s][r][1], screens[s][r][2], screens[s][r][3], screens[s][r][4], screens[s][r][5]);
		//DEBUG_TX(debug_buff);
		config_stored++;
		//sprintf(debug_buff, "Store_Config, config_stored: %d\r\n", config_stored);
		//DEBUG_TX(debug_buff);
		if(s == current_screen)
		{
			DEBUG_TX("Store_Config. Current screen updated\r\n\0");
			Set_Display(current_screen);
		}
	}
	else if(strncmp(Rx_Buffer+pos, "A", 1) == 0)
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

int Read_Battery(uint8_t send)
{
	const uint16_t adc2volts[] =
	{0, 2, 5, 7, 9, 12, 14, 16, 19, 21, 24, 26, 28, 31, 33, 35, 38, 40, 42, 45, 47, 49, 52, 54, 57, 59, 61,
	64, 66, 68, 71, 73, 75, 78, 80, 82, 85, 87, 90, 92, 94, 97, 99, 101, 104, 106, 108, 111, 113, 115, 118,
	120, 123, 125, 127, 130, 132, 134, 137, 139, 141, 144, 146, 148, 151, 153, 156, 158, 160, 163, 165, 167,
	170, 172, 174, 177, 179, 181, 184, 186, 189, 191, 193, 196, 198, 200, 203, 205, 207, 210, 212, 214, 217,
	219, 222, 224, 226, 229, 231, 233, 236, 238, 240, 243, 245, 247, 250, 252, 255, 257, 259, 262, 264, 266,
	269, 271, 273, 276, 278, 280, 283, 285, 288, 290, 292, 295, 297, 299, 302, 304, 306, 309, 311, 313, 316,
	318, 321, 323, 325, 328, 330, 332, 335, 337, 339, 342, 344, 346, 349, 351, 354, 356, 358, 361, 363, 365,
	368, 370, 372, 375, 377, 379, 382, 384, 386, 389, 391, 394, 396, 398, 401, 403, 405, 408, 410, 412, 415,
	417, 419, 422, 424, 427, 429, 431, 434, 436, 438, 441, 443, 445, 448, 450, 452, 455, 457, 460, 462, 464,
	467, 469, 471, 474, 476, 478, 481, 483, 485, 488, 490, 493, 495, 497, 500, 502, 504, 507, 509, 511, 514,
	516, 518, 521, 523, 526, 528, 530, 533, 535, 537, 540, 542, 544, 547, 549, 551, 554, 556, 559, 561, 563,
	566, 568, 570, 573, 575, 577, 580, 582, 584, 587, 589, 592, 594, 596, 599, 601};

	uint32_t adc_value = 0x55;
	uint8_t alert[] = {0x02, 0x00};
	int volts;
	MX_ADC_Init();
	HAL_GPIO_WritePin(GPIOB, BATT_READ_Pin, GPIO_PIN_SET);
	Delay_ms(100);
	if(HAL_ADC_Start(&hadc) != HAL_OK)
	{
		DEBUG_TX("ADC start error\r\n\0");
		HAL_ADC_DeInit(&hadc);
		HAL_GPIO_WritePin(GPIOB, BATT_READ_Pin, GPIO_PIN_RESET);
		return 0;
	}
	if(HAL_ADC_PollForConversion(&hadc, 50) != HAL_OK)
	{
		DEBUG_TX("ADC conversion error\r\n\0");
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
		volts = adc2volts[adc_value >> 8];
		sprintf(debug_buff, "ADC battery value: %x, volts %d.%d\r\n", (int)adc_value, (int)(volts/100), volts%100);
		DEBUG_TX(debug_buff);
		if(send)
		{
			alert[1] = adc_value >> 8;
			sprintf(debug_buff, "Sending alert: %x %x\r\n", alert[0], alert[1]);
			DEBUG_TX(debug_buff);
			Send_Message(f_alert, 2, alert, 1, 0);
		}
	}
	return volts;
}


uint8_t Message_Search(uint8_t *address, uint8_t *Rx_Buffer, uint16_t max_search_time)
{
	int search_start = Cbr_Now();
	//sprintf(debug_buff, "Message_Search, address: %02x %02x\r\n", address[0], address[1]);
	//DEBUG_TX(debug_buff);
	while (1)
	{
		__HAL_UART_FLUSH_DRREGISTER(&huart3);
		if(current_state != STATE_INITIAL) // So we can display "sending" message if there is no immediate ack
			status = Rx_Message(Rx_Buffer, &length, 1500);
		else
			status = Rx_Message(Rx_Buffer, &length, 8000);
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
				DEBUG_TX("Message_Search timeout\r\n\0");
				return SEARCH_TIMEOUT;
			}
		}
		else
		{
			__HAL_UART_FLUSH_DRREGISTER(&huart3);
			if ((Cbr_Now() - search_start) > max_search_time)
			{
				DEBUG_TX("Message_Search 2nd timeout error\r\n\0");
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
				DEBUG_TX("Radio_Rx error 2\r\n\0");
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
	if(include_state != 0)
	{
		Network_Include();
	}
	else if(send_attempt == 4 || send_attempt == 7)
		Manage_Send(1, 0, f_unknown);
	else if(states[current_state][S_W] != 0xFF)  // We were waiting in a state for a fixed time
	{
		DEBUG_TX("Woken up after delay\r\n\0");
		current_state = states[current_state][S_WS];
		On_NewState();
	}
	else
	{
		DEBUG_TX("Woken up\r\n\0");
		Radio_On(1);
		uint8_t data[] = {0x00, 0x00};
		Send_Message(f_woken_up, 0, data, 1, 0);
	}
}

void Configure_And_Test(void)
{
	int voltage = 0;
	uint8_t network_found = 0;

	voltage = Read_Battery(0);
	Radio_On(1);
	RADIO_TXS("ER_CMD#R0", 9);
	status = Rx_Message(Rx_Buffer, &length, 3000);
	DEBUG_TX("Received: ");
	DEBUG_TXS(Rx_Buffer, 9);
	DEBUG_TX("\r\n\0");
	Delay_ms(100);
	RADIO_TXS("ACK", 3);
	Delay_ms(200);

	// Set OTA data rate to 1200 bps
	RADIO_TXS("ER_CMD#B0", 9);
	status = Rx_Message(Rx_Buffer, &length, 3000);
	sprintf(debug_buff, "Received %.*s\r\n", length, Rx_Buffer);
	DEBUG_TX(debug_buff);
	Delay_ms(100);
	RADIO_TXS("ACK", 3);
	Delay_ms(200);

	RADIO_TXS("Hello World", 11);
	DEBUG_TX("Sent Hello World\r\n\0");
	Delay_ms(50);
	status = Rx_Message(Rx_Buffer, &length, 7000);
	if (status == HAL_OK)
	{
		network_found = 1;
		sprintf(debug_buff, "Received %.*s\r\n", length, Rx_Buffer);
		DEBUG_TX(debug_buff);
		// Get RSSI
		RADIO_TXS("ER_CMD#T8", 9);
		status = Rx_Message(Rx_Buffer, &length, 3000);
		sprintf(debug_buff, "RSSI echo: %.*s\r\n", length, Rx_Buffer);
		DEBUG_TX(debug_buff);
		Delay_ms(100);
		RADIO_TXS("ACK", 3);
		status = Rx_Message(Rx_Buffer, &length, 3000);
		sprintf(debug_buff, "RSSI: %.*s\r\n", length, Rx_Buffer);
		DEBUG_TX(debug_buff);
	}
	else
		DEBUG_TX("Receive problem\r\n\0");
	Radio_Off();
	sprintf(debug_buff, "Battery voltage: %d.%d\r\n", (int)(voltage/100), voltage%100);
	DEBUG_TX(debug_buff);
	sprintf(screens[16][0], "F\x02Y\x04G\x13Node ID: %010d", (int)node_id_int);
	screens[16][0][4] = 0x43;  // C
	sprintf(screens[16][0]+26, "Y\x1AG\x15Gattery voltage: %d.%d", (int)(voltage/100), voltage%100);
	screens[16][0][28] = 0x43;  // C
	screens[16][0][30] = 0x42;  // B
	if(network_found)
	{
	  	sprintf(screens[16][0]+52, "Y\x30G\x0DRSSI: %7s", Rx_Buffer);
	  	screens[16][0][54] = 0x43;  // C
	}
	else
	{
	  	sprintf(screens[16][0]+52, "Y\x30G\x0D No network  ");
	  	screens[16][0][54] = 0x43;  // C
	}
	sprintf(screens[16][0]+70, "Y\x46G\x15Push here to continue");
	screens[16][0][72] = 0x43;  // C
	Set_Display(STATE_TEST);
}

void Initialise_States(void)
{
    //                               S   D    A   LD    LS   MS  MD   RS   RD   XV   XS    W   WS
	static const uint8_t test[16] = {16, 16, 255,   0,   0,   0,   0,   0,   0, 255,  0, 255, 255}; // Test
	static const uint8_t init[16] = {19, 19, 255,  20, 255, 255,  20, 255,  20, 255, 255, 255, 255}; // Push to Connect
	static const uint8_t conn[16] = {20, 20, 255, 255, 255, 255, 255, 255, 255,   2,  21, 255, 255}; // Connecting
	static const uint8_t conf[16] = {21, 21, 255,  19,  19,  19,  19,  19,  19,  13,   0, 255, 255}; // Configuring
	static const uint8_t strt[16] = {22, 22, 255,   0, 255, 255,   0, 255,   0,  13,   0, 255, 255}; // Double-push to start
	static const uint8_t prob[16] = {23, 23, 255,  22, 255, 255,  19, 255,  22,  255,  0, 255, 255}; // Comms Problem
	memcpy(states[STATE_INITIAL], init, sizeof(init));
	memcpy(states[STATE_CONNECTING], conn, sizeof(conn));
	memcpy(states[STATE_CONFIG], conf, sizeof(conf));
	memcpy(states[STATE_START], strt, sizeof(strt));
	memcpy(states[STATE_PROBLEM], prob, sizeof(prob));
}

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT ContinuumBridge Ltd *****END OF FILE****/
