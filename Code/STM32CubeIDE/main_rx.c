/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lptim.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"   // <- needed on RX for TIM1 handle & init
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nrf_drivers.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_QUIET_MS 150u
static uint32_t rx_quiet_until = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint64_t RxpipeAddrs = 0x11223344AAULL;
char myRxData[50];
char myAckPayload[32] = "Ack by STMF7!";

extern TIM_HandleTypeDef htim1;
#define SERVO_TIM       (&htim1)
#define SERVO_CH        TIM_CHANNEL_4

// 50 Hz frame
#define SERVO_PERIOD_US 20000U
// Your stated end-stops
#define SERVO_MIN_US    500U
#define SERVO_MAX_US    2500U

static bool servo_is_pos90 = false;

static inline void Servo_WritePulseUS(uint16_t us)
{
    if (us < SERVO_MIN_US) us = SERVO_MIN_US;
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;

    // We assume ARR is 20000-1 and PSC makes 1 tick = 1 µs.
    __HAL_TIM_SET_COMPARE(SERVO_TIM, SERVO_CH, us);
}

void Servo_Init(void)
{
    // Start PWM output
    HAL_TIM_PWM_Start(SERVO_TIM, SERVO_CH);

    // Safe neutral on boot (optional)
    Servo_WritePulseUS(1500);
    servo_is_pos90 = false;
}

void Servo_GotoNeg90(void) { servo_is_pos90 = false; Servo_WritePulseUS(500);  }
void Servo_GotoPos90(void) { servo_is_pos90 = true;  Servo_WritePulseUS(2500); }

void Servo_Toggle(void)
{
    if (servo_is_pos90) Servo_GotoNeg90();
    else                Servo_GotoPos90();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPTIM1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  	NRF24_begin(GPIOA, NRF_CSN_Pin, NRF_CE_Pin, hspi1);
  	nrf24_DebugUART_Init(huart2);

  	GPIO_InitTypeDef gi_ce = {0};
  	gi_ce.Pin = NRF_CE_Pin;
  	gi_ce.Mode = GPIO_MODE_OUTPUT_PP;
  	gi_ce.Pull = GPIO_NOPULL;
  	gi_ce.Speed = GPIO_SPEED_FREQ_HIGH;
  	HAL_GPIO_Init(NRF_CE_GPIO_Port, &gi_ce);

  	NRF24_disableDynamicPayloads();
  	NRF24_setAutoAck(true);

    uint8_t enaa = NRF24_read_register(REG_EN_AA);

  	NRF24_setRetries(10,15);
  	NRF24_setDataRate(RF24_250KBPS);
  	NRF24_setChannel(76);
  	NRF24_setPayloadSize(32);
  	NRF24_openWritingPipe(0x11223344AAULL);      // TX
  	NRF24_openReadingPipe(0, 0x11223344AAULL);   // RX

  	// Power-up + force PRX
  	uint8_t cfg = NRF24_read_register(REG_CONFIG);
  	cfg |= _BV(BIT_PWR_UP) | _BV(BIT_PRIM_RX);
  	NRF24_write_register(REG_CONFIG, cfg);
  	HAL_Delay(5);
  	// Make absolutely sure CE is high in RX
  	HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET);

  	uint8_t after = NRF24_read_register(REG_CONFIG);
  	printf("CONFIG=0x%02X (expect bit0=1 for PRX)\r\n", after);

  	NRF24_flush_rx();                           // clean slate
  	NRF24_resetStatus();
  	NRF24_startListening();

  	printRadioSettings();
  	printStatusReg();
  	printConfigReg();
    MX_TIM1_Init();     // make sure Cube actually generated this
    Servo_Init();       // calls HAL_TIM_PWM_Start + sets neutral

  /* USER CODE END 2 */


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if (NRF24_available()) {
	      bool got_any = false;
	      while (!(NRF24_read_register(REG_FIFO_STATUS) & _BV(BIT_RX_EMPTY))) {
	          NRF24_read((uint8_t*)myRxData, 32);
	          got_any = true;
	      }
	      if (got_any) {
	          uint32_t now = HAL_GetTick();
	          if (now >= rx_quiet_until) {
	        	    HAL_GPIO_TogglePin(LED_OUT_GPIO_Port, LED_OUT_Pin);
	        	    Servo_Toggle();
	              rx_quiet_until = now + RX_QUIET_MS;
	          }
	      }
	  }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
