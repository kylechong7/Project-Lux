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
#include "adc.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nrf_drivers.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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
uint64_t TxpipeAddrs = 0x11223344AA;
char myTxData[50] = "Hello World!";
char AckPayload[32];

#define RF_ADDR64     0x11223344AAULL   // 5 LSBytes go on-air: AA 44 33 22 11
#define RF_CHANNEL    52                // keep in ISM band, avoid Wi-Fi
#define RF_PAYLEN     32                // fixed 32B for simplicity

#define BTN_PORT      GPIOB
#define BTN_PIN       GPIO_PIN_9
#define DEBOUNCE_MS   25u

volatile uint32_t g_btn_last_ms = 0;
volatile uint8_t  g_btn_press_event = 0;

static inline void on_button_irq(uint16_t GPIO_Pin) {
    if (GPIO_Pin != BTN_PIN) return;

    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - g_btn_last_ms) >= DEBOUNCE_MS) {
        g_btn_last_ms = now;
        // Active-low check for Falling+PullUp
        if (HAL_GPIO_ReadPin(BTN_PORT, BTN_PIN) == GPIO_PIN_RESET) {
            g_btn_press_event = 1;
        }
    }
}

// ALWAYS ADD THIS FOR EXTI!!!
void HAL_GPIO_EXTI_Callback(uint16_t pin)         { on_button_irq(pin); }
void HAL_GPIO_EXTI_Falling_Callback(uint16_t pin) { on_button_irq(pin); }
void HAL_GPIO_EXTI_Rising_Callback(uint16_t pin)  { on_button_irq(pin); }

typedef struct {
    bool acked;
    bool max_rt;
    bool timeout;
    uint8_t status;
    uint8_t fifo;
    uint8_t arc_cnt;   // # of retries used for this packet
    uint8_t plos_cnt;  // cumulative lost packets (cleared by writing RF_CH)
} nrf_tx_report_t;

// Registers/bits (rename to your enum/defines)
#define REG_STATUS        0x07
#define REG_FIFO_STATUS   0x17
#define REG_OBSERVE_TX    0x08
#define REG_RF_CH         0x05
#define BIT_RX_DR         6
#define BIT_TX_DS         5
#define BIT_MAX_RT        4

static inline bool bit_is_set(uint8_t v, uint8_t b){ return (v & (1U<<b)) != 0; }

nrf_tx_report_t NRF24_writeBlockingAck(const uint8_t* buf, uint8_t len, uint32_t timeout_ms)
{
    nrf_tx_report_t r = (nrf_tx_report_t){0};

    /* Force PTX + PWR_UP, then wait tpd2stby (≥1.5ms) */
    uint8_t cfg = NRF24_read_register(REG_STATUS - 0x00); // REG_CONFIG is 0x00
    (void)cfg; // silence unused in some builds
    uint8_t config = NRF24_read_register(0x00);
    config = (config | _BV(BIT_PWR_UP)) & ~_BV(BIT_PRIM_RX);
    NRF24_write_register(0x00, config);
    HAL_Delay(2);

    /* Load payload while CE is low */
    NRF24_ce(0);
    NRF24_write_payload(buf, len);

    /* Kick TX: CE high >10us */
    NRF24_ce(1);
    NRF24_DelayMicroSeconds(20);
    NRF24_ce(0);

    /* Wait for TX_DS (ACK) or MAX_RT */
    const uint32_t t0 = HAL_GetTick();
    while (1) {
        r.status = NRF24_get_status();
        if (bit_is_set(r.status, BIT_TX_DS)) { r.acked = true;  break; }
        if (bit_is_set(r.status, BIT_MAX_RT)) { r.max_rt = true; break; }
        if ((HAL_GetTick() - t0) > timeout_ms) { r.timeout = true; break; }
    }

    /* Snapshot diagnostics */
    uint8_t obs = NRF24_read_register(REG_OBSERVE_TX);
    r.arc_cnt  = (obs & 0x0F);
    r.plos_cnt = (obs >> 4);
    r.fifo     = NRF24_read_register(REG_FIFO_STATUS);

    /* Clear IRQs and recover on MAX_RT */
    NRF24_resetStatus();
    if (r.max_rt) NRF24_flush_tx();

    return r;
}

// Pretty print (call after send)
static void NRF24_print_tx_report(const nrf_tx_report_t* r)
{
    const char* verdict =
        r->acked  ? "ACK" :
        r->max_rt ? "FAIL(MAX_RT)" :
        r->timeout? "FAIL(TIMEOUT)" : "UNKNOWN";

    printf("TX %s | STATUS=0x%02X FIFO=0x%02X ARC=%u PLOS=%u\r\n",
           verdict, r->status, r->fifo, r->arc_cnt, r->plos_cnt);
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
  MX_ADC1_Init();
//  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start(&hadc1);

  NRF24_begin(GPIOA, NRF_CSN_Pin, NRF_CE_Pin, hspi1);
  nrf24_DebugUART_Init(huart2);



  NRF24_setAutoAck(true);
  uint8_t enaa = NRF24_read_register(REG_EN_AA);
  printf("EN_AA=0x%02X (expect 00)\r\n", enaa);
  NRF24_setRetries(10,15);
  NRF24_setDataRate(RF24_250KBPS);
  NRF24_setChannel(76);
  NRF24_setPayloadSize(32);
  NRF24_openWritingPipe(0x11223344AAULL);      // TX

  // Re-arm radio after any power hiccup
  NRF24_ce(0);
  NRF24_flush_tx();
  NRF24_flush_rx();
  NRF24_resetStatus();

  // PTX (PRIM_RX=0) + PWR_UP=1; keep CRC as-is
  uint8_t cfg = NRF24_read_register(0x00);             // CONFIG
  cfg = (cfg | _BV(BIT_PWR_UP)) & ~_BV(BIT_PRIM_RX);
  NRF24_write_register(0x00, cfg);
  HAL_Delay(2);                                        // tpd2stby

  // (optional) prove it
  printf("CONFIG(after arm)=0x%02X\r\n", NRF24_read_register(0x00));

  printConfigReg();

//  HAL_ADCEx_Calibration_Start(&hadc1);        // 1-arg on G0
//  HAL_ADC_Start(&hadc1);
//
//	NRF24_begin(GPIOA, NRF_CSN_Pin, NRF_CE_Pin, hspi1);
//	nrf24_DebugUART_Init(huart2);
//
//		//**** TRANSMIT - ACK ****//
//	NRF24_stopListening();
//	NRF24_openWritingPipe(TxpipeAddrs);
//	NRF24_setAutoAck(true);
//	NRF24_setChannel(52);
//	NRF24_setPayloadSize(32);
  ////-BRING THIS BACK-/////

//	NRF24_write(myTxData,32);
//	printRadioSettings();
//	uint8_t c0 = NRF24_read_register(0x00);      // CONFIG
//	NRF24_write_register(0x00, c0 ^ 0x02);       // toggle PWR_UP bit
//	uint8_t c1 = NRF24_read_register(0x00);
//	printf("CONFIG was 0x%02X, now 0x%02X\r\n", c0, c1);
//	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//	{
//	    if (GPIO_Pin == TRIGGER_IN_Pin)   // we only care about PA1
//	    {
//	        HAL_GPIO_TogglePin(LED_OUT_GPIO_Port, LED_OUT_Pin);
//	    }
//	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//#define PIEZO_THRESHOLD        1000U   // ADC counts to call a “hit”
//#define DEBOUNCE_MS              30U   // ignore chatter for 30 ms
//#define DOUBLE_SNAP_WINDOW_MS   250U   // max gap 1st→2nd snap
//#define LED_ON_MS               500U   // keep LED on this long
//	/* ------------------------------------------------------------- */
//
//uint32_t piezo_raw;
//uint32_t t_first_hit    = 0;   // time of the 1st valid snap
//uint8_t  waiting_second = 0;   // FSM state: 0 = idle, 1 = saw first snap
//uint32_t led_on_until   = 0;   // when to turn LED off
//uint32_t last_hit_ms    = 0;   // last time we accepted a hit
//	tie_ce_high_diag();
  // If CE is physically tied high, optionally make MCU CE pin input (see step 0)
//  printRadioSettings();
  printConfigReg();
	NRF24_flush_rx();
	NRF24_flush_tx();



  while (1)
  {
    /* USER CODE END WHILE */
	  if (g_btn_press_event) {
	      g_btn_press_event = 0;
	      printf("hello");
	      static uint8_t txbuf[32] = "hello-from-PTX-noack"; // rest zeros
	      bool ok = NRF24_write(txbuf, 32);   // driver handles CE pulse

	      uint8_t payload[32] = { /* ... */ };
	      nrf_tx_report_t rep = NRF24_writeBlockingAck(payload, 32, /*timeout_ms=*/50);
	      NRF24_print_tx_report(&rep);

	      printStatusReg();
	      uint8_t fifo = NRF24_read_register(REG_FIFO_STATUS);
	      printf("write()->%d  FIFO=0x%02X\r\n", ok, fifo);

	  }
		}
}
//	  HAL_GPIO_TogglePin(LED_OUT_GPIO_Port, LED_OUT_Pin);
//	  HAL_Delay(500);
    /* USER CODE BEGIN 3 */
	/*  transmitter loop */

//	  if (NRF24_write(myTxData, 32)) {
//	    HAL_UART_Transmit(&huart2,(uint8_t*)"TX OK\r\n",6,10);
//
//	    if (NRF24_isAckPayloadAvailable()) {
//	      NRF24_read(AckPayload, 32);
//	      char msg[80];
//	      snprintf(msg, sizeof(msg), "AckPayload: %.*s\r\n", 32, AckPayload);
//	      HAL_UART_Transmit(&huart2,(uint8_t*)msg, strlen(msg), 10);
//	    }
//	  } else {
//	    HAL_UART_Transmit(&huart2,(uint8_t*)"TX FAIL\r\n",8,10);
//	  }
//	  printStatusReg();
//	  HAL_Delay(1000);

	  //piezo input loop

	  // Read once from ADC1 (PA1 = ADC_IN6 on STM32G031) and drive LED
//	  read_and_print_piezo();
//	  HAL_Delay(100);  // print every 100 ms
//	  // Send result over UART to confirm
//	  char msg[50];
//	  sprintf(msg, "ADC: %lu\r\n", adc_val);
//	  HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 10);
//	  HAL_Delay(50);
//	  HAL_GPIO_WritePin(GPIOA, LED_OUT_Pin, RESET);

	  /* --- piezo / LED loop ----------------------------------------- */

//
///* in main() before the loop, you should already have: */
///*  HAL_UART_Transmit(&huart2, (uint8_t *)"Boot up complete\r\n", 18, HAL_MAX_DELAY); */
//
//    HAL_ADC_Start(&hadc1);
//    if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK)
//    {
//        piezo_raw = HAL_ADC_GetValue(&hadc1);
//
//        // echo raw value
//        {
//            char buf[32];
//            int len = sprintf(buf, "Piezo: %4lu  ", piezo_raw);
//            HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
//        }
//
//        if (piezo_raw > PIEZO_THRESHOLD)
//        {
//            uint32_t now = HAL_GetTick();
//
//            // debounce
//            if (now - last_hit_ms >= DEBOUNCE_MS)
//            {
//                last_hit_ms = now;
//                HAL_UART_Transmit(&huart2,
//                    (uint8_t *)"  >> Hit!  ", 11, HAL_MAX_DELAY);
//
//                // FSM: first vs. second snap
//                if (!waiting_second)
//                {
//                    waiting_second = 1;
//                    t_first_hit    = now;
//                    HAL_UART_Transmit(&huart2,
//                        (uint8_t *)"[1st snap]\r\n", 13, HAL_MAX_DELAY);
//                }
//                else if ((now - t_first_hit) <= DOUBLE_SNAP_WINDOW_MS)
//                {
//                    waiting_second = 0;
//                    led_on_until   = now + LED_ON_MS;
//                    HAL_GPIO_WritePin(LED_OUT_GPIO_Port, LED_OUT_Pin, GPIO_PIN_SET);
//                    HAL_UART_Transmit(&huart2,
//                        (uint8_t *)"[DOUBLE SNAP! LED ON]\r\n", 24, HAL_MAX_DELAY);
//                }
//                else
//                {
//                    // too late → treat as new first
//                    t_first_hit    = now;
//                    HAL_UART_Transmit(&huart2,
//                        (uint8_t *)"[late second → reset]\r\n", 25, HAL_MAX_DELAY);
//                }
//            }
//            else
//            {
//                // within debounce window
//                HAL_UART_Transmit(&huart2,
//                    (uint8_t *)"[debounced]\r\n", 14, HAL_MAX_DELAY);
//            }
//        }
//        else
//        {
//            // below threshold
//            HAL_UART_Transmit(&huart2,
//                (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);
//        }
//
//        // timeout waiting for 2nd
//        if (waiting_second && (HAL_GetTick() - t_first_hit > DOUBLE_SNAP_WINDOW_MS))
//        {
//            waiting_second = 0;
//            HAL_UART_Transmit(&huart2,
//                (uint8_t *)"[2nd snap timeout → idle]\r\n", 28, HAL_MAX_DELAY);
//        }
//
//        // turn LED off when its time expires
//        if (led_on_until && HAL_GetTick() > led_on_until)
//        {
//            led_on_until = 0;
//            HAL_GPIO_WritePin(LED_OUT_GPIO_Port, LED_OUT_Pin, GPIO_PIN_RESET);
//            HAL_UART_Transmit(&huart2,
//                (uint8_t *)"[LED OFF]\r\n", 11, HAL_MAX_DELAY);
//        }
//    }

  /* USER CODE END 3 */

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
int __io_putchar(int ch) {
	    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	    return ch;
	}

	void read_and_print_piezo(void) {
	    uint32_t adc_val = 0;

	    HAL_ADC_Start(&hadc1);
	    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
	        adc_val = HAL_ADC_GetValue(&hadc1);
	    }
	    HAL_ADC_Stop(&hadc1);

	    printf("Piezo ADC value: %lu\r\n", adc_val);
	}


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
