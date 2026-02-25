/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include <stdio.h>
#include <stdint.h>
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
/* ADC DMA buffer layout: [CH5,CH6,CH7, CH5,CH6,CH7, ...] */
#define ADC_CHANNEL_COUNT 3U
#define ADC_SAMPLES_PER_CHANNEL 8U
#define ADC_BUFFER_SIZE (ADC_CHANNEL_COUNT * ADC_SAMPLES_PER_CHANNEL)
#define ADC_CH_VBUS_INDEX 0U
#define ADC_CH_AUX1_INDEX 1U
#define ADC_CH_AUX2_INDEX 2U
uint16_t adc_buffer[ADC_BUFFER_SIZE];
uint16_t adc_channel_avg[ADC_CHANNEL_COUNT] = {0};

/* Flag set by DMA callback when new data is available */
volatile uint8_t adc_data_ready = 0;
volatile uint32_t adc_full_buffer_events = 0;
uint32_t last_adc_full_buffer_events = 0;

/* System state */
typedef enum {
  STATE_CUTOFF,   /* Power OFF - PA4=HIGH, PWM=100% */
  STATE_CONDUCTION /* Power ON - PA4=LOW, PWM=0% */
} SystemState_t;

SystemState_t current_state = STATE_CUTOFF;  /* Default safe state */

/* Voltage calculation result */
float vbus_voltage = 0.0f;

/* Timing for OLED update (1 Hz) */
uint32_t last_oled_update = 0;
#define OLED_UPDATE_INTERVAL 1000  /* 1Hz (1000ms/1 = 1000ms) */

/* TIM2-based software ADC triggering */
uint16_t last_tim2_cnt = 0;
#define ADC_TRIGGER_INTERVAL 10  /* 1ms @ 10kHz TIM2 = 72MHz/7200 = 10kHz, but PSC=71, ARR=999 gives 1kHz */

/* Voltage thresholds for hysteresis */
#define VBUS_UPPER_THRESHOLD 51.0f
#define VBUS_LOWER_THRESHOLD 48.0f

/* Software filter to eliminate noise
 * Moving average filter with 16 samples for stable readings
 * Reduces noise when ADC input is floating or has interference */
#define FILTER_SIZE 16
uint16_t filter_buffer[FILTER_SIZE];
uint8_t filter_index = 0;
uint8_t filter_count = 0;        /* Tracks number of samples filled */
uint8_t filter_initialized = 0;

/* PWM duty cycle constants for TIM3 CH3/CH4 (ARR=7199)
 * CCRx = ARR + 1 = 7200 gives constant HIGH (100% duty = cutoff)
 * CCRx = 0 gives constant LOW (0% duty = conduction) */
#define PWM_DUTY_CUTOFF     7200  /* 100% duty = constant HIGH */
#define PWM_DUTY_CONDUCTION 0     /* 0% duty = constant LOW */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Process_Voltage_And_Control(void);
void Update_OLED_Display(void);
void Set_Output_State(SystemState_t state);
static uint16_t Compute_Channel_Average(uint8_t channel_index);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern UART_HandleTypeDef huart2;

#ifdef __GNUC__
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

int _write(int file, char *ptr, int len)
{
  (void)file;
  for (int i = 0; i < len; i++)
    __io_putchar(*ptr++);
  return len;
}
#else
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
#endif
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /*
   * Startup sequence - ensure safe default state
   * 1. GPIO PA4 already initialized HIGH (OFF) by MX_GPIO_Init()
   * 2. Set PWM CH3/CH4 to 100% duty (constant HIGH = OFF state)
   *    For TIM3: ARR=7199, so CCRx = ARR + 1 = 7200 gives constant HIGH
   */
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_DUTY_CUTOFF);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_DUTY_CUTOFF);

  /* Start PWM outputs on CH3 (PB0) and CH4 (PB1) */
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start ADC with DMA FIRST - ADC must be ready before TIM2 triggers it
   * ADC is configured for external trigger (TIM2_CC2), so it will wait
   * for the trigger signal after this call */
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE) != HAL_OK)
  {
    Error_Handler();
  }

  /* CRITICAL FIX: Ensure ADC DMA bit is set
   * HAL_ADC_Start_DMA should do this, but verify it */
  SET_BIT(ADC1->CR2, ADC_CR2_DMA);
  
  /* Start TIM2 Output Compare on CH2 for ADC triggering.
   * CC2 event fires at CNT==CCR2 (every 1 ms) and triggers ADC. */
  if (HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  printf("[DEBUG] ADC1->CR2 after start: 0x%08lX (DMA=%lu,EXTTRIG=%lu)\r\n",
         ADC1->CR2,
         (ADC1->CR2 & ADC_CR2_DMA) ? 1UL : 0UL,
         (ADC1->CR2 & ADC_CR2_EXTTRIG) ? 1UL : 0UL);
  printf("[DEBUG] TIM2_CCER = 0x%04lX (CC2E=%lu)\r\n",
         TIM2->CCER, (TIM2->CCER & TIM_CCER_CC2E) ? 1UL : 0UL);
  printf("[DEBUG] TIM2_CCMR1 = 0x%04lX (OC2M=%lu, expect 3=TOGGLE)\r\n",
         TIM2->CCMR1, (TIM2->CCMR1 & TIM_CCMR1_OC2M) >> TIM_CCMR1_OC2M_Pos);
  printf("[DEBUG] TIM2->SR = 0x%04lX (CC2IF=%lu)\r\n",
         TIM2->SR, (TIM2->SR & TIM_SR_CC2IF) ? 1UL : 0UL);
  printf("[DEBUG] DMA1_Ch1->CCR = 0x%04lX (TCIE=%lu,HTIE=%lu)\r\n",
         DMA1_Channel1->CCR,
         (DMA1_Channel1->CCR & DMA_CCR_TCIE) ? 1UL : 0UL,
         (DMA1_Channel1->CCR & DMA_CCR_HTIE) ? 1UL : 0UL);
  printf("[DEBUG] DMA1_Ch1->CNDTR = %lu (remaining transfers)\r\n",
         DMA1_Channel1->CNDTR);
  
  printf("[INFO] ADC configured for TIM2_CC2 external trigger\r\n");

  /* Initialize OLED display */
  OLED_Init();
  OLED_Clear();

  /* Display startup message */
  OLED_ShowString(1, 1, "Power Protection");
  OLED_ShowString(2, 1, "Initializing...");

  /* Print startup message via UART */
  printf("\r\n========================================\r\n");
  printf("Power Protection System v2.0\r\n");
  printf("ADC: PA5/PA6/PA7 (CH5/6/7), PWM: PB0/CH3 + PB1/CH4, GPIO: PA4\r\n");
  printf("UART: PA2/PA3 @ 115200\r\n");
  printf("========================================\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /*
     * Main loop processing:
     * 1. Check if new ADC data is available (set by DMA callback)
     * 2. Calculate voltage and apply hysteresis logic
     * 3. Update OLED display at 1 Hz rate
     */

    if (adc_data_ready)
    {
      adc_data_ready = 0;  /* Clear flag */
      Process_Voltage_And_Control();
    }

    /* Update OLED display every 1 second */
    if (HAL_GetTick() - last_oled_update >= OLED_UPDATE_INTERVAL)
    {
      last_oled_update = HAL_GetTick();
      Update_OLED_Display();
    }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief ADC DMA Conversion Complete Callback
 * @param hadc: ADC handle
 * @note This callback is triggered when DMA completes a full buffer transfer
 *       CRITICAL: Only set flag, NO business logic here
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1)
  {
    adc_full_buffer_events++;
    adc_data_ready = 1;  /* Set flag for main loop processing */
    /* Debug: print first ADC value */
    printf("[ADC_CB] evt=%lu buf[0]=%u\r\n", adc_full_buffer_events, adc_buffer[0]);
  }
}

/**
 * @brief ADC DMA Half Conversion Complete Callback
 * @param hadc: ADC handle
 * @note Half-transfer events are intentionally ignored because the control
 *       path averages the full 8-sample buffer.
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  (void)hadc;
}

static uint16_t Compute_Channel_Average(uint8_t channel_index)
{
  uint32_t sum = 0U;

  for (uint8_t i = channel_index; i < ADC_BUFFER_SIZE; i += ADC_CHANNEL_COUNT)
  {
    sum += adc_buffer[i];
  }

  return (uint16_t)(sum / ADC_SAMPLES_PER_CHANNEL);
}

/**
 * @brief Process ADC data and apply hysteresis control logic
 * @note Called from main loop when adc_data_ready flag is set
 */
void Process_Voltage_And_Control(void)
{
  float adc_avg, vadc, vbus;
  SystemState_t new_state;

  /* Calculate per-channel average from interleaved DMA buffer */
  for (uint8_t ch = 0; ch < ADC_CHANNEL_COUNT; ch++)
  {
    adc_channel_avg[ch] = Compute_Channel_Average(ch);
  }
  adc_avg = (float)adc_channel_avg[ADC_CH_VBUS_INDEX];

  /* Apply moving average filter for noise reduction
   * This stabilizes readings when ADC input has interference */
  filter_buffer[filter_index] = (uint16_t)adc_avg;
  filter_index = (filter_index + 1) % FILTER_SIZE;

  /* Calculate filtered average */
  uint32_t filtered_sum = 0;
  if (!filter_initialized)
  {
    /* Increment sample counter during buffer filling phase */
    filter_count++;

    if (filter_count < FILTER_SIZE)
    {
      /* Still filling: average only available samples */
      for (uint8_t i = 0; i < filter_count; i++)
      {
        filtered_sum += filter_buffer[i];
      }
      adc_avg = (float)filtered_sum / filter_count;
    }
    else
    {
      /* Buffer full: use all FILTER_SIZE samples and mark initialized */
      for (uint8_t i = 0; i < FILTER_SIZE; i++)
      {
        filtered_sum += filter_buffer[i];
      }
      filter_initialized = 1;
      adc_avg = (float)filtered_sum / FILTER_SIZE;
    }
  }
  else
  {
    /* Normal operation: average over FILTER_SIZE samples */
    for (uint8_t i = 0; i < FILTER_SIZE; i++)
    {
      filtered_sum += filter_buffer[i];
    }
    adc_avg = (float)filtered_sum / FILTER_SIZE;
  }

  /* Convert to voltage
   * Vadc = adc_avg * 3.3 / 4095
   * Vbus = Vadc * 25
   */
  vadc = adc_avg * 3.3f / 4095.0f;
  vbus = vadc * 25.0f;

  /* Store global voltage for OLED display */
  vbus_voltage = vbus;

  /* Apply hysteresis logic */
  if (vbus > VBUS_UPPER_THRESHOLD)
  {
    /* Vbus > 51V: Cut off power */
    new_state = STATE_CUTOFF;
  }
  else if (vbus < VBUS_LOWER_THRESHOLD)
  {
    /* Vbus < 48V: Enable power conduction */
    new_state = STATE_CONDUCTION;
  }
  else
  {
    /* 48V <= Vbus <= 51V: Maintain current state (hysteresis band) */
    new_state = current_state;
  }

  /* Update outputs only if state changed */
  if (new_state != current_state)
  {
    Set_Output_State(new_state);
    current_state = new_state;
  }
}

/**
 * @brief Set output state (PA4 and PWM) based on system state
 * @param state: Desired system state
 * @note Updates PA4 and PWM synchronously
 */
void Set_Output_State(SystemState_t state)
{
  if (state == STATE_CUTOFF)
  {
    /* CUTOFF state: Power OFF
     * PA4 = HIGH (cutoff)
     * PWM CH3/CH4 (PB0/PB1) = 100% duty (constant HIGH = cutoff)
     */
    HAL_GPIO_WritePin(POWER_CTRL_GPIO_Port, POWER_CTRL_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_DUTY_CUTOFF);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_DUTY_CUTOFF);
    printf("[STATE] CUTOFF\r\n");
  }
  else  /* STATE_CONDUCTION */
  {
    /* CONDUCTION state: Power ON
     * PA4 = LOW (conduction)
     * PWM CH3/CH4 (PB0/PB1) = 0% duty (constant LOW = conduction)
     */
    HAL_GPIO_WritePin(POWER_CTRL_GPIO_Port, POWER_CTRL_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_DUTY_CONDUCTION);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_DUTY_CONDUCTION);
    printf("[STATE] CONDUCTION\r\n");
  }
}

/**
 * @brief Update OLED display with current voltage and state
 * @note Called at 1 Hz rate from main loop
 *       CRITICAL: Never call this from ISR/DMA callback
 */
void Update_OLED_Display(void)
{
  char line_buffer[32];

  /* Clear display */
  OLED_Clear();

  /* Display voltage (after x25 multiplier)
   * Use integer math to avoid float formatting issues
   * Display as integer + 1 decimal place */
  uint32_t vbus_int = (uint32_t)vbus_voltage;
  uint32_t vbus_frac = (uint32_t)(vbus_voltage * 10.0f) % 10;
  snprintf(line_buffer, sizeof(line_buffer), "Vbus:%lu.%luV", vbus_int, vbus_frac);
  OLED_ShowString(1, 1, line_buffer);
  OLED_ShowString(1, 16, "L");  /* LUX - L at right side */

  /* Display PWM state - shorten to fit 16 chars */
  if (current_state == STATE_CUTOFF)
  {
    OLED_ShowString(2, 1, "State:OFF 100%");
  }
  else
  {
    OLED_ShowString(2, 1, "State:ON 0%");
  }
  OLED_ShowString(2, 16, "U");  /* LUX - U at right side */

  /* Display CH5 raw ADC average used by control loop */
  uint32_t adc_avg = Compute_Channel_Average(ADC_CH_VBUS_INDEX);
  uint32_t adc_aux1 = Compute_Channel_Average(ADC_CH_AUX1_INDEX);
  uint32_t adc_aux2 = Compute_Channel_Average(ADC_CH_AUX2_INDEX);
  snprintf(line_buffer, sizeof(line_buffer), "ADC5:%lu", adc_avg);
  OLED_ShowString(3, 1, line_buffer);
  OLED_ShowString(3, 16, "X");  /* LUX - X at right side */

  /* Display diagnostic info - check ADC/TIM2 status and data flow heartbeat */
  uint8_t exttrig = (hadc1.Init.ExternalTrigConv == ADC_EXTERNALTRIGCONV_T2_CC2) ? 1U : 0U;
  uint8_t tim2_cr1 = (TIM2->CR1 & TIM_CR1_CEN) ? 1 : 0;
  uint8_t data_flow_ok = (adc_full_buffer_events != last_adc_full_buffer_events) ? 1U : 0U;
  last_adc_full_buffer_events = adc_full_buffer_events;
  /* Display actual threshold values from defines (convert float to int for printf) */
  snprintf(line_buffer, sizeof(line_buffer), "TER%u%u%u(%d,%d)",
           tim2_cr1, exttrig, data_flow_ok,
           (int)VBUS_LOWER_THRESHOLD, (int)VBUS_UPPER_THRESHOLD);
  OLED_ShowString(4, 1, line_buffer);

  /* Send periodic debug info via UART without float-format dependency */
  uint32_t monitor_vbus_int = (uint32_t)vbus_voltage;
  uint32_t monitor_vbus_frac = (uint32_t)(vbus_voltage * 10.0f) % 10;
  /* Debug: Show first few ADC buffer values and DMA event count */
  printf("[MONITOR] Vbus=%lu.%luV ADC5=%lu ADC6=%lu ADC7=%lu State=%s DMA_EVT=%lu\r\n",
         monitor_vbus_int, monitor_vbus_frac, adc_avg, adc_aux1, adc_aux2,
         (current_state == STATE_CUTOFF) ? "CUTOFF" : "CONDUCTION",
         adc_full_buffer_events);
  printf("[DEBUG] adc_buffer[0-2]=%u,%u,%u TIM2_SR=0x%04lX(CC2IF=%lu)\r\n",
         adc_buffer[0], adc_buffer[1], adc_buffer[2],
         TIM2->SR, (TIM2->SR & TIM_SR_CC2IF) ? 1UL : 0UL);
  printf("[DEBUG] DMA1_Ch1->CNDTR=%lu ISR=0x%04lX\r\n",
         DMA1_Channel1->CNDTR, DMA1->ISR);
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
#ifdef USE_FULL_ASSERT
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
