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
/* ADC DMA buffer - 8 samples for hardware-triggered sampling at 10 kHz */
#define ADC_BUFFER_SIZE 8
uint16_t adc_buffer[ADC_BUFFER_SIZE];

/* Flag set by DMA callback when new data is available */
volatile uint8_t adc_data_ready = 0;

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

/* Voltage thresholds for hysteresis */
#define VBUS_UPPER_THRESHOLD 51.0f
#define VBUS_LOWER_THRESHOLD 48.0f

/* Software filter to eliminate noise
 * Moving average filter with 16 samples for stable readings
 * Reduces noise when ADC input is floating or has interference */
#define FILTER_SIZE 16
uint16_t filter_buffer[FILTER_SIZE];
uint8_t filter_index = 0;
uint8_t filter_initialized = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Process_Voltage_And_Control(void);
void Update_OLED_Display(void);
void Set_Output_State(SystemState_t state);
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
   * 2. Set PWM to 100% duty (constant HIGH = OFF state)
   *    For TIM3: ARR=7199, so CCR3 = ARR + 1 = 7200 gives constant HIGH
   */
  TIM3->CCR3 = 7200;  /* 100% duty = constant HIGH = cutoff */

  /* Start PWM output on CH3 (PB0) */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  /* Start ADC with DMA FIRST - ADC must be ready before TIM2 triggers it
   * ADC is configured for external trigger (TIM2_CC2), so it will wait
   * for the trigger signal after this call
   *
   * NOTE: For STM32F1, ensure external trigger is enabled in ADC_CR2 */
  ADC1->CR2 |= ADC_CR2_EXTTRIG;  /* Explicitly enable external trigger */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE);

  /* Start TIM2 Output Compare LAST - this starts triggering the ADC at 10 kHz
   * Delay: ensure ADC DMA is fully ready before first trigger */
  HAL_Delay(10);

  /* Start TIM2 - this should trigger ADC conversions at 10 kHz */
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_2);

  /* Initialize OLED display */
  OLED_Init();
  OLED_Clear();

  /* Display startup message */
  OLED_ShowString(1, 1, "Power Protection");
  OLED_ShowString(2, 1, "Initializing...");

  /* Print startup message via UART */
  printf("\r\n========================================\r\n");
  printf("Power Protection System v2.0\r\n");
  printf("ADC: PA5/CH5, PWM: PB0/CH3, GPIO: PA4\r\n");
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
    adc_data_ready = 1;  /* Set flag for main loop processing */
  }
}

/**
 * @brief ADC DMA Half Conversion Complete Callback
 * @param hadc: ADC handle
 * @note This callback is triggered when DMA completes half buffer transfer
 *       Also set data ready flag for more frequent updates
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc != NULL && hadc->Instance == ADC1)
  {
    adc_data_ready = 1;  /* Set flag for main loop processing */
  }
}

/**
 * @brief Process ADC data and apply hysteresis control logic
 * @note Called from main loop when adc_data_ready flag is set
 */
void Process_Voltage_And_Control(void)
{
  uint32_t adc_sum = 0;
  float adc_avg, vadc, vbus;
  SystemState_t new_state;

  /* Calculate average of 8 DMA samples */
  for (uint8_t i = 0; i < ADC_BUFFER_SIZE; i++)
  {
    adc_sum += adc_buffer[i];
  }
  adc_avg = (float)adc_sum / ADC_BUFFER_SIZE;

  /* Apply moving average filter for noise reduction
   * This stabilizes readings when ADC input has interference */
  filter_buffer[filter_index] = (uint16_t)adc_avg;
  filter_index = (filter_index + 1) % FILTER_SIZE;

  /* Calculate filtered average */
  uint32_t filtered_sum = 0;
  if (!filter_initialized)
  {
    /* First fill: use available samples */
    for (uint8_t i = 0; i <= filter_index; i++)
    {
      filtered_sum += filter_buffer[i];
    }
    adc_avg = (float)filtered_sum / (filter_index + 1);

    /* Mark as initialized after first complete cycle */
    if (filter_index == 0 && filtered_sum > 0)
    {
      filter_initialized = 1;
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
 * @brief Set output state (PA1 and PWM) based on system state
 * @param state: Desired system state
 * @note Updates PA1 and PWM synchronously
 */
void Set_Output_State(SystemState_t state)
{
  if (state == STATE_CUTOFF)
  {
    /* CUTOFF state: Power OFF
     * PA4 = HIGH (cutoff)
     * PWM CH3 (PB0) = 100% duty (constant HIGH = cutoff)
     */
    HAL_GPIO_WritePin(POWER_CTRL_GPIO_Port, POWER_CTRL_Pin, GPIO_PIN_SET);
    TIM3->CCR3 = 7200;  /* ARR + 1 = constant HIGH */
    printf("[STATE] CUTOFF\r\n");
  }
  else  /* STATE_CONDUCTION */
  {
    /* CONDUCTION state: Power ON
     * PA4 = LOW (conduction)
     * PWM CH3 (PB0) = 0% duty (constant LOW = conduction)
     */
    HAL_GPIO_WritePin(POWER_CTRL_GPIO_Port, POWER_CTRL_Pin, GPIO_PIN_RESET);
    TIM3->CCR3 = 0;  /* 0% duty = constant LOW */
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

  /* Debug: Display raw ADC value to verify ADC is working
   * Expected: ~0 when ADC input grounded, ~400-800 at 48-51V */
  uint32_t adc_avg = 0;
  for (uint8_t i = 0; i < ADC_BUFFER_SIZE; i++)
  {
    adc_avg += adc_buffer[i];
  }
  adc_avg /= ADC_BUFFER_SIZE;
  snprintf(line_buffer, sizeof(line_buffer), "ADC:%lu", adc_avg);
  OLED_ShowString(3, 1, line_buffer);
  OLED_ShowString(3, 16, "X");  /* LUX - X at right side */

  /* Display diagnostic info - check ADC and TIM2 status */
  uint32_t adc_cr2 = ADC1->CR2;
  uint8_t exttrig = (adc_cr2 & ADC_CR2_EXTTRIG) ? 1 : 0;
  uint8_t tim2_cr1 = (TIM2->CR1 & TIM_CR1_CEN) ? 1 : 0;
  /* Display actual threshold values from defines (convert float to int for printf) */
  snprintf(line_buffer, sizeof(line_buffer), "TER%u%u%u(%d,%d)",
           tim2_cr1, exttrig, adc_data_ready,
           (int)VBUS_LOWER_THRESHOLD, (int)VBUS_UPPER_THRESHOLD);
  OLED_ShowString(4, 1, line_buffer);

  /* Send periodic debug info via UART */
  printf("[MONITOR] Vbus=%.1fV ADC=%lu State=%s\r\n",
         vbus_voltage, adc_avg,
         (current_state == STATE_CUTOFF) ? "CUTOFF" : "CONDUCTION");
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
