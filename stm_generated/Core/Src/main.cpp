// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ether Form Inc.
#include "main.h"
#include <Canard_gnss_node.h>
#include <Stm_settings.h>

#include <stdint.h>

// The ID jumpers select a node ID of NODE_ID_OFFSET plus the jumper value (0 to 15)
#define NODE_ID_OFFSET 16

// Default receiver measurement period. 100 ms is 10 Hz, the fastest the receiver supports.
#define DEFAULT_MEAS_PERIOD_MS 100

// Replaces libstdc++'s default terminate handler, which would link in the C++ symbol
// demangler (about 32 KB of flash) just to print a message nobody can see.
namespace __gnu_cxx
{
void __verbose_terminate_handler()
{
  __disable_irq();
  while (1)
  {
  }
}
} // namespace __gnu_cxx

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  // Reset of all peripherals, Initializes the Flash interface and the Systick.
  HAL_Init();

  // Configure the system clock 
  SystemClock_Config();

  // Initialize all configured peripherals
  MX_GPIO_Init();

  // The ID jumpers form a 4 bit number. The pins have external pull-ups, so a fitted jumper
  // (shorted to GND) reads low and sets its bit. GPIO1 is the LSB. The node ID is that number
  // plus NODE_ID_OFFSET, so with no jumpers fitted it is NODE_ID_OFFSET.
  uint8_t jumpers = 0;
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET)
  {
    jumpers |= (1 << 0);
  }
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET)
  {
    jumpers |= (1 << 1);
  }
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_RESET)
  {
    jumpers |= (1 << 2);
  }
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET)
  {
    jumpers |= (1 << 3);
  }
  uint8_t can_id = NODE_ID_OFFSET + jumpers;

  // A node ID stored in flash overrides the jumpers. It is used as is (no offset). DroneCAN
  // node IDs must be 1-127 (0 is reserved for anonymous nodes), so anything else is ignored.
  uint8_t stored_can_id;
  if (Stm_settings::read(Stm_settings::node_id, stored_can_id) && stored_can_id >= 1 && stored_can_id <= 127)
  {
    can_id = stored_can_id;
  }

  uint8_t update_rate;
  if (!Stm_settings::read(Stm_settings::update_rate, update_rate))
  {
    update_rate = DEFAULT_MEAS_PERIOD_MS;
  }

  // This needs to be static as we will blow through the stack otherwise
  static Canard_gnss_node node{can_id, update_rate};

  // Serves the CAN node and the GNSS receiver forever
  node.run();

  // This should never be reached
  while (1)
  {
    Error_Handler();
  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 PB2 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

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
