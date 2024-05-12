/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
uint32_t blink_frequency_ms = 1000; // Initial blinking frequency is 1 second
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	 uint8_t button_state = 0;
	 uint8_t last_button_state = 0;
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
  GPIO_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //1.1
	  // Binary 0: Both LEDs off
//	         GPIOD->ODR &= ~(1 << 15); // Clear bit 15 of GPIOA_ODR
//	         GPIOD->ODR &= ~(1 << 12); // Clear bit 12 of GPIOB_ODR
//	         HAL_Delay(1000);
//
//	         // Binary 2: Blue LED on
//	         GPIOD->ODR |= (1 << 15);  // Set bit 15 of GPIOA_ODR
//	         GPIOD->ODR &= ~(1 << 12); // Clear bit 12 of GPIOB_ODR
//	         HAL_Delay(1000);
//
//	         // Binary 3: Both LEDs on
//	         GPIOD->ODR |= (1 << 15);  // Set bit 15 of GPIOA_ODR
//	         GPIOD->ODR |= (1 << 12);  // Set bit 12 of GPIOB_ODR
//	         HAL_Delay(1000);
	  //1.2
	  // Red LED (PD14) blinks every second
//	         GPIOD->ODR ^= (1 << 13); // Toggle bit 14 of GPIOD_ODR
//	         HAL_Delay(1000);
//
//	         // Orange LED (PD13) blinks every three seconds
//	         GPIOD->ODR ^= (1 << 14); // Toggle bit 13 of GPIOD_ODR
//	         HAL_Delay(2000);
//	         GPIOD->ODR ^= (1 << 14); // Toggle bit 13 of GPIOD_ODR again
//	         HAL_Delay(1000);
	 //1.3papildoma
	         // Check if the user button is pressed
//	                 if (GPIOA->IDR & (1 >> 0)) // Button is pressed
//	                 {
//	                     // Turn off the blue LED (PD15)
//	                     GPIOD->ODR |= (1 << 15);
//	                 }
//	                 else // Button is released
//	                 {
//	                     // Turn on the blue LED (PD15)
//	                     GPIOD->ODR &= ~(1 << 15);
//	                 }
   //1.4papildoma
	                 // Read the button state
	                         button_state = (GPIOA->IDR & GPIO_PIN_0) ? 1 : 0;

	                         // Check for button press
	                         if (button_state == 1 && last_button_state == 0) {
	                             // Increase the blinking frequency by 500ms
	                             blink_frequency_ms += 500;
	                             HAL_Delay(200); // Add a small delay to debounce the button
	                         }

	                         // Update last button state
	                         last_button_state = button_state;

	                         // Toggle the orange LED
	                         GPIOD->ODR ^= GPIO_PIN_13;

	                         // Wait according to the current blinking frequency
	                         HAL_Delay(blink_frequency_ms);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */


/* USER CODE BEGIN 4 */
void GPIO_Init(void)
{
	// Enable GPIOD and GPIOA clock
	    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOAEN;

	    // Configure GPIOA PIN_15 as output
	    GPIOD->MODER &= ~(3U << (15 * 2)); // Clear bits for pin 15
	    GPIOD->MODER |= (1U << (15 * 2));  // Set bit 1 for pin 15

	    // Configure GPIOB PIN_12 as output
	    GPIOD->MODER &= ~(3U << (12 * 2)); // Clear bits for pin 12
	    GPIOD->MODER |= (1U << (12 * 2));  // Set bit 1 for pin 12

	    // Configure GPIOD PIN_13, PIN_14 as output
	    GPIOD->MODER &= ~(0xFU << (13 * 2)); // Clear bits for pins 13, 14
	    GPIOD->MODER |= (0x5U << (13 * 2));  // Set bit 1 for pin 13, 14

	    // Configure GPIOA PIN_0 as input (user button)
	    GPIOA->MODER &= ~(3U << (0 * 2)); // Clear bits for pin 0
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
