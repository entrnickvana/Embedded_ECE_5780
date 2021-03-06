/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

	uint32_t temp;
	uint32_t button_is_pressed;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  //HAL_Init(); // Reset of all peripherals, init the Flash and Systick
SystemClock_Config(); //Configure the system clock
/* This example uses HAL library calls to control
the GPIOC peripheral. You'll be redoing this code
with hardware register access. */
	
	// ***DISABLED***
//  __HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock in the RCC
	
	
// Set up a configuration struct to pass to the initialization function
	
// --- USEFUL INFO ---
/*
	  __IO uint32_t AHBENR;     !< RCC AHB peripheral clock register,                           Address offset: 0x14 
*/	
	
GPIO_InitTypeDef initStr = {GPIO_PIN_8 | GPIO_PIN_9,
GPIO_MODE_OUTPUT_PP,
GPIO_SPEED_FREQ_LOW,
GPIO_NOPULL};

button_is_pressed = 0;
// $$$$$ GPIOC->MODER = 0x50000;

// $$$$$ GPIOC->MODER |= 0b00001000; // Sets the 3rd bit

// $$$$$ GPIOC->MODER &= ~(0b00001000); // Clears the 3rd bit

// $$$$$ GPIOC->MODER ^= 0b00001000; // Inverts the 3rd bit



// $$$$$ RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable peripheral clock to TIMER2



// Sets the 3rd bit in the GPIOC_MODER register
// $$$$$  GPIOC->MODER |= (1 << 3);

// Sets the 3rd and 5th bits in the GPIOC_MODER register
// $$$$$  GPIOC->MODER |= (1 << 3) | (1 << 5);

// Clears the 3rd and 5th bits in the GPIOC_MODER register
// $$$$$  GPIOC->MODER &= ~((1 << 3) | (1 << 5));
	
//HAL_GPIO_Init(GPIOC, &initStr); // Initialize pins PC8 & PC9

//------------------------------------------------------------------------
// NEW CODE --------------------------------------------------------------

// 1- Enable GPIO C clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; 



// 2- Set GPIOC MODE to GENERAL OUTPUT ( 01 )

	//clear MSB PC8
		GPIOC->MODER &= ~((1 << 17));
	//SET LSB PC8
	 	GPIOC->MODER |= (1 << 16);

	//clear MSB PC9
		GPIOC->MODER &= ~((1 << 19));
	//SET LSB PC8
	 	GPIOC->MODER |= (1 << 18);
	
	//SET BUTTON MODE
		//GPIOA->MODER |= (1 << 0);
		
	//SET BUTTON SPEED
	
	//
		
		
		
// 3- Set GPIOC OTYPER to OUTPUT PUSH-PULL

	//SET bit 9, 8 to 0 for PC8, PC9
		GPIOC->OTYPER &= ~(( 1<<8) | (1<<9));


// 4- Set GPIOC OSPEEDR to x0

	//SET bit 18, 16 to 0
		GPIOC->OTYPER &= ~(( 1<<18) | (1<<16));
	
	
// 5- Set GPIOC PUDR to 0 for bits 19-16

	// SET bit 19, 18, 17, 16 to 0
		GPIOC->PUPDR &= 	~((1<<19) 		//MSB PC9 
											| (1<<18) 		//LSB PC9
											| (1<<17) 		//MSB PC8
											| (1<<16));		//LSB PC8
		
// 6-  
	
/*
	PC7 = BLUE
	PC8 = Orange
	PC9 = Green
	PA0 = User Button
*/

//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // Start PC8 high

	// INITIALIZE PC9 as HIGH
	GPIOC->ODR |= ((1<<9));
	
	// INITIALIZE PC8 as LOW
	GPIOC->ODR &= ~((1<<8));


	while (1) {
		


	

	HAL_Delay(100); // Delay 200ms
	
	// TOGGLE bits 8,9
		GPIOC->ODR ^= 	((1<<9) 	//INVERT PC9
									| (1<<8)); 	//INVERT PC8
	}

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
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

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
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
