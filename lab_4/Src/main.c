/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
 
*/

#include "main.h"
#include "stm32f0xx_hal.h"

void SystemClock_Config(void);
char glb_rx_data;
int glb_rx_flag;
char* glb_buf;
char* cmd2;
int match_str(char txd[], char lcl[]);
int id_cmd(char txd[]);
int glb_buf_idx = 0;
char** glb_strs;
int glb_cmd_count = 0;
void led_cmd_cntl(void);
void tx_char(char data);
void tx_string(char arr[]);
void tx_string_no_carriage(char arr[]);
void clear_buff(void);

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  cmd2 = NULL;
  
 // 1- Enable GPIO C clock
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; 

 // +++++++++++++++ SET LEDS +++++++++++++++++++++
 // 2- Set GPIOB MODE to ALTERNATE FUNCTION (  )
	
 //SET MODER TO AF FOR PB8 and PB9
    GPIOB->MODER |= GPIO_MODER_MODER11_1 | GPIO_MODER_MODER10_1;
		
    GPIOB->MODER &= ~(GPIO_MODER_MODER11_0 | GPIO_MODER_MODER10_0);

    GPIOB->AFR[1] |= (0x4U 	<< (2*4)) | 	//POS 2 FOR PB10  TX
					   (0x4U 	<< (3*4));	//POS 3 FOR PB11  RX

    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;  // USART 3 Clock ENABLE
	
	USART3->BRR = HAL_RCC_GetSysClockFreq()/115200;	// SET TX/RX BAUD RATE TO 115200
	USART3->CR1 |= USART_CR1_TE | USART_CR1_RE;	 	// ENABLE TX/RX
	USART3->CR1 |= USART_CR1_RXNEIE;
	USART3->CR1 |= USART_CR1_UE;						// ENABLE USART3
		 
    // RX INTERRUPT
    NVIC_EnableIRQ(USART3_4_IRQn);
    NVIC_SetPriority(USART3_4_IRQn,0);

  // 1- Enable GPIO C clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; 

// 2- Set GPIOC MODE to GENERAL OUTPUT ( 01 )


  //SET LSB PC8
   	GPIOC->MODER |= (1 << 16);
  //SET LSB PC7
   	GPIOC->MODER |= (1 << 14);
  //SET LSB PC9
   	GPIOC->MODER |= (1 << 18);
  //SET LSB PC6
   	GPIOC->MODER |= (1 << 12);
	
  /*
  PC6 = RED
  PC7 = BLUE
  PC8 = ORANGE
  PC9 = GREEN
  PA0 = User Button
  */

// INITIALIZE PC9 as HIGH
  GPIOC->ODR |= ((1<<9));
	
  // INITIALIZE PC8 as LOW
  	GPIOC->ODR &= ~((1<<8));

  	char buffy[100];
  	int i;
  	for(i = 0; i < 99; ++i)
  		buffy[i] = '\0';

  	glb_buf = buffy;

  		tx_string("\n\r");
    	tx_string("\t\t------------------------------------------------------\t\t\n\r");
  		tx_string("\t\t|          WELCOME TO UART LED CONTROL               |\t\t\n\r");
  		tx_string("\t\t------------------------------------------------------\t\t\n\r");
  		tx_string("\t\t|          ENTER THE FOLLOWING COMMANDS:             |\t\t\n\r");
  		tx_string("\t\t|                                                    |\t\t\n\r");  
  		tx_string("\t\t|              FIRST SELECT COLOR:                   |\t\t\n\r");
  		tx_string("\t\t|                                                    |\t\t\n\r");
  		tx_string("\t\t|           'red <ENTER>' FOR RED LED                |\t\t\n\r");
  		tx_string("\t\t|           'blue <ENTER>' FOR BLUE LED              |\t\t\n\r");
  		tx_string("\t\t|           'orange <ENTER>' FOR ORANGE LED          |\t\t\n\r");
  		tx_string("\t\t|           'green <ENTER>' FOR GREEN LED            |\t\t\n\r");
  		tx_string("\t\t|                                                    |\t\t\n\r");  
  		tx_string("\t\t|              SECOND SELECT COMMAND:                |\t\t\n\r");  
  		tx_string("\t\t|                                                    |\t\t\n\r");  
  		tx_string("\t\t|           'off <ENTER>' TO TURN OFF                |\t\t\n\r");  
  		tx_string("\t\t|           'on <ENTER>' TO TURN ON                  |\t\t\n\r");  
  		tx_string("\t\t|           'toggle <ENTER>' TO SWITCH STATE         |\t\t\n\r");  
  		tx_string("\t\t|                                                    |\t\t\n\r");
  		tx_string("\t\t------------------------------------------------------\t\t\n\r");
  		tx_string("\n\r\n\r");
  		tx_string("LED>>  ");
  
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  	
  }
 

}

int id_cmd(char txd[])
{

	if(match_str(txd, "red\r")) 		{return 0;}
	if(match_str(txd, "blue\r")) 		{return 1;}
	if(match_str(txd, "orange\r")) 		{return 2;}
	if(match_str(txd, "green\r")) 		{return 3;}
	if(match_str(txd, "off\r")) 		{return 4;}
	if(match_str(txd, "on\r")) 			{return 5;}
	if(match_str(txd, "toggle\r")) 		{return 6;}
	tx_string("FAILED ID\n\r");
	return -1;
}

int match_str(char txd[], char lcl[])
{
		int idx = 0;
		char temp = 48;
	while(txd[idx] == lcl[idx]){
		if(txd[idx] == '\r' || lcl[idx] == '\0'){
		return 1;
		}
	idx++;
	}
	return 0;
}	

void tx_char(char data)
{	
	while( (USART3->ISR & 0x80) == 0){}
	/* HAL_Delay(2000); */ /* PART 1 & 2 ONLY */
	USART3->TDR = (uint8_t)data;	
}

void tx_string(char arr[])
{
	char* i = arr;
	while(*i != '\0')
	{tx_char(*i++);}		
}

void tx_string_no_carriage(char arr[])
{
	char* i = arr;
	while(*i != '\0')
	{if(*i != '\r') tx_char(*i++); else {tx_char('\n');}}		
}
/**
  * @brief System Clock Configuration
  * @retval None
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
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
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

void clear_buff(void)
{
	int i;
	for(i = 0; i < 99; ++i)
		glb_buf[i] = '\0';
}

void set_led(int pos, int cmd)
{
	if(pos == -1) {return;}

	switch(cmd)
	{
		case 4: GPIOC->ODR &= ~(GPIO_ODR_0 << (pos + 6)); break;
		case 5:	GPIOC->ODR |= GPIO_ODR_0 << (pos + 6); break;
		case 6:	GPIOC->ODR ^= GPIO_ODR_0 << (pos + 6); break;
		default: break;
	}

}

void USART3_4_IRQHandler(void)
{
  glb_rx_data = USART3->RDR;
  int id1 = -1;
  int id2 = -1;

  if(glb_buf_idx == 0){tx_string("\rLED>>  ");}
  if(glb_rx_data != '\r') {tx_char(glb_rx_data);}

  glb_buf[glb_buf_idx++] = glb_rx_data;

  if(glb_rx_data == '\r' && cmd2 == NULL) 
  {
  	glb_buf[glb_buf_idx++] = '\0';
  	cmd2 = &glb_buf[glb_buf_idx]; 
  	tx_string("\n\r\rCMD>>  ");
  }
  else if(glb_rx_data == '\r' && cmd2 != NULL)
  {
    glb_buf[glb_buf_idx++] = '\0';

    id1 = id_cmd(glb_buf);
    id2 = id_cmd(cmd2);

  	set_led(id1, id2);
  	glb_buf_idx = 0;
  	cmd2 = NULL;
  	clear_buff();
  	glb_rx_flag = 1;
  	tx_string("\n\r\LED>>  ");
  	return;
  }

  if(glb_buf_idx > 98)	{ glb_buf_idx = 0;}
  glb_rx_flag = 1;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
