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

#define BUFFERLENGTH 256

#define NCOLUMNS 6
#define NLINES 6

#define ENTERKEYBINDING 35
#define SPACEKEYBINDING 32 //TODO

#define SLEEPDURATION 5

#define MORSEUNITDURATION 300 // IN MS


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

char buffer[BUFFERLENGTH]; // buffer for holding the letters



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int read_mux_output()
{
	// returns value from MUX OUTPUT PIN
	// TODO
	return -1;
}

int setColumn(int columnNumber)
{
	// sets the column where a signal is sent through the demux
	// should change A0, A1, A2 state
	if (columnNumber < 0 || columnNumber > 5) //column selected is out of bounds
		return -1;


	HAL_GPIO_WritePin(GPIOA, A0_Pin, columnNumber % 2);
	HAL_GPIO_WritePin(GPIOA, A1_Pin, columnNumber >> 1 % 2);
	HAL_GPIO_WritePin(GPIOA, A2_Pin, columnNumber >> 2 % 2);

	return 0;
}

int setLine(int lineNumber)
{
	// sets the listened line where a signal is read through the MUX
	// should affect S0, S1, S2 state
	if (lineNumber < 0 || lineNumber > 5) //line selected is out of bounds
		return -1;


	HAL_GPIO_WritePin(GPIOA, S0_Pin, lineNumber % 2);
	HAL_GPIO_WritePin(GPIOA, S1_Pin, lineNumber >> 1 % 2);
	HAL_GPIO_WritePin(GPIOA, S2_Pin, lineNumber >> 2 % 2);

	return 0;
}

void beepBuzzer(int msDuration)
{
	HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, 1);
	HAL_Sleep(msDuration);
	HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, 0);
	HAL_Sleep(MORSEUNITDURATION);

}

void readLetter(char letter)
{
	// letter: position in the keyboard

	// character: 1 for short, 2 for short, 0 for end
	char letters[32][5] = {
	    {1, 2, 0, 0, 0},  // A: .-
	    {2, 1, 1, 1, 0},  // B: -...
	    {2, 1, 2, 1, 0},  // C: -.-.
	    {2, 1, 1, 0, 0},  // D: -..
	    {1, 0, 0, 0, 0},  // E: .
	    {1, 1, 2, 1, 0},  // F: ..-.
	    {2, 2, 1, 0, 0},  // G: --.
	    {1, 1, 1, 1, 0},  // H: ....
	    {1, 1, 0, 0, 0},  // I: ..
	    {1, 2, 2, 2, 0},  // J: .---
	    {2, 1, 2, 0, 0},  // K: -.-
	    {1, 2, 1, 1, 0},  // L: .-..
	    {2, 2, 0, 0, 0},  // M: --
	    {2, 1, 0, 0, 0},  // N: -.
	    {2, 2, 2, 0, 0},  // O: ---
	    {1, 2, 2, 1, 0},  // P: .--.
	    {2, 2, 1, 2, 0},  // Q: --.-
	    {1, 2, 1, 0, 0},  // R: .-.
	    {1, 1, 1, 0, 0},  // S: ...
	    {2, 0, 0, 0, 0},  // T: -
	    {1, 1, 2, 0, 0},  // U: ..-
	    {1, 1, 1, 2, 0},  // V: ...-
	    {1, 2, 2, 0, 0},  // W: .--
	    {2, 1, 1, 2, 0},  // X: -..-
	    {2, 1, 2, 2, 0},  // Y: -.--
	    {2, 2, 1, 1, 0},  // Z: --..
	    {0, 0, 0, 0, 0},  // Padding to make array length 32
	    {0, 0, 0, 0, 0},  // Padding to make array length 32
	    {0, 0, 0, 0, 0},  // Padding to make array length 32
	    {0, 0, 0, 0, 0},  // Padding to make array length 32
	    {0, 0, 0, 0, 0},  // Padding to make array length 32
	    {0, 0, 0, 0, 0},  // Padding to make array length 32
	    {0, 0, 0, 0, 0},  // Padding to make array length 32
	    {0, 0, 0, 0, 0}   // Padding to make array length 32
	};

	  if(letter == SPACEKEYBINDING)
	  {
		  HAL_Sleep(7 * MORSEUNITDURATION); // gap between words
		  return;
	  }

	for(int i = 0; i < 5; i++)
	{
		if(letters[letter][i] == 0) //end of the word
		{
			return;
		}

		if(letters[letter][i] == 1) // dot
		{
			beepBuzzer(MORSEUNITDURATION);
		}
		else
		{
			beepBuzzer(MORSEUNITDURATION * 3);
		}
	}

	HAL_Sleep(3 * MORSEUNITDURATION); // gap between letters

}

void readBuffer(int bufferCursor)
{
	for(int i = 0; i < bufferCursor; i++)
	{
		// read each character (buffer[i])
		readLetter(buffer[i]);

	}
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

  int bufferCursor = 0;

  for(size_t i = 0; i < BUFFERLENGTH; i++)
  {
	  buffer[i] = 0;
  }

  char keyPressed;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  for(int column = 0; column < NCOLUMNS; column++)
	  {
		  setColumn(column);
		  for(int line = 0; line < NLINES; line++)
		  {
			  setLine(line);

			  HAL_Sleep(SLEEPDURATION);
			  if(read_mux_output()) //key is pressed
			  {
				  keyPressed = column + line * NLINES;
				  if(keyPressed == ENTERKEYBINDING) // we should read the buffer and bip the buzzer
				  {
					  readBuffer(bufferCursor);
					  bufferCursor = 0;

				  }
				  else
				  {
					  buffer[bufferCursor] = keyPressed;
					  bufferCursor++;
				  }

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Buzzer_Pin|A0_Pin|A1_Pin|A2_Pin
                          |S0_Pin|S1_Pin|S2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Buzzer_Pin A0_Pin A1_Pin A2_Pin
                           S0_Pin S1_Pin S2_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin|A0_Pin|A1_Pin|A2_Pin
                          |S0_Pin|S1_Pin|S2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MUX_OUTPUT_Pin */
  GPIO_InitStruct.Pin = MUX_OUTPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MUX_OUTPUT_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
