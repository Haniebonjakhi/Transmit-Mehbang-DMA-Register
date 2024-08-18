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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

//UART_HandleTypeDef huart2;  
//DMA_HandleTypeDef hdma_uart2_tx; 

//uint8_t txBuffer[] = "Mehbang";
//uint16_t txBufferSize = sizeof(txBuffer) - 1;
//uint8_t txIndex =0 ;

uint8_t data1[] = "Mehbang";
uint16_t sizeofdata = 7;

uint8_t a = 0;
uint8_t b = 0;
uint8_t c = 0;
uint8_t d = 0;
uint8_t e = 0;
uint8_t f = 0;
uint8_t h = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// DMA
//void DMA1_Channel7_Init(void) {
//    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

//    DMA1_Channel7->CCR = 0;
//    while (DMA1_Channel7->CCR & DMA_CCR_EN);

//    DMA1_Channel7->CPAR = (uint32_t)&USART2->DR;    
//    DMA1_Channel7->CMAR = (uint32_t)txBuffer;
//    DMA1_Channel7->CNDTR = txBufferSize;              

//	
//    DMA1_Channel7->CCR = DMA_CCR_MINC |      
//                         DMA_CCR_DIR |            
//                         DMA_CCR_TCIE |         
//                         DMA_CCR_PL_1;            
//}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

//	DMA1_Channel7->CCR &= ~DMA_CCR_EN; 
//  DMA1_Channel7->CNDTR = txBufferSize;
//  DMA1_Channel7->CCR |= DMA_CCR_EN;
//	
//}



void USART2_DMA_Init()  
{  
		a=1;
		RCC->AHBENR |= RCC_AHBENR_DMA1EN;
		DMA1_Channel7->CCR = 0;
		USART2->CR3 |= USART_CR3_DMAT; 
	
    // Ensure that DMA Channel 7 is ready for the transmission  
    while (DMA1_Channel7->CCR & DMA_CCR_EN); // Wait until the current DMA transfer (if any) is complete  
		b=1;
	
    // Set peripheral and memory addresses  
    DMA1_Channel7->CPAR = (uint32_t)&USART2->DR;  // Peripheral address (USART TDR register)  
    DMA1_Channel7->CMAR = (uint32_t)data1;            // Memory address (Data buffer)  
    DMA1_Channel7->CNDTR = sizeofdata;                     // Number of data items to transfer  

    // Configure the DMA transfer  
    DMA1_Channel7->CCR |= DMA_CCR_DIR;               // Set data direction (Memory to Peripheral)  
    DMA1_Channel7->CCR |= DMA_CCR_MINC;              // Enable memory address increment  
    DMA1_Channel7->CCR |= DMA_CCR_TCIE;              // Enable transfer complete interrupt  
		DMA1_Channel7->CCR |= DMA_CCR_PL_1;

} 


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	c=1;
	DMA1_Channel7->CCR &= ~DMA_CCR_EN;
	DMA1_Channel7->CNDTR = sizeofdata;                     
	DMA1_Channel7->CCR |= DMA_CCR_EN;
}


//void usart_config(void)  
//{  
//		e=1;
//    // 1. Enable the USART by writing the UE bit in USART_CR1 register to 1.  
//    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;   


//    // 2. Configure USART2 settings   
//    huart2.Instance = USART2;  
//    huart2.Init.BaudRate = 115200;         
//    huart2.Init.WordLength = UART_WORDLENGTH_8B;  
//    huart2.Init.StopBits = UART_STOPBITS_1;  
//    huart2.Init.Parity = UART_PARITY_NONE;  
//    huart2.Init.Mode = UART_MODE_TX;     
//    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;  
//    huart2.Init.OverSampling = UART_OVERSAMPLING_16;  
//    
//    HAL_UART_Init(&huart2);  

//    USART2->CR1 |= USART_CR1_UE; 
//}  

//void DMA_Init(void)  
//{  
//			f=1;
//	
//    // Enable DMA1 clock  
//    __HAL_RCC_DMA1_CLK_ENABLE();  

//    // Configure DMA for USART2 TX  
//    hdma_uart2_tx.Instance = DMA1_Channel7;  // Use DMA1 Channel 7 for TX  
//    hdma_uart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;  
//    hdma_uart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;  
//    hdma_uart2_tx.Init.MemInc = DMA_MINC_ENABLE;  
//    hdma_uart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;  
//    hdma_uart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;  
//    hdma_uart2_tx.Init.Mode = DMA_NORMAL;  
//    hdma_uart2_tx.Init.Priority = DMA_PRIORITY_LOW;  
//    HAL_DMA_Init(&hdma_uart2_tx);  

//    // Link DMA TX to USART2  
//    __HAL_LINKDMA(&huart2, hdmatx, hdma_uart2_tx);  
//}  

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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	USART2_DMA_Init();
	HAL_TIM_Base_Start_IT(&htim2);
	//	DMA_Init();
	//	usart_config();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL15;
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
