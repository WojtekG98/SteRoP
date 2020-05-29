/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dfsdm.h"
#include "dma.h"
#include "lcd.h"
#include "quadspi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include "audio_fw_glo.h"
#include "smr_glo.h"
#include "sdr_glo.h"
#include <math.h>
#include "stm32l476g_discovery_qspi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AUDIO_REC 160
#define READ_ID_CMD 0x9E
#define LCD_COM0	LCD_RAM_REGISTER0
#define LCD_COM0_1	LCD_RAM_REGISTER1
#define LCD_COM1	LCD_RAM_REGISTER2
#define LCD_COM1_1	LCD_RAM_REGISTER3
#define LCD_COM2	LCD_RAM_REGISTER4
#define LCD_COM2_1	LCD_RAM_REGISTER5
#define LCD_COM3	LCD_RAM_REGISTER6
#define LCD_COM3_1	LCD_RAM_REGISTER7

#define LCD_SEG8	(1U << 8)
#define LCD_SEG25	(1U << 25)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
LCD_HandleTypeDef hlcd;

QSPI_CommandTypeDef sCommand;
uint8_t ID_REG[3];

int32_t RecBuf[AUDIO_REC];
int32_t PlayBuf[AUDIO_REC];
uint8_t dane_do_wyslania[8] = {1, 2, 3, 4, 5, 6, 7, 8};
uint8_t dane_do_odebrania[8];

int32_t					SoundDetectorOutputBuff[AUDIO_REC];

void					*persistent_mem_ptr;
void					*scratch_mem_ptr;
sdr_static_param_t 		sdr_input_static_param_ptr;
sdr_dynamic_param_t 	sdr_input_dynamic_param_ptr;
buffer_t				*sdr_input_buffer;
buffer_t				*sdr_output_buffer;

int32_t 				ratio_threshold1_dB = 11;
int32_t					ratio_threshold2_dB = 9;
int32_t 				hangover_number_of_frames = 4;
int32_t					learning_frames_number = 2;

uint8_t DmaRecHalfBuffCplt = 0;
uint8_t DmaRecBuffCplt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void SoundDetector_Init(void);

int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 50);
	return len;
}

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

	uint16_t i = 0;
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
  MX_DFSDM1_Init();
  MX_USART2_UART_Init();
  //MX_QUADSPI_Init();
  MX_LCD_Init();
  /* USER CODE BEGIN 2 */

  //sCommand.InstructionMode 	 = QSPI_INSTRUCTION_1_LINE;
  //sCommand.Instruction 		 = READ_ID_CMD;
  //sCommand.AddressMode 		 = QSPI_ADDRESS_NONE;
  //sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  //sCommand.DataMode			 = QSPI_DATA_1_LINE;
  //sCommand.DummyCycles		 = 0;
  //sCommand.Address			 = 0;
  //sCommand.DdrMode			 = QSPI_DDR_MODE_DISABLE;
  //sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  //sCommand.SIOOMode			 = QSPI_SIOO_INST_EVERY_CMD;
  //sCommand.NbData			 = 3;

  //HAL_QSPI_Command(&hqspi, &sCommand, 1000);
  //HAL_QSPI_Receive(&hqspi, ID_REG, 1000);
  //asm("NOP");
  //printf("%d, %d, %d\n\r", ID_REG[0], ID_REG[1], ID_REG[2]);
  printf("BSP_QSPI_Init(): %d\r\n",BSP_QSPI_Init());
  printf("BSP_QSPI_Erase_Sector(): %d\r\n", BSP_QSPI_Erase_Sector(0));
  HAL_Delay(1000);
  printf("BSP_QSPI_Write(): %d\r\n",BSP_QSPI_Write(dane_do_wyslania, 0, 8));
  printf("BSP_QSPI_Read(): %d\r\n",BSP_QSPI_Read(dane_do_odebrania, 0, 8));
  for (i = 0; i < 8; i++)
	printf("%d\r\n", dane_do_odebrania[i]);
  HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, RecBuf, AUDIO_REC);
  i = 0;
  SoundDetector_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_LCD_Clear(&hlcd);
  HAL_LCD_Write(&hlcd, LCD_COM0, 0xFFFFFFFF, 0xFFFFFFFF);
  HAL_LCD_Write(&hlcd, LCD_COM0_1, 0xFF, 0xFF);
  HAL_LCD_Write(&hlcd, LCD_COM1, 0xFFFFFFFF, 0xFFFFFFFF);
  HAL_LCD_Write(&hlcd, LCD_COM1_1, 0xFF, 0xFF);
  HAL_LCD_Write(&hlcd, LCD_COM2, 0xFFFFFFFF, 0xFFFFFFFF);
  HAL_LCD_Write(&hlcd, LCD_COM2_1, 0xFF, 0xFF);
  HAL_LCD_Write(&hlcd, LCD_COM3, 0xFFFFFFFF, 0xFFFFFFFF);
  HAL_LCD_Write(&hlcd, LCD_COM3_1, 0xFF, 0xFF);
  HAL_LCD_UpdateDisplayRequest(&hlcd);
  HAL_Delay(1500);
  HAL_LCD_Clear(&hlcd);
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /*
	  if((DmaRecHalfBuffCplt == 1) || (DmaRecBuffCplt == 1))
	  {


		  StartIndex = (DmaRecBuffCplt == 1) ? 512 : 0;
		  for(i = 0; i < AUDIO_REC; i++)
		  {
			  PlayBuf[i] = RecBuf[i + StartIndex] >> 8;
		  }
		  if( SDR_ERROR_NONE != sdr_process(sdr_input_buffer, sdr_output_buffer, persistent_mem_ptr))
		  {
			  printf("sdr_process error: %ld\r\n",sdr_process(sdr_input_buffer, sdr_output_buffer, persistent_mem_ptr));
		  }
	      if(DmaRecHalfBuffCplt  == 1)
	      {
	        DmaRecHalfBuffCplt  = 0;
	      }
	      else
	      {
	        DmaRecBuffCplt = 0;
	      }
		  if( SDR_ERROR_NONE != sdr_getConfig(&sdr_input_dynamic_param_ptr, persistent_mem_ptr))
		  {
			  printf("sdr_getConfig error: %ld",sdr_getConfig(&sdr_input_dynamic_param_ptr, persistent_mem_ptr));
		  }
		  else
			  printf("state= %ld\r\n", sdr_input_dynamic_param_ptr.output_state);
		  if(sdr_input_dynamic_param_ptr.output_state == 1)
		  		  {
		  			  HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, GPIO_PIN_RESET);
		  		  	  HAL_GPIO_WritePin(LD_G_GPIO_Port, LD_G_Pin, GPIO_PIN_SET);
		  		  }
		  		  else
		  		  {
		  			  HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, GPIO_PIN_SET);
		  			  HAL_GPIO_WritePin(LD_G_GPIO_Port, LD_G_Pin, GPIO_PIN_RESET);
		  		  }
	  }
	  */

	  //if(DmaRecHalfBuffCplt==1)
	  //{
		 // for (i = 0; i < AUDIO_REC/2; i++)
		//	  PlayBuf[i] = RecBuf[i]>>8;

//		  DmaRecHalfBuffCplt=0;
	//  }
	  //if (DmaRecBuffCplt==1)
	  //{
		//  for (i = AUDIO_REC/2; i < AUDIO_REC; i++)
			//  PlayBuf[i] = RecBuf[i]>>8;

	//	  DmaRecBuffCplt=0;
	  //}
	  //HAL_LCD_Write(&hlcd, LCD_COM3, ~(LCD_SEG8), LCD_SEG8);
	  //HAL_LCD_UpdateDisplayRequest(&hlcd);
	  //HAL_Delay(100);
	  //HAL_LCD_Write(&hlcd, LCD_COM2, ~(LCD_SEG8), LCD_SEG8);
	  //HAL_LCD_UpdateDisplayRequest(&hlcd);
	  //HAL_Delay(100);
	  //HAL_LCD_Write(&hlcd, LCD_COM3, ~(LCD_SEG25), LCD_SEG25);
	  //HAL_LCD_UpdateDisplayRequest(&hlcd);
	  //HAL_Delay(100);
	  HAL_LCD_Write(&hlcd, LCD_COM2, 0xFFFFFFFF, 0xFFFFFFFF);
	  HAL_LCD_Write(&hlcd, LCD_COM2_1, 0xFF, 0xFF);
	  HAL_Delay(100);
	  if(DmaRecBuffCplt == 1){
		  for (i = 0; i < AUDIO_REC; i++)
		  {
			  PlayBuf[i]=RecBuf[i]>>8;
		  }
		  if( SDR_ERROR_NONE != sdr_process(sdr_input_buffer, sdr_output_buffer, persistent_mem_ptr))
		  {
			  printf("sdr_process error: %ld\r\n",sdr_process(sdr_input_buffer, sdr_output_buffer, persistent_mem_ptr));
		  }
		  if( SDR_ERROR_NONE != sdr_getConfig(&sdr_input_dynamic_param_ptr, persistent_mem_ptr))
		  {
			  printf("sdr_getConfig error: %ld",sdr_getConfig(&sdr_input_dynamic_param_ptr, persistent_mem_ptr));
		  }
		 // for (i = 0; i < AUDIO_REC; i++)
		//	  printf("%ld\r\n",RecBuf[i]);
		  //printf("\r\n\r\nEND\r\n\r\n");
		  if(sdr_input_dynamic_param_ptr.output_state == 1)
	  		  {
	  			  HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, GPIO_PIN_RESET);
	  		  	  HAL_GPIO_WritePin(LD_G_GPIO_Port, LD_G_Pin, GPIO_PIN_SET);
	  		  }
	  		  else
	  		  {
	  			  HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, GPIO_PIN_SET);
	  			  HAL_GPIO_WritePin(LD_G_GPIO_Port, LD_G_Pin, GPIO_PIN_RESET);
	  		  }
	  DmaRecBuffCplt = 0;
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_DFSDM1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void SoundDetector_Init(void)
{
	/* CRC enable and reset */
	__HAL_RCC_CRC_CLK_ENABLE();
	CRC->CR = CRC_CR_RESET;

	/* Memory allocation */
	persistent_mem_ptr = malloc(sdr_persistent_mem_size);
	scratch_mem_ptr    = malloc(sdr_scratch_mem_size);
	sdr_input_buffer   = (buffer_t *)malloc(sizeof(buffer_t));
	sdr_output_buffer   = (buffer_t *)malloc(sizeof(buffer_t));

	/* sdr_reset() */
	if(SDR_ERROR_NONE != sdr_reset(persistent_mem_ptr, scratch_mem_ptr))
		printf("sdr_reset error: %ld\r\n", sdr_reset(persistent_mem_ptr, scratch_mem_ptr));

	/* static_param initialization */
	sdr_input_static_param_ptr.sampling_rate = 8000;
	sdr_input_static_param_ptr.buffer_size   = AUDIO_REC;
	sdr_input_static_param_ptr.learning_frame_nb = learning_frames_number;

	/* dynamic_param initialization */
	sdr_input_dynamic_param_ptr.enable = 1;
	sdr_input_dynamic_param_ptr.ratio_thr1_dB = ratio_threshold1_dB;
	sdr_input_dynamic_param_ptr.ratio_thr2_dB = ratio_threshold2_dB;
	sdr_input_dynamic_param_ptr.noise_pwr_min_dB = -60;
	sdr_input_dynamic_param_ptr.hangover_nb_frame = hangover_number_of_frames;

	/* sdr_setParam() */
	if(SDR_ERROR_NONE != sdr_setParam(&sdr_input_static_param_ptr, persistent_mem_ptr))
		printf("sdr_setParam error: %ld\r\n", sdr_setParam(&sdr_input_static_param_ptr, persistent_mem_ptr));

	/* sdr_setConfig() */
	if(SDR_ERROR_NONE != sdr_setConfig(&sdr_input_dynamic_param_ptr, persistent_mem_ptr))
		printf("sdr_setConfig error: %ld\r\n", sdr_setConfig(&sdr_input_dynamic_param_ptr, persistent_mem_ptr));

	/* buffers init */
	sdr_input_buffer -> nb_channels = 1;
	sdr_input_buffer -> nb_bytes_per_Sample = 3;
	sdr_input_buffer -> data_ptr = PlayBuf;
	sdr_input_buffer -> buffer_size = AUDIO_REC;
	sdr_input_buffer -> mode = 0;

	sdr_output_buffer -> nb_channels = 1;
	sdr_output_buffer -> nb_bytes_per_Sample = 3;
	sdr_output_buffer -> data_ptr = SoundDetectorOutputBuff;
	sdr_output_buffer -> buffer_size = AUDIO_REC;
	sdr_output_buffer -> mode = 0;

	/* sdr_setConfig() */
	if(SDR_ERROR_NONE != sdr_setConfig(&sdr_input_dynamic_param_ptr, persistent_mem_ptr))
		printf("sdr_setConfig error: %ld\n", sdr_setConfig(&sdr_input_dynamic_param_ptr, persistent_mem_ptr));

}

void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
	//DmaRecHalfBuffCplt = 1;
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
	DmaRecBuffCplt = 1;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
