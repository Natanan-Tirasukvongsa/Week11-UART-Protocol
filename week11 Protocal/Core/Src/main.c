/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <stdlib.h>
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define UARTDEBUG
#define MAX_PACKET_LEN 255
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
typedef struct _UartStructure
{
	UART_HandleTypeDef *huart;
	uint16_t TxLen, RxLen;
	uint8_t *TxBuffer;
	uint16_t TxTail, TxHead;
	uint8_t *RxBuffer;
	uint16_t RxTail; //RXHeadUseDMA

} UARTStucrture;

UARTStucrture UART2 =
{ 0 };

uint8_t MainMemory[255] =
{ 0 };

typedef enum
{
	DNMXP_idle,
	DNMXP_1stHeader,
	DNMXP_2ndHeader,
	DNMXP_3rdHeader,
	DNMXP_Reserved,
	DNMXP_ID,
	DNMXP_LEN1,
	DNMXP_LEN2,
	DNMXP_Inst,
	DNMXP_ParameterCollect,
	DNMXP_CRCAndExecute

} DNMXPState;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void UARTInit(UARTStucrture *uart);

void UARTResetStart(UARTStucrture *uart);

uint32_t UARTGetRxHead(UARTStucrture *uart);

int16_t UARTReadChar(UARTStucrture *uart);

void UARTTxDumpBuffer(UARTStucrture *uart);

void UARTTxWrite(UARTStucrture *uart, uint8_t *pData, uint16_t len);

unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr,
		unsigned short data_blk_size);
void DynamixelProtocal2(uint8_t *Memory, uint8_t MotorID, int16_t dataIn,
		UARTStucrture *uart);
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
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	UART2.huart = &huart2;
	UART2.RxLen = 255;
	UART2.TxLen = 255;
	UARTInit(&UART2);
	UARTResetStart(&UART2);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		int16_t inputChar = UARTReadChar(&UART2);
		if (inputChar != -1)
		{
#ifdef UARTDEBUG
			char temp[32];
			sprintf(temp, "Recived [%d]\r\n", inputChar);
			UARTTxWrite(&UART2, (uint8_t*) temp, strlen(temp));
#else
			DynamixelProtocal2(MainMemory, 1, inputChar, &UART2);
#endif

		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		UARTTxDumpBuffer(&UART2);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

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
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	/* DMA1_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void UARTInit(UARTStucrture *uart)
{
	//dynamic memory allocate
	uart->RxBuffer = (uint8_t*) calloc(sizeof(uint8_t), UART2.RxLen);
	uart->TxBuffer = (uint8_t*) calloc(sizeof(uint8_t), UART2.TxLen);
	uart->RxTail = 0;
	uart->TxTail = 0;
	uart->TxHead = 0;

}

void UARTResetStart(UARTStucrture *uart)
{
	HAL_UART_Receive_DMA(uart->huart, uart->RxBuffer, uart->RxLen);
}
uint32_t UARTGetRxHead(UARTStucrture *uart)
{
	return uart->RxLen - __HAL_DMA_GET_COUNTER(uart->huart->hdmarx);
}
int16_t UARTReadChar(UARTStucrture *uart)
{
	int16_t Result = -1; // -1 Mean no new data

	//check Buffer Position
	if (uart->RxTail != UARTGetRxHead(uart))
	{
		//get data from buffer
		Result = uart->RxBuffer[uart->RxTail];
		uart->RxTail = (uart->RxTail + 1) % uart->RxLen;

	}
	return Result;

}
void UARTTxDumpBuffer(UARTStucrture *uart)
{
	static uint8_t MultiProcessBlocker = 0;

	if (uart->huart->gState == HAL_UART_STATE_READY && !MultiProcessBlocker)
	{
		MultiProcessBlocker = 1;

		if (uart->TxHead != uart->TxTail)
		{
			//find len of data in buffer (Circular buffer but do in one way)
			uint16_t sentingLen =
					uart->TxHead > uart->TxTail ?
							uart->TxHead - uart->TxTail :
							uart->TxLen - uart->TxTail;

			//sent data via DMA
			HAL_UART_Transmit_DMA(uart->huart, &(uart->TxBuffer[uart->TxTail]),
					sentingLen);
			//move tail to new position
			uart->TxTail = (uart->TxTail + sentingLen) % uart->TxLen;

		}
		MultiProcessBlocker = 0;
	}
}
void UARTTxWrite(UARTStucrture *uart, uint8_t *pData, uint16_t len)
{
	//check data len is more than buffur?
	uint16_t lenAddBuffer = (len <= uart->TxLen) ? len : uart->TxLen;
	// find number of data before end of ring buffer
	uint16_t numberOfdataCanCopy =
			lenAddBuffer <= uart->TxLen - uart->TxHead ?
					lenAddBuffer : uart->TxLen - uart->TxHead;
	//copy data to the buffer
	memcpy(&(uart->TxBuffer[uart->TxHead]), pData, numberOfdataCanCopy);

	//Move Head to new position

	uart->TxHead = (uart->TxHead + lenAddBuffer) % uart->TxLen;
	//Check that we copy all data That We can?
	if (lenAddBuffer != numberOfdataCanCopy)
	{
		memcpy(uart->TxBuffer, &(pData[numberOfdataCanCopy]),
				lenAddBuffer - numberOfdataCanCopy);
	}
	UARTTxDumpBuffer(uart);

}


void DynamixelProtocal2(uint8_t *Memory, uint8_t MotorID, int16_t dataIn,
		UARTStucrture *uart)
{
	//all Static Variable
	static DNMXPState State = DNMXP_idle;
	static uint16_t datalen = 0;
	static uint16_t CollectedData = 0;
	static uint8_t inst = 0;
	static uint8_t parameter[256] =
	{ 0 };
	static uint16_t CRCCheck = 0;
	static uint16_t packetSize = 0;
	static uint16_t CRC_accum;

	//State Machine
	switch (State)
	{
	case DNMXP_idle:
		if (dataIn == 0xFF)
			State = DNMXP_1stHeader;
		break;
	case DNMXP_1stHeader:
		if (dataIn == 0xFF)
			State = DNMXP_2ndHeader;
		break;
	case DNMXP_2ndHeader:
		if (dataIn == 0xFD)
			State = DNMXP_3rdHeader;
		else if (dataIn == 0xFF)
			; //do nothing
		else
			State = DNMXP_idle;
		break;
	case DNMXP_3rdHeader:
		if (dataIn == 0x00)
			State = DNMXP_Reserved;
		else
			State = DNMXP_idle;
		break;
	case DNMXP_Reserved:
		if ((dataIn == MotorID) | (dataIn == 0xFE))
			State = DNMXP_ID;
		else
			State = DNMXP_idle;
		break;
	case DNMXP_ID:
		datalen = dataIn & 0xFF;
		State = DNMXP_LEN1;
		break;
	case DNMXP_LEN1:
		datalen |= (dataIn & 0xFF) << 8;
		State = DNMXP_LEN2;
		break;
	case DNMXP_LEN2:
		inst = dataIn;
		State = DNMXP_Inst;
		break;
	case DNMXP_Inst:
		if (datalen > 3)
		{
			parameter[0] = dataIn;
			CollectedData = 1; //inst 1 + para[0] 1
			State = DNMXP_ParameterCollect;
		}
		else
		{
			CRCCheck = dataIn & 0xff;
			State = DNMXP_CRCAndExecute;
		}

		break;
	case DNMXP_ParameterCollect:

		if (datalen-3 > CollectedData)
		{
			parameter[CollectedData] = dataIn;
			CollectedData++;
		}
		else
		{
			CRCCheck = dataIn & 0xff;
			State = DNMXP_CRCAndExecute;
		}
		break;
	case DNMXP_CRCAndExecute:
		CRCCheck |= (dataIn & 0xff) << 8;
		//Check CRC
		CRC_accum = 0;
		packetSize = datalen + 7;
		//check overlapse buffer
		if (uart->RxTail - packetSize >= 0) //not overlapse
		{
			CRC_accum = update_crc(CRC_accum,
					&(uart->RxBuffer[uart->RxTail - packetSize]),
					packetSize - 2);
		}
		else//overlapse
		{
			uint16_t firstPartStart = uart->RxTail - packetSize + uart->RxLen;
			CRC_accum = update_crc(CRC_accum, &(uart->RxBuffer[firstPartStart]),
					uart->RxLen - firstPartStart);
			CRC_accum = update_crc(CRC_accum, uart->RxBuffer, uart->RxTail - 2);

		}

		if (CRC_accum == CRCCheck)
		{
			switch (inst)
			{
			case 0x01:// ping
			{
				//create packet template
				uint8_t temp[] =
				{ 0xff, 0xff, 0xfd, 0x00, 0x00, 0x05, 0x00, 0x55, 0x00, 0x00,
						0x00 };
				//config MotorID
				temp[4] = MotorID;
				//calcuate CRC
				uint16_t crc_calc = update_crc(0, temp, 9);
				temp[9] = crc_calc & 0xff;
				temp[10] = (crc_calc >> 8) & 0xFF;
				//Sent Response Packet
				UARTTxWrite(uart, temp, 11);
				break;
			}

			case 0x02://READ
			{
				uint16_t startAddr = (parameter[0]&0xFF)|(parameter[1]<<8 &0xFF);
				uint16_t numberOfDataToRead = (parameter[2]&0xFF)|(parameter[3]<<8 &0xFF);
				uint8_t temp[] = {0xff,0xff,0xfd,0x00,0x00,0x00,0x00,0x55,0x00};
				temp[4] = MotorID;
				temp[5] = (numberOfDataToRead + 4) & 0xff ; // +inst+err+crc1+crc2
				temp[6] = ((numberOfDataToRead + 4)>>8) & 0xff ;
				uint16_t crc_calc = update_crc(0, temp, 9);
				crc_calc = update_crc(crc_calc ,&(Memory[startAddr]),numberOfDataToRead);
				uint8_t crctemp[2];
				crctemp[0] = crc_calc&0xff;
				crctemp[1] = (crc_calc>>8)&0xff;
				UARTTxWrite(uart, temp,9);
				UARTTxWrite(uart, &(Memory[startAddr]),numberOfDataToRead);
				UARTTxWrite(uart, crctemp,2);
				break;
			}
			case 0x03://WRITE
			{
				//LAB
			}
			default: //Unknown Inst
			{
				uint8_t temp[] =
				{ 0xff, 0xff, 0xfd, 0x00, 0x00, 0x05, 0x00, 0x55, 0x02, 0x00,
						0x00 };
				temp[4] = MotorID;
				uint16_t crc_calc = update_crc(0, temp, 9);
				temp[9] = crc_calc & 0xff;
				temp[10] = (crc_calc >> 8) & 0xFF;
				UARTTxWrite(uart, temp, 11);

				break;
			}
			}
		}
		else //crc error
		{
			uint8_t temp[] =
			{ 0xff, 0xff, 0xfd, 0x00, 0x00, 0x05, 0x00, 0x55, 0x03, 0x00, 0x00 };
			temp[4] = MotorID;
			uint16_t crc_calc = update_crc(0, temp, 9);
			temp[9] = crc_calc & 0xff;
			temp[10] = (crc_calc >> 8) & 0xFF;
			UARTTxWrite(uart, temp, 11);
		}
		State = DNMXP_idle;
		break;
	}

}
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr,
		unsigned short data_blk_size)
{
	unsigned short i, j;
	unsigned short crc_table[256] =
	{ 0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011, 0x8033,
			0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022, 0x8063,
			0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072, 0x0050,
			0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041, 0x80C3,
			0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0,
			0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0,
			0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
			0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082, 0x8183,
			0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192, 0x01B0,
			0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1, 0x01E0,
			0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1, 0x81D3,
			0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140,
			0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173,
			0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
			0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132, 0x0110,
			0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101, 0x8303,
			0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312, 0x0330,
			0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321, 0x0360,
			0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371, 0x8353,
			0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0,
			0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
			0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2, 0x83A3,
			0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2, 0x0390,
			0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381, 0x0280,
			0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291, 0x82B3,
			0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3,
			0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0,
			0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
			0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252, 0x0270,
			0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261, 0x0220,
			0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231, 0x8213,
			0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202 };

	for (j = 0; j < data_blk_size; j++)
	{
		i = ((unsigned short) (crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
		crc_accum = (crc_accum << 8) ^ crc_table[i];
	}

	return crc_accum;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
