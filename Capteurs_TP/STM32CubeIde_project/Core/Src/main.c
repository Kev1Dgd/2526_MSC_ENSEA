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
#include "tools.h"
#include "functions.h"
#include "compass.h"

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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN PV */
CompassCalib_t acc_calib;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_LPUART1_UART_Init(void);
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
	MX_I2C1_Init();
	MX_LPUART1_UART_Init();
	/* USER CODE BEGIN 2 */
	setup();

	HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\r\n--- Initialisation du MPU9250 ---\r\n", 39, HAL_MAX_DELAY);
	checkMPU9250_ID();   // Vérifie le WHO_AM_I du MPU
	InitSensors();       // Initialise les registres de base

	// === Calibration du gyroscope ===
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\r\n\r\nDébut de la calibration du gyroscope...\r\n", 47, HAL_MAX_DELAY);
	GyrCalib(1000);  // Prend 1000 échantillons immobiles pour calculer les offsets
	ReadOffsetGyro();

	// === Calibration « light » du magnétomètre ===
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\r\nDébut de la calibration lite du magnétomètre...\r\n", 52, HAL_MAX_DELAY);
	MagCalibLite(300, 50);  // 300 mesures, 50 ms d'écart
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)"Calibration lite du magnétomètre terminée !\r\n", 49, HAL_MAX_DELAY);

	// === Calibration plus poussée du magnétomètre ===
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\r\nDébut de la calibration hard & soft iron magnétomètre...\r\n", strlen("\r\nDébut de la calibration hard & soft iron magnétomètre...\r\n"), HAL_MAX_DELAY);

	double mag_bias[3], mag_scale[3];
	MagCalib(&hi2c1, mag_bias, mag_scale);

	// Affichage des résultats de calibration
	char calib_msg[100];
	snprintf(calib_msg, sizeof(calib_msg),
			"\r\nBias: X=%.3f | Y=%.3f | Z=%.3f\r\nScale: X=%.3f | Y=%.3f | Z=%.3f\r\n",
			mag_bias[0], mag_bias[1], mag_bias[2],
			mag_scale[0], mag_scale[1], mag_scale[2]);
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)calib_msg, strlen(calib_msg), HAL_MAX_DELAY);

	HAL_UART_Transmit(&hlpuart1,(uint8_t*)"Calibration hard & soft iron terminée", strlen("Calibration hard & soft iron terminée"), HAL_MAX_DELAY);

	// --- Lecture d'une mesure magnétomètre calibrée ---
	double mag_results[3];
	char mag_msg[100];

	MagMeasureMsgCalib(mag_results, mag_msg, sizeof(mag_msg), mag_bias, mag_scale);
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)mag_msg, strlen(mag_msg), HAL_MAX_DELAY);

	// --- Tests éventuels ---
	//TestAccelSensZ();
	//TestGyroNoise(1000);


	// --- Test Calib Mag MATLAB ---
	/*int N = 500;
	for (int i=1; i<N; i++) {
	    MagMeasureMsgCalib(mag_data, mag_msg, sizeof(mag_msg), mag_bias, mag_scale);
	    HAL_UART_Transmit(&hlpuart1, (uint8_t*)mag_msg, strlen(mag_msg), HAL_MAX_DELAY);
	    HAL_Delay(100); // 100 ms entre chaque mesure
	}*/
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		//loop();
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		char temp_msg[50];
		char gyro_msg[100];
		char acc_msg[100];
		char mag_msg[100];
		char compass_msg[100];

		double acc_data[3], gyro_data[3], mag_data[3];
		double roll, pitch, heading;

		// Mesures
		TempMeasureMsg(temp_msg, sizeof(temp_msg));
		GyroMeasureMsg(gyro_data, gyro_msg, sizeof(gyro_msg));
		AccMeasureMsg(acc_data, acc_msg, sizeof(acc_msg));
		MagMeasureMsg(mag_data, mag_msg, sizeof(mag_msg));

		// Boussole compensée
		 CompassMeasureMsg(acc_data, compass_msg, sizeof(compass_msg), &acc_calib);

		// Transmission UART dans l'ordre désiré
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)temp_msg, strlen(temp_msg), HAL_MAX_DELAY);
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)gyro_msg, strlen(gyro_msg), HAL_MAX_DELAY);
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)acc_msg, strlen(acc_msg), HAL_MAX_DELAY);
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)mag_msg, strlen(mag_msg), HAL_MAX_DELAY);

		HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\r\n", strlen("\r\n"), HAL_MAX_DELAY);

		// Calcul de l'orientation compensée
		TiltCompensatedCompass(acc_data, mag_data, &roll, &pitch, &heading);

		snprintf(compass_msg, sizeof(compass_msg),
		         "Roulis=%.2f deg | Tangage=%.2f deg | NordMag=%.2f deg\r\n",
		         roll, pitch, heading);
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)compass_msg, strlen(compass_msg), HAL_MAX_DELAY);


		HAL_Delay(1000);
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
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x40B285C2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief LPUART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_LPUART1_UART_Init(void)
{

	/* USER CODE BEGIN LPUART1_Init 0 */

	/* USER CODE END LPUART1_Init 0 */

	/* USER CODE BEGIN LPUART1_Init 1 */

	/* USER CODE END LPUART1_Init 1 */
	hlpuart1.Instance = LPUART1;
	hlpuart1.Init.BaudRate = 115200;
	hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
	hlpuart1.Init.StopBits = UART_STOPBITS_1;
	hlpuart1.Init.Parity = UART_PARITY_NONE;
	hlpuart1.Init.Mode = UART_MODE_TX_RX;
	hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&hlpuart1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN LPUART1_Init 2 */

	/* USER CODE END LPUART1_Init 2 */

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
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA5 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
