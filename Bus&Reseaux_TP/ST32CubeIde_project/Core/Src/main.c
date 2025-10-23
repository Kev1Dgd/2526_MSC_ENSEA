/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
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
#define BMP280_I2C_ADDR (0x77 << 1) 	// SDO = VDDIO & I2C sur 7 bits et non 8
#define REG_ID 0xD0
#define REG_ID_VAL 0x58
#define NORMAL_MODE 0b11
#define REG_RESET 0xE0
#define REG_CALIB 0x88
#define CALIB_LENGTH 26
#define REG_PRESS_MSB 0xF7
#define REG_TEMP_MSB  0xFA
#define REG_CTRL_MEAS 0xF4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int chr){
	HAL_UART_Transmit(&huart2, (uint8_t*) &chr, 1, HAL_MAX_DELAY);
	return chr;
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

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	printf("\n=== Initialisation du BMP280... ===\r\n");

	/* Variables */
	uint8_t id_reg = REG_ID;
	uint8_t id;
	uint8_t buf[2];
	uint8_t calib[CALIB_LENGTH];
	uint8_t data[6];

	/* Étalonnage */
	uint16_t dig_T1;
	int16_t dig_T2, dig_T3;
	uint16_t dig_P1;
	int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
	int32_t t_fine;

	/* 1 - Présence du capteur */
	HAL_I2C_Master_Transmit(&hi2c1, BMP280_I2C_ADDR, &id_reg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, BMP280_I2C_ADDR, &id, 1, HAL_MAX_DELAY);

	if (id == 0x58)
		printf("BMP280 détecté (ID = 0x%02X)\r\n", id);
	else
		printf("Erreur : BMP280 non détecté (ID lu = 0x%02X)\r\n", id);

	/* 2 - Configuration du capteur */
	buf[0] = REG_CTRL_MEAS;
	buf[1] = 0x57;
	HAL_I2C_Master_Transmit(&hi2c1, BMP280_I2C_ADDR, buf, 2, HAL_MAX_DELAY);
	HAL_Delay(100);

	/* 3 - Coefficients d'étalonage */
	uint8_t reg_calib = REG_CALIB;
	HAL_I2C_Master_Transmit(&hi2c1, BMP280_I2C_ADDR, &reg_calib, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, BMP280_I2C_ADDR, calib, CALIB_LENGTH, HAL_MAX_DELAY);

	/* Cf Datasheet */
	dig_T1 = (uint16_t)(calib[1] << 8 | calib[0]);
	dig_T2 = (int16_t)(calib[3] << 8 | calib[2]);
	dig_T3 = (int16_t)(calib[5] << 8 | calib[4]);
	dig_P1 = (uint16_t)(calib[7] << 8 | calib[6]);
	dig_P2 = (int16_t)(calib[9] << 8 | calib[8]);
	dig_P3 = (int16_t)(calib[11] << 8 | calib[10]);
	dig_P4 = (int16_t)(calib[13] << 8 | calib[12]);
	dig_P5 = (int16_t)(calib[15] << 8 | calib[14]);
	dig_P6 = (int16_t)(calib[17] << 8 | calib[16]);
	dig_P7 = (int16_t)(calib[19] << 8 | calib[18]);
	dig_P8 = (int16_t)(calib[21] << 8 | calib[20]);
	dig_P9 = (int16_t)(calib[23] << 8 | calib[22]);

	printf("\n== Coefficients d’étalonnage lus ==\r\n");

	printf("dig_T1 = %u\r\n", dig_T1);
	printf("dig_T2 = %d\r\n", dig_T2);
	printf("dig_T3 = %d\r\n", dig_T3);

	printf("dig_P1 = %u\r\n", dig_P1);
	printf("dig_P2 = %d\r\n", dig_P2);
	printf("dig_P3 = %d\r\n", dig_P3);
	printf("dig_P4 = %d\r\n", dig_P4);
	printf("dig_P5 = %d\r\n", dig_P5);
	printf("dig_P6 = %d\r\n", dig_P6);
	printf("dig_P7 = %d\r\n", dig_P7);
	printf("dig_P8 = %d\r\n", dig_P8);
	printf("dig_P9 = %d\r\n", dig_P9);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		/* 4 -  Lecture des datas */
		printf("\n=== Lecture des données === \r\n");
		uint8_t reg_data = REG_PRESS_MSB;

		HAL_I2C_Master_Transmit(&hi2c1, BMP280_I2C_ADDR, &reg_data, 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(&hi2c1, BMP280_I2C_ADDR, data, 6, HAL_MAX_DELAY);

		uint32_t adc_P = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | (data[2] >> 4);
		uint32_t adc_T = ((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | (data[5] >> 4);

		/* 5 - Température compensée */
		int32_t var1, var2, T;

		var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
		var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
		t_fine = var1 + var2;

		T = (t_fine * 5 + 128) >> 8;

		int64_t var1_p, var2_p, p;

		var1_p = ((int64_t)t_fine) - 128000;
		var2_p = var1_p * var1_p * (int64_t)dig_P6;
		var2_p = var2_p + ((var1_p * (int64_t)dig_P5) << 17);
		var2_p = var2_p + (((int64_t)dig_P4) << 35);
		var1_p = ((var1_p * var1_p * (int64_t)dig_P3) >> 8) + ((var1_p * (int64_t)dig_P2) << 12);
		var1_p = (((((int64_t)1) << 47) + var1_p) * ((int64_t)dig_P1)) >> 33;

		if (var1_p == 0) continue;
		p = 1048576 - adc_P;
		p = (((p << 31) - var2_p) * 3125) / var1_p;
		var1_p = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
		var2_p = (((int64_t)dig_P8) * p) >> 19;
		p = ((p + var1_p + var2_p) >> 8) + (((int64_t)dig_P7) << 4);

		/* --- 6 : Conversion et affichage en virgule fixe --- */
		int32_t temperature_centi = T;               // Température en centi-degrés
		int32_t pression_centi_hPa = (int32_t)(p / 256); // Pression en centi-hPa

		printf("Temp = %ld.%02ld °C | Pression = %ld.%02ld hPa\r\n",
				temperature_centi / 100, abs(temperature_centi % 100),
				pression_centi_hPa / 100, abs(pression_centi_hPa % 100));
	}
	HAL_Delay(500);
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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
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
