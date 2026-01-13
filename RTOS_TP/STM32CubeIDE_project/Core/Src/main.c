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
#include "cmsis_os.h"
#include "usart.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../shell/shell.h"

#include "FreeRTOS.h"
#include "queue.h"

#include "spi.h"
#include "adxl345.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef struct {
    uint32_t period_ms;
    uint32_t count;
    char message[32];   // message max = 31 caractères
} spam_cmd_t;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
QueueHandle_t led_cmd_queue;
QueueHandle_t spam_cmd_queue;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void task_led(void *unused);
int led(int argc, char **argv);
void task_spam(void *unused);
int spam(int argc, char **argv);

//void task_goofy(void *unused);  // Partie 3.1
//void task_overflow(void *unused);  // Partie 3.2
int stats(int argc, char **argv);  // Partie 3.4

int adxl_test(int argc, char **argv);
static int adxl_read4(int argc, char **argv);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);

	return ch;
}

int fonction(int argc, char ** argv)  // Toujours mettre les même arguments
{
	printf("Je suis une fonction bidon\r\n");
	printf("Nombre d'arguments : %d\r\n", argc);

	for (int i = 0; i < argc; i++)
	{
		printf("Argument [%d] : %s\r\n", i, argv[i]);
	}

	return 0;  // toujours mettre return 0
}

int addition(int argc, char **argv)
{
	if (argc != 3)
	{
		printf("Error: expected two arguments\r\n");
		return -1;
	}

	int a = atoi(argv[1]);  // conversion string → int (atoi = ASCII to interger)
	int b = atoi(argv[2]);

	int somme = a + b;
	printf("%d + %d = %d\r\n", a, b, somme);

	return 0;
}

void task_shell(void * unused)
{
	shell_init();
	shell_add('f',	fonction,	"Une fonction inutile");
	shell_add('+', 	addition,	"Addition de deux nombres");
	shell_add('l', 	led,		"Controle de la LED (periode en ms)");
	shell_add('s', 	spam,		"Spam UART: periode, nb, message");
	shell_add('%', 	stats,		"Affiche stats FreeRTOS (tasks + runtime)");
	shell_add('t', 	adxl_test,	"Test ADXL345: lire DEVID (0x00)");
	shell_add('a', 	adxl_read4,	"ADXL345: lire 4 mesures (polling INT1)");
	shell_run();

	// Une tache ne doit *JAMAIS* retourner
	// Ici elle ne retourne pas parce qu'il y a une boucle infinie dans shell_run();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == LPUART1)	// LPUART1 si carte de ppz avec shield pour moteur
	{
		// Caractère reçu : Donner le sémaphore pour débloquer task_shell
		shell_uart_rx_callback();
	}
}

void task_led(void *unused)
{
	uint32_t period_ms = 0;      // période de clignotement en ms
	uint32_t new_period_ms = 0;

	// On part LED éteinte
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

	for (;;)
	{
		// On regarde s'il y a une nouvelle période envoyée par le shell
		if (xQueueReceive(led_cmd_queue, &new_period_ms, 0) == pdPASS)
		{
			period_ms = new_period_ms;
		}

		if (period_ms == 0)
		{
			// LED éteinte
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			vTaskDelay(pdMS_TO_TICKS(10));
		}
		// essayer de faire avec vTaskSuspend(0) <- 0 pour dire qu'on suspend la tâche en cours
		else
		{
			// On toggle la LED puis on attend "period_ms"
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			vTaskDelay(pdMS_TO_TICKS(period_ms));
		}
	}
}

int led(int argc, char **argv)
{
	if (argc != 2)
	{
		printf("Usage: l <periode_ms>\r\n");
		printf("Exemple: l 500   (clignote toutes les 500 ms)\r\n");
		printf("          l 0     (eteint la LED)\r\n");
		return -1;
	}

	uint32_t period_ms = (uint32_t)atoi(argv[1]);

	if (led_cmd_queue == NULL)
	{
		printf("Erreur: queue LED non initialisee\r\n");
		return -1;
	}

	// On envoie la nouvelle période à la tâche
	if (xQueueOverwrite(led_cmd_queue, &period_ms) != pdPASS)
	{
		printf("Erreur: impossible d'envoyer la commande LED\r\n");
		return -1;
	}

	printf("LED periode = %lu ms\r\n", (unsigned long)period_ms);
	return 0;
}

void task_spam(void *unused)
{
    spam_cmd_t cmd;

    for (;;)
    {
        // On attend une nouvelle commande (bloquant)
        if (xQueueReceive(spam_cmd_queue, &cmd, portMAX_DELAY) == pdPASS)
        {
            // On envoie "count" fois le message
            for (uint32_t i = 0; i <= cmd.count; i++)
            {
                printf("%s\r\n", cmd.message);
                vTaskDelay(pdMS_TO_TICKS(cmd.period_ms));
            }
        }
    }
}

int spam(int argc, char **argv)
{
    if (argc != 4)
    {
        printf("Usage: s <periode_ms> <nb_messages> <message>\r\n");
        printf("Exemple: s 500 10 hello\r\n");
        return -1;
    }

    if (spam_cmd_queue == NULL)
    {
        printf("Erreur: queue spam non initialisee\r\n");
        return -1;
    }

    spam_cmd_t cmd;
    cmd.period_ms = (uint32_t)atoi(argv[1]);
    cmd.count     = (uint32_t)atoi(argv[2]);

    // Copie simple du message (un seul mot pour l'instant)
    strncpy(cmd.message, argv[3], sizeof(cmd.message) - 1);
    cmd.message[sizeof(cmd.message) - 1] = '\0';

//    if (xQueueSend(spam_cmd_queue, &cmd, 0) != pdPASS)
//    {
//        printf("Erreur: impossible d'envoyer la commande spam\r\n");
//        return -1;
//    }

//    A mettre à la place de xQueueSend(spam_cmd_queue, &cmd, 0) ...
    if (xQueueOverwrite(spam_cmd_queue, &cmd) != pdPASS)
    {
    	printf("Erreur: impossible d'envoyer la commande SPAM\r\n");
    	return -1;
    }

    printf("Spam: %lu messages toutes les %lu ms, message=\"%s\"\r\n",
           (unsigned long)cmd.count,
           (unsigned long)cmd.period_ms,
           cmd.message);

    return 0;
}

////Tâche pour saturer la mémoire
//void task_goofy(void *unused)
//{
//    (void)unused;
//    for (;;)
//    {
//        vTaskDelay(portMAX_DELAY);   // dort pour toujours
//    }
//}

//void task_overflow(void *unused)
//{
//    (void)unused;
//
//    // Très gros tableau local => utilise la stack
//    volatile uint8_t big[1000];
//    for (int i = 0; i < (int)sizeof(big); i++) big[i] = (uint8_t)i;
//
//    // Forcer des context switches / appels kernel
//    for (;;)
//    {
//        vTaskDelay(pdMS_TO_TICKS(10));
//    }
//}

int stats(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    static char buf[1024];	// Doit être augmenté si on rajoute bcp de tâche par la suite

    printf("\r\n===== TASK LIST =====\r\n");
    vTaskList(buf);
    printf("%s\r\n", buf);

    printf("===== RUN TIME STATS =====\r\n");
    vTaskGetRunTimeStats(buf);
    printf("%s\r\n", buf);

    return 0;
}

int adxl_test(int argc, char **argv)
{
    uint8_t devid = 0;
    if (adxl345_read_reg(&hspi3, NSS_GPIO_Port, NSS_Pin, 0x00, &devid) != HAL_OK)
    {
        printf("SPI error\r\n");
        return -1;
    }
    printf("DEVID=0x%02X\r\n", devid);

    if (devid == 0xE5)
        printf("OK: communication SPI valide\r\n");
    else
        printf("Erreur : valeur attendue = 0xE5\r\n");

    return 0;
}

static int adxl_read4(int argc, char **argv)
{
    (void)argc; (void)argv;

    // Démarrer la mesure
    adxl345_write_reg(&hspi3, NSS_GPIO_Port, NSS_Pin, 0x2D, 0x08); // POWER_CTL = MEASURE

    // Activer l'interruption "Data Ready"
    adxl345_write_reg(&hspi3, NSS_GPIO_Port, NSS_Pin, 0x2E, 0x80); // INT_ENABLE = DATA_READY

    // Format des valeurs de sortie en g
    adxl345_write_reg(&hspi3, NSS_GPIO_Port, NSS_Pin, 0x31, 0x08); // DATA_FORMAT: FULL_RES, +/-2g


    int32_t sx = 0, sy = 0, sz = 0;

    for (int k = 0; k < 4; k++)
    {
        // Polling : attendre INT1 = HIGH (avec timeout pour éviter blocage infini)
        uint32_t t0 = HAL_GetTick();
        while (HAL_GPIO_ReadPin(INT_GPIO_Port, INT_Pin) == GPIO_PIN_RESET)
        {
            if ((HAL_GetTick() - t0) > 1000)   // 1s timeout
            {
                printf("Timeout: INT1 ne passe pas a HIGH\r\n");
                return -1;
            }
        }

        // Lire 6 octets: X0 X1 Y0 Y1 Z0 Z1
        uint8_t raw[6];
        if (adxl345_read_multi(&hspi3, NSS_GPIO_Port, NSS_Pin, 0x32, raw, 6) != HAL_OK)
        {
            printf("SPI error read_multi\r\n");
            return -1;
        }

        int16_t x = (int16_t)((raw[1] << 8) | raw[0]);
        int16_t y = (int16_t)((raw[3] << 8) | raw[2]);
        int16_t z = (int16_t)((raw[5] << 8) | raw[4]);

        sx += x; sy += y; sz += z;

        printf("[%d] X=%d Y=%d Z=%d\r\n", k, x, y, z);

        // Petite pause pour éviter de relire les mêmes mesures si INT reste haut
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Moyenne
    float x_avg = sx / 4.0f;
    float y_avg = sy / 4.0f;
    float z_avg = sz / 4.0f;

    // Conversion en g : 256 LSB / g en ±2g
    const float LSB_PER_G = 256.0f;
    float x_g = x_avg / LSB_PER_G;
    float y_g = y_avg / LSB_PER_G;
    float z_g = z_avg / LSB_PER_G;

    printf("AVG RAW: X=%.1f Y=%.1f Z=%.1f\r\n", x_avg, y_avg, z_avg);
    printf("AVG [g] : X=% .3f Y=% .3f Z=% .3f\r\n", x_g, y_g, z_g);

    return 0;
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
  MX_LPUART1_UART_Init();
  MX_TIM2_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

	printf("Boot avant FreeRTOS\r\n");		// Debug

	// Création des queues :
	led_cmd_queue = xQueueCreate(1, sizeof(uint32_t));
	if (led_cmd_queue == NULL)
	{
		printf("Erreur: impossible de creer la queue LED\r\n");
		Error_Handler();
	}
	vQueueAddToRegistry(led_cmd_queue, "LED_CMD_Q");  // Question 9 (Partie 3.3) pour nommer ma led_queue pour l'affichage de la mémoire dans le debugger

	spam_cmd_queue = xQueueCreate(1, sizeof(spam_cmd_t));
	if (spam_cmd_queue == NULL)
	{
	    printf("Erreur: impossible de creer la queue SPAM\r\n");
	    Error_Handler();
	}
	vQueueAddToRegistry(spam_cmd_queue, "SPAM_CMD_Q");  // Pareil, c'est pour nommer ma spam_queue pour l'affichage

	// Création des taches :
	if (xTaskCreate(task_shell, "Shell", 512, NULL, 1, NULL) != pdPASS) {
		    printf("Erreur creation task_shell\r\n");
		    Error_Handler();
		}

	if (xTaskCreate(task_led, "LED", 128, NULL, 1, NULL) != pdPASS) {
	    printf("Erreur creation task_led\r\n");
	    Error_Handler();
	}

	if (xTaskCreate(task_spam, "SPAM", 256, NULL, 1, NULL) != pdPASS) {
	    printf("Erreur creation task_spam\r\n");
	    Error_Handler();
	}

//	// Création de taches inutiles à l'infini (avant d'entrer dans le Scheduler)
//	int i = 0;
//	while (1)
//	{
//	    char name[16];
//	    snprintf(name, sizeof(name), "Goofy%02d", i);
//
//	    BaseType_t ok = xTaskCreate(task_goofy, name, 128, NULL, 1, NULL);
//	    if (ok != pdPASS)
//	    {
//	        printf("Erreur: xTaskCreate Goofy a echoue (i=%d)\r\n", i);
//	        Error_Handler();
//	    }
//
//	    i++;
//	}

//	if (xTaskCreate(task_overflow, "OVF", 64, NULL, 1, NULL) != pdPASS){
//		printf("Erreur creation task_overflow\r\n");
//		Error_Handler();
//	}



	vTaskStartScheduler();
	printf("ERREUR : Scheduler non demarre \r\n");

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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

/* USER CODE BEGIN 4 */
void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    printf("ERROR: FreeRTOS malloc failed (heap empty)\r\n");
    Error_Handler();
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;

    taskDISABLE_INTERRUPTS();

    // Allume une LED pour signaler l'erreur
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

//    // Breakpoint logiciel (le debugger s'arrête ici)
//    __BKPT(0);

    while (1) { }
}

void configureTimerForRunTimeStats(void)
{
    HAL_TIM_Base_Start(&htim2);
}

unsigned long getRunTimeCounterValue(void)
{
    return (unsigned long)__HAL_TIM_GET_COUNTER(&htim2);
}



/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
