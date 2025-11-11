/*
 * fonctions.c
 *
 *  Created on: Nov 8, 2024
 *      Author: Nicolas SIMOND
 */


#include "tools.h"
#include "string.h"
#include "stdio.h"

/* === Définition des broches === */
#define LED_G_PORT      GPIOA
#define LED_G_PIN       GPIO_PIN_5    // LED verte Nucleo
#define BTN_PORT        GPIOC
#define BTN_PIN         GPIO_PIN_13   // Bouton bleu B1
#define ACTIVE_LOW      0             // 0 si le bouton est actif-haut, 1 sinon

/* === Déclarations externes === */
extern UART_HandleTypeDef hlpuart1;  // UART pour le terminal
extern I2C_HandleTypeDef hi2c1;      // Bus I2C utilisé

/* === Variables globales === */
uint8_t prompt[15] = "\nSTM32G431 >> ";
uint8_t message[100];
uint8_t foundAddresses[128];  // tableau pour stocker les périphériques détectés
uint8_t foundCount = 0;

/* === Lecture de l’état du bouton === */
static inline uint8_t button_is_pressed(void) {
    GPIO_PinState s = HAL_GPIO_ReadPin(BTN_PORT, BTN_PIN);
#if ACTIVE_LOW
    return (s == GPIO_PIN_RESET);   // appui = 0 logique
#else
    return (s == GPIO_PIN_SET);     // appui = 1 logique
#endif
}



/**
 * @brief Initialisation UART + message d’accueil.
 * Affiche un message de bienvenue sur le terminal série
 * et prépare l’environnement de test.
 */
void setup(void) {
    strcpy((char*)message, "MSC 2025 - Capteurs\r\n");
    HAL_UART_Transmit(&hlpuart1, prompt, strlen((char*)prompt), HAL_MAX_DELAY);
    HAL_UART_Transmit(&hlpuart1, message, strlen((char*)message), HAL_MAX_DELAY);

    test_LED();
    I2C_scan();
}


/**
 * @brief Boucle principale : clignotement LED + affichage compteur UART
 */
void loop(void) {
    static uint32_t cnt = 0;

    /* Périodes de clignotement : normal et accéléré (en ms) */
    const uint32_t T_NORMAL = 500;
    const uint32_t T_FAST   = 150;

    /* Choix de la période selon l’état du bouton */
    uint32_t period = button_is_pressed() ? T_FAST : T_NORMAL;

    /* Bascule de la LED */
    HAL_GPIO_TogglePin(LED_G_PORT, LED_G_PIN);
    HAL_Delay(period);

    /* Affichage du compteur */
    snprintf((char*)message, sizeof(message), "cnt : %lu\r\n", cnt++);
    HAL_UART_Transmit(&hlpuart1, prompt, strlen((char*)prompt), HAL_MAX_DELAY);
    HAL_UART_Transmit(&hlpuart1, message, strlen((char*)message), HAL_MAX_DELAY);
}

/**
 * @brief Scan du bus I2C pour détecter les périphériques connectés.
 * Balaye toutes les adresses possibles (0x00 à 0x7F) et affiche
 * sur le terminal les adresses des périphériques répondant.
 *
 * @note Utilise la fonction HAL_I2C_IsDeviceReady().
 */
void I2C_scan(void) {
    foundCount = 0;
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\r\n*** Début du scan I2C ***\r\n", 30, HAL_MAX_DELAY);

    for (uint8_t address = 0; address < 128; address++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(address << 1), 1, 10) == HAL_OK) {
            foundAddresses[foundCount++] = address;
            snprintf((char*)message, sizeof(message), "Adresse détectée : %d\r\n", address);
            HAL_UART_Transmit(&hlpuart1, message, strlen((char*)message), HAL_MAX_DELAY);
        }
    }

    snprintf((char*)message, sizeof(message),
             "\r\nNombre total d’adresses détectées : %d\r\n\r\n", foundCount);
    HAL_UART_Transmit(&hlpuart1, message, strlen((char*)message), HAL_MAX_DELAY);
}


/**
 * @brief Test du fonctionnement des LEDs verte et rouge.
 *
 * Allume la LED pendant 1 seconde puis l'éteint.
 * Permet de vérifier la configuration GPIO et la fonction HAL_Delay.
 */
void test_LED(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);   // LED verte ON
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // LED verte OFF
}


/**
 * @brief Enregistre les mesures du magnétomètre dans un fichier CSV
 *
 * @param filename Nom du fichier CSV
 * @param Bx Tableau des mesures X
 * @param By Tableau des mesures Y
 * @param Bz Tableau des mesures Z
 * @param N Nombre de mesures
 * @return 0 si OK, -1 si erreur
 */
int SaveMagToCSV(const char *filename, float *Bx, float *By, float *Bz, uint16_t N)
{
    FILE *fp = fopen(filename, "w");
    if (fp == NULL) return -1;  // Erreur ouverture fichier

    // Écrire l'en-tête
    fprintf(fp, "X,Y,Z\n");

    // Écrire les données
    for (uint16_t i = 0; i < N; i++)
    {
        fprintf(fp, "%.3f,%.3f,%.3f\n", Bx[i], By[i], Bz[i]);
    }

    fclose(fp);
    return 0;
}

#define SAMPLE_RATE_HZ 100

void TestMagCSV(void)
{
    uint16_t N = 2 * SAMPLE_RATE_HZ;
    float Bx[N], By[N], Bz[N];

    MagMeasure2s(Bx, By, Bz, SAMPLE_RATE_HZ);

    if (SaveMagToCSV("MagData.csv", Bx, By, Bz, N) == 0)
    {
        HAL_UART_Transmit(&hlpuart1, (uint8_t*)"CSV créé avec succès !\r\n", 26, HAL_MAX_DELAY);
    }
    else
    {
        HAL_UART_Transmit(&hlpuart1, (uint8_t*)"Erreur création CSV\r\n", 22, HAL_MAX_DELAY);
    }
}


