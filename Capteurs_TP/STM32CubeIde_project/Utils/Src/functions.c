/**
 * @file functions.c
 * @brief Fonctions de gestion du capteur MPU-9250 (IMU 9 axes).
 * @author Kevin
 * @date Novembre 2025
 */

#include "functions.h"
#include "tools.h"
#include "stm32g4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <float.h>

/* === Déclarations externes === */
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef hlpuart1;
extern uint8_t message[100];
extern void Error_Handler(void);
extern void MagMeasureMsg(double *mag_data, char *buffer, size_t size);

// ==== Initialisation ==== \\


/**
 * @brief Lecture du registre WHO_AM_I du MPU-9250 pour vérifier sa présence.
 *
 * Lit le registre 0x75 du capteur (adresse 0x68) via le bus I2C.
 * Affiche la valeur lue sur le terminal et allume la LED rouge en cas d’erreur.
 *
 * @param none
 * @retval none
 */
void checkMPU9250_ID(void) {
	uint8_t who_am_i = 0;

	// Lecture du registre WHO_AM_I (adresse 0x75)
	if (HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x75, 1, &who_am_i, 1, HAL_MAX_DELAY) != HAL_OK) {
		Error_Handler();
	}


	memset(message, 0, sizeof(message));
	snprintf((char*)message, sizeof(message), "MPU9250 WHO_AM_I = 0x%02X\r\n", who_am_i);
	HAL_UART_Transmit(&hlpuart1, message, strlen((char*)message), HAL_MAX_DELAY);

	// Vérification de l'identifiant
	if (who_am_i != 0x71) {
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)"Erreur MPU-9250\r\n", 18, HAL_MAX_DELAY);
		Error_Handler();
	} else {
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)"MPU-9250 détecté avec succès.\r\n", 36, HAL_MAX_DELAY);
	}
}


/**
 * @brief Initialise le MPU-9250 et le magnétomètre AK8963.
 *
 * Cette fonction effectue les opérations suivantes :
 * 1. Reset du MPU-9250
 * 2. Sélection de l’horloge PLL interne
 * 3. Lecture et vérification du registre WHO_AM_I du MPU-9250
 * 4. Configuration du DLPF et du Sample Rate Divider pour ODR ≈ 3,91 Hz
 * 5. Activation du mode BYPASS I2C pour accéder au magnétomètre
 * 6. Lecture et vérification du registre WHO_AM_I du magnétomètre AK8963
 * 7. Configuration du magnétomètre : 16 bits, Continuous Measurement Mode 2 (100 Hz)
 *
 * @note L’ODR du MPU-9250 est fixée à sa plus faible valeur (≈3,91 Hz)
 *       en utilisant SMPLRT_DIV = 255 et DLPF_CFG = 6.
 */
void InitSensors(void) {
	uint8_t data;

	// === Reset du MPU-9250 ===
	data = 0x80; // bit7 = H_RESET
	HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x6B, 1, &data, 1, HAL_MAX_DELAY);
	HAL_Delay(100);

	// === Sélection horloge PLL ===
	data = 0x01; // PLL_X_CLK
	HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x6B, 1, &data, 1, HAL_MAX_DELAY);
	HAL_Delay(10);

	// === Configuration DLPF pour ODR faible ===
	data = 0x06;  // DLPF_CFG = 6 → accéléro et gyro à 1 kHz interne
	HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x1A, 1, &data, 1, HAL_MAX_DELAY);

	// === Sample Rate Divider pour 3,91 Hz ===
	data = 255;   // SMPLRT_DIV = 255 → f_s ≈ 1000 / (1+255) ≈ 3,91 Hz
	HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x19, 1, &data, 1, HAL_MAX_DELAY);

	// === Activation du BYPASS (connexion du magnétomètre AK8963) ===
	data = 0x02; // Bit1 = BYPASS_EN
	HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x37, 1, &data, 1, HAL_MAX_DELAY);
	HAL_Delay(10);

	// === Vérification WHO_AM_I du magnétomètre AK8963 ===
	uint8_t who_am_i_mag = 0;
	HAL_I2C_Mem_Read(&hi2c1, 0x0C << 1, 0x00, 1, &who_am_i_mag, 1, HAL_MAX_DELAY);

	char msg[100];
	snprintf(msg, sizeof(msg), "AK8963 WHO_AM_I = 0x%02X\r\n", who_am_i_mag);
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	if (who_am_i_mag != 0x48) {
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)"Erreur magnétomètre AK8963\r\n", 29, HAL_MAX_DELAY);
		Error_Handler();
	} else {
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)"Magnétomètre détecté avec succès.\r\n", 38, HAL_MAX_DELAY);
	}

	// === Configuration du magnétomètre : 16-bit output + mode continu 2 (100Hz) ===
	data = 0x16; // 0b00010110 : 16-bit (bit4=1) + mode2 (bits[3:0]=0110)
	HAL_I2C_Mem_Write(&hi2c1, 0x0C << 1, 0x0A, 1, &data, 1, HAL_MAX_DELAY);
}




// ==== Mesures ==== \\


// == Temperature == \\


/**
 * @brief Mesure la température et écrit le résultat dans un buffer
 *
 * @param buffer pointeur vers char[] pour recevoir le message
 * @param size taille du buffer
 * @return double température en °C
 */
double TempMeasureMsg(char *buffer, size_t size) {
	uint8_t data[2];
	int16_t raw;
	double temperature;

	if(HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x41, 1, data, 2, HAL_MAX_DELAY) != HAL_OK)
		Error_Handler();

	raw = (int16_t)((data[0]<<8)|data[1]);
	temperature = ((double)raw - TEMP_OFFSET)/333.87 + 21.0;

	int temp_int = (int)temperature;
	int temp_frac = (int)((temperature - temp_int)*100);
	if(temp_frac<0) temp_frac=-temp_frac;
	temp_int -= 5; // Mesure expérimentale

	snprintf(buffer, size, "\r\nTemperature = %d.%02d °C\r\n", temp_int, temp_frac);
	return temperature;
}


// == Acceleration == \\

/**
 * @brief Lit la sensibilité réelle de l'accéléromètre MPU-9250
 *
 * Lit le registre ACCEL_CONFIG et retourne le facteur de conversion
 * en LSB/g pour la plage configurée.
 *
 * @return double Sensibilité (LSB/g)
 */
double ReadAccelSensitivity(void) {
	uint8_t reg;
	double factor;

	// Lecture du registre ACCEL_CONFIG
	if (HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x1C, 1, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
		Error_Handler();

	// Bits [4:3] : FS_SEL
	reg = (reg >> 3) & 0x03;

	switch(reg) {
	case 0: factor = 16384.0; break; // ±2g
	case 1: factor = 8192.0;  break; // ±4g
	case 2: factor = 4096.0;  break; // ±8g
	case 3: factor = 2048.0;  break; // ±16g
	default: factor = 16384.0; break;
	}

	return factor;
}


/**
 * @brief Mesure l'accélération selon les 3 axes du MPU-9250
 *
 * Lit les registres ACCEL_XOUT_H/L (0x3B/0x3C), ACCEL_YOUT_H/L (0x3D/0x3E),
 * ACCEL_ZOUT_H/L (0x3F/0x40) via I2C et convertit les valeurs brutes en g.
 *
 * @details
 * Les valeurs brutes sont codées en complément à 2 (int16_t).
 * Pour l'étendue par défaut ±2g, la conversion est :
 * ax = ACCEL_X_RAW / 16384.0
 * ay = ACCEL_Y_RAW / 16384.0
 * az = ACCEL_Z_RAW / 16384.0
 *
 * La fonction affiche également les valeurs mesurées sur le terminal UART.
 *
 * @param acc_data Pointeur vers un tableau de 3 doubles
 *                 pour stocker [ax, ay, az] en g.
 */
void AccMeasureMsg(double *acc_data, char *buffer, size_t size) {
	uint8_t raw[6];
	int16_t ax_raw, ay_raw, az_raw;
	double sensitivity = ReadAccelSensitivity();

	if(HAL_I2C_Mem_Read(&hi2c1, 0x68<<1, 0x3B, 1, raw, 6, HAL_MAX_DELAY) != HAL_OK)
		Error_Handler();

	ax_raw = (int16_t)((raw[0]<<8)|raw[1]);
	ay_raw = (int16_t)((raw[2]<<8)|raw[3]);
	az_raw = (int16_t)((raw[4]<<8)|raw[5]);

	acc_data[0] = (double)ax_raw / sensitivity;
	acc_data[1] = (double)ay_raw / sensitivity;
	acc_data[2] = (double)az_raw / sensitivity;

	double norm = sqrt(acc_data[0]*acc_data[0] + acc_data[1]*acc_data[1] + acc_data[2]*acc_data[2]);
	snprintf(buffer, size, "Accel - X: %.3f g, Y: %.3f g, Z: %.3f g\r\n||A|| = %.3f g\r\n",
			acc_data[0], acc_data[1], acc_data[2], norm);
}


/**
 * @brief Teste l'accéléromètre sur toutes les plages de sensibilité et affiche Z
 *
 * Affiche : FS_SEL, plage (g), sensibilité (LSB/g), Z_raw et Z_g
 */
void TestAccelSensZ(void) {
	uint8_t reg;
	int16_t z_raw;
	double z_g;
	double sensitivity;
	double z_range;
	char msg[150];

	HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\r\n*** Test de l'accéléromètre sur toutes les plages FS_SEL (Z) "
			"***\r\n", 72, HAL_MAX_DELAY);

	for (uint8_t fs_sel = 0; fs_sel <= 3; fs_sel++) {
		// Lire le registre ACCEL_CONFIG
		if (HAL_I2C_Mem_Read(&hi2c1, 0x68<<1, 0x1C, 1, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
			Error_Handler();

		// Modification FS_SEL (bits 4:3)
		reg &= ~(0x18);           // Clear bits 4:3
		reg |= (fs_sel << 3);     // Set FS_SEL
		if (HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, 0x1C, 1, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
			Error_Handler();

		HAL_Delay(100); // Stabilisation

		// Lire la valeur brute de l'axe Z
		uint8_t data[2];
		if (HAL_I2C_Mem_Read(&hi2c1, 0x68<<1, 0x3F, 1, data, 2, HAL_MAX_DELAY) != HAL_OK)
			Error_Handler();
		z_raw = (int16_t)((data[0]<<8)|data[1]);

		switch(fs_sel) {
		case 0: sensitivity = 16384.0; z_range = 2.0; break;  // ±2g
		case 1: sensitivity = 8192.0;  z_range = 4.0; break;  // ±4g
		case 2: sensitivity = 4096.0;  z_range = 8.0; break;  // ±8g
		case 3: sensitivity = 2048.0;  z_range = 16.0; break; // ±16g
		default: sensitivity = 16384.0; z_range = 2.0; break;
		}

		z_g = (double)z_raw / sensitivity;

		// Affichage UART
		snprintf(msg, sizeof(msg),
				"FS_SEL=%d | Plage=±%.1f g | Sens=%g LSB/g | Z_raw=%d | Z_g=%.3f g\r\n",
				fs_sel, z_range, sensitivity, z_raw, z_g);
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	}

	// Ligne vide après
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
}


// == Gyroscope == \\


/**
 * @brief Lit la sensibilité réelle du gyroscope MPU-9250
 *
 * @return double Sensibilité (LSB/(°/s)) selon la plage configurée
 */
double ReadGyroSensitivity(void) {
	uint8_t reg;
	double factor;

	if(HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x1B, 1, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
		Error_Handler();

	reg = (reg >> 3) & 0x03; // Bits FS_SEL
	switch(reg) {
	case 0: factor = 131.0; break;    // ±250 °/s
	case 1: factor = 65.5;  break;    // ±500 °/s
	case 2: factor = 32.8;  break;    // ±1000 °/s
	case 3: factor = 16.4;  break;    // ±2000 °/s
	default: factor = 131.0; break;
	}
	return factor;
}


/**
 * @brief Mesure la vitesse angulaire sur les 3 axes du MPU-9250
 *
 * Lit les registres GYRO_XOUT_H/L, GYRO_YOUT_H/L, GYRO_ZOUT_H/L
 * et convertit les valeurs brutes en °/s.
 *
 * @param gyro_data Pointeur vers un tableau de 3 doubles [gx, gy, gz] en °/s
 */
void GyroMeasureMsg(double *gyro_data, char *buffer, size_t size) {
	uint8_t raw[6];
	int16_t gx_raw, gy_raw, gz_raw;
	double sensitivity = ReadGyroSensitivity();

	if(HAL_I2C_Mem_Read(&hi2c1, 0x68<<1, 0x43, 1, raw, 6, HAL_MAX_DELAY) != HAL_OK)
		Error_Handler();

	gx_raw = (int16_t)((raw[0]<<8)|raw[1]);
	gy_raw = (int16_t)((raw[2]<<8)|raw[3]);
	gz_raw = (int16_t)((raw[4]<<8)|raw[5]);

	gyro_data[0] = (double)gx_raw / sensitivity;
	gyro_data[1] = (double)gy_raw / sensitivity;
	gyro_data[2] = (double)gz_raw / sensitivity;

	snprintf(buffer, size, "Gyro - X: %.2f °/s, Y: %.2f °/s, Z: %.2f °/s\r\n",
			gyro_data[0], gyro_data[1], gyro_data[2]);
}


/**
 * @brief Calcule le bruit RMS du gyroscope sur les 3 axes.
 *
 * Lit nb_samples valeurs du gyroscope, convertit en °/s et calcule le RMS.
 *
 * @param[out] gyro_rms Tableau de 3 doubles [gx, gy, gz] pour stocker les valeurs RMS.
 * @param[in] nb_samples Nombre d'échantillons à lire.
 */
void GyrNoise(double *gyro_rms, uint16_t nb_samples) {
	uint8_t raw[6];
	int16_t gx_raw, gy_raw, gz_raw;
	double gx_sum2 = 0.0, gy_sum2 = 0.0, gz_sum2 = 0.0;
	double sensitivity = 131.0; // LSB/(°/s) par défaut, ajuster selon FS_SEL
	double gx, gy, gz;

	for(uint16_t i=0; i<nb_samples; i++) {
		// Lire les registres GYRO_XOUT_H (0x43) à GYRO_ZOUT_L (0x48)
		if(HAL_I2C_Mem_Read(&hi2c1, 0x68<<1, 0x43, 1, raw, 6, HAL_MAX_DELAY) != HAL_OK)
			Error_Handler();

		gx_raw = (int16_t)((raw[0]<<8) | raw[1]);
		gy_raw = (int16_t)((raw[2]<<8) | raw[3]);
		gz_raw = (int16_t)((raw[4]<<8) | raw[5]);

		// Conversion en °/s
		gx = (double)gx_raw / sensitivity;
		gy = (double)gy_raw / sensitivity;
		gz = (double)gz_raw / sensitivity;

		// Somme des carrés pour RMS
		gx_sum2 += gx*gx;
		gy_sum2 += gy*gy;
		gz_sum2 += gz*gz;

		HAL_Delay(1); // Petit délai pour espacer les échantillons si nécessaire
	}

	// Calcul RMS
	gyro_rms[0] = sqrt(gx_sum2 / nb_samples);
	gyro_rms[1] = sqrt(gy_sum2 / nb_samples);
	gyro_rms[2] = sqrt(gz_sum2 / nb_samples);
}


/**
 * @brief Teste le bruit RMS du gyroscope pour différentes bandes passantes
 *
 * @param nb_samples Nombre de mesures pour calculer le RMS
 */
void TestGyroNoise(uint16_t nb_samples) {
	uint16_t bw_values[] = {3600, 250, 100, 20, 5};
	double gyro_noise[3];
	char msg[100];

	HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\r\n*** Test du bruit du gyroscope sur différentes bandes passantes ***\r\n",
			76, HAL_MAX_DELAY);

	for(int i=0; i<5; i++) {
		// Affiche ce que l'on va faire
		snprintf(msg, sizeof(msg), "Configuration bande passante Gyro: %d Hz\r\n", bw_values[i]);
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

		// Configurer la bande passante du gyroscope
		SetGyroBandwidth(bw_values[i]);

		// Mesurer le bruit RMS
		GyrNoise(gyro_noise, nb_samples);

		// Affiche les valeurs RMS
		snprintf(msg, sizeof(msg),
				"BW=%d Hz | RMS Gyro: gx=%.3f °/s, gy=%.3f °/s, gz=%.3f °/s\r\n",
				bw_values[i], gyro_noise[0], gyro_noise[1], gyro_noise[2]);
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	}
}


void SetGyroBandwidth(uint8_t bw_code) {
	uint8_t reg = bw_code & 0x07; // Exemple, selon MPU-9250
	HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x1A, 1, &reg, 1, HAL_MAX_DELAY);
}


// == Champ magnétique == \\


/**
 * @brief Lit la sensibilité nominale du magnétomètre AK8963 selon le mode de sortie.
 *
 * @param bit16_mode Si 1, mode 16 bits (0.15 µT/LSB). Si 0, mode 14 bits (0.6 µT/LSB).
 * @return double Sensibilité du magnétomètre en µT/LSB
 */
double ReadMagSensitivity(uint8_t bit16_mode) {
	double factor;

	if(bit16_mode) {
		factor = 0.15; // µT/LSB en 16 bits
	} else {
		factor = 0.6;  // µT/LSB en 14 bits
	}

	return factor;
}


/**
 * @brief Mesure le champ magnétique 3D via le magnétomètre AK8963.
 *
 * Lit les registres HXL à HZH (0x03 à 0x08), applique les coefficients
 * d’ajustement (ASAX, ASAY, ASAZ), puis convertit les valeurs brutes en µT.
 *
 * @param mag_data Pointeur vers un tableau de 3 doubles [Bx, By, Bz] (µT)
 * @param buffer   Pointeur vers la chaîne de caractères à remplir
 * @param size     Taille du buffer
 */
void MagMeasureMsg(double *mag_data, char *buffer, size_t size) {
	uint8_t raw[7];
	int16_t mx_raw, my_raw, mz_raw;
	static double mag_adj[3] = {0.0, 0.0, 0.0};  // facteurs d’ajustement
	static uint8_t initialized = 0;
	double sens;  // sensibilité en µT/LSB

	// 1. Initialisation + Lecture ASAX, ASAY, ASAZ
	if (!initialized) {
		uint8_t asa[3];
		uint8_t mode = 0x0F;
		if (HAL_I2C_Mem_Write(&hi2c1, 0x0C << 1, 0x0A, 1, &mode, 1, HAL_MAX_DELAY) != HAL_OK)
			Error_Handler();
		HAL_Delay(10);

		if (HAL_I2C_Mem_Read(&hi2c1, 0x0C << 1, 0x10, 1, asa, 3, HAL_MAX_DELAY) != HAL_OK)
			Error_Handler();

		// 2. Passer en Continuous Measurement Mode 2 (16 bits, 100Hz)
		mode = 0x16;
		if (HAL_I2C_Mem_Write(&hi2c1, 0x0C << 1, 0x0A, 1, &mode, 1, HAL_MAX_DELAY) != HAL_OK)
			Error_Handler();
		HAL_Delay(10);

		// 3. Calcul des coefficients d’ajustement
		for (int i = 0; i < 3; i++) {
			mag_adj[i] = ((double)(asa[i] - 128) / 256.0) + 1.0;
		}

		sens = ReadMagSensitivity(1);  // 16 bits = 0.15 µT/LSB
		initialized = 1;

		char msg[100];
		snprintf(msg, sizeof(msg),
				"AK8963 Sensitivity Adj: X=%.3f, Y=%.3f, Z=%.3f, sens=%.3f µT/LSB\r\n",
				mag_adj[0], mag_adj[1], mag_adj[2], sens);
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	} else {
		sens = ReadMagSensitivity(1);
	}

	// 4. Attendre de nouvelles données : ST1.DRDY
	uint8_t st1;
	do {
		if(HAL_I2C_Mem_Read(&hi2c1, 0x0C << 1, 0x02, 1, &st1, 1, HAL_MAX_DELAY) != HAL_OK)
			Error_Handler();
	} while(!(st1 & 0x01));  // DRDY = 1

	// 5. Lecture des registres de data
	if(HAL_I2C_Mem_Read(&hi2c1, 0x0C << 1, 0x03, 1, raw, 7, HAL_MAX_DELAY) != HAL_OK)
		Error_Handler();

	// 6. Vérifier l’overflow
	uint8_t st2;
	if(HAL_I2C_Mem_Read(&hi2c1, 0x0C << 1, 0x09, 1, &st2, 1, HAL_MAX_DELAY) != HAL_OK)
		Error_Handler();
	if(st2 & 0x08) return; // données saturées :ignorer lecture

	// 7. Assemblage des données brutes
	mx_raw = (int16_t)((raw[1] << 8) | raw[0]);
	my_raw = (int16_t)((raw[3] << 8) | raw[2]);
	mz_raw = (int16_t)((raw[5] << 8) | raw[4]);

	// 8. Conversion en µT : sensibilité + coefficients d’ajustement
	mag_data[0] = (double)mx_raw * sens * mag_adj[0];
	mag_data[1] = (double)my_raw * sens * mag_adj[1];
	mag_data[2] = (double)mz_raw * sens * mag_adj[2];

	// 9. Calcul de la norme du vecteur B
	double norm = sqrt(mag_data[0]*mag_data[0] + mag_data[1]*mag_data[1] + mag_data[2]*mag_data[2]);

	// 10. Écriture du message
	snprintf(buffer, size,
			"Mag - X: %.2f µT, Y: %.2f µT, Z: %.2f µT\r\n||B|| = %.2f µT\r\n",
			mag_data[0], mag_data[1], mag_data[2], norm);
}




// ===== Fonctions Mathématiques ==== \\


/**
 * @brief Calcule la moyenne de N mesures gyroscopiques.
 * @param data Tableau contenant les mesures (X, Y, Z)
 * @param N Nombre d’échantillons
 * @param avg Tableau de sortie [3] pour stocker les moyennes
 */
void Average(float data[][3], int N, float avg[3]) {
	avg[0] = avg[1] = avg[2] = 0.0f;
	for (int i = 0; i < N; i++) {
		avg[0] += data[i][0];
		avg[1] += data[i][1];
		avg[2] += data[i][2];
	}
	avg[0] /= N;
	avg[1] /= N;
	avg[2] /= N;
}


/**
 * @brief Calcule la variance des N mesures gyroscopiques.
 * @param data Tableau [N][3]
 * @param N Nombre d’échantillons
 * @param avg Moyennes calculées pour chaque axe
 * @param var Tableau de sortie [3] pour stocker les variances
 */
void Variance(float data[][3], int N, float avg[3], float var[3]) {
	var[0] = var[1] = var[2] = 0.0f;
	for (int i = 0; i < N; i++) {
		var[0] += powf(data[i][0] - avg[0], 2);
		var[1] += powf(data[i][1] - avg[1], 2);
		var[2] += powf(data[i][2] - avg[2], 2);
	}
	var[0] /= N;
	var[1] /= N;
	var[2] /= N;
}




// ==== Calibration ==== \\


// == Gyroscope == \\

/**
 * @brief Lit les vitesses angulaires brutes (°/s) sur les 3 axes.
 *
 * @param gx Pointeur vers la valeur X
 * @param gy Pointeur vers la valeur Y
 * @param gz Pointeur vers la valeur Z
 */
void ReadGyro(float *gx, float *gy, float *gz) {
	uint8_t raw[6];
	int16_t gx_raw, gy_raw, gz_raw;
	double sensitivity = ReadGyroSensitivity();

	if (HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x43, 1, raw, 6, HAL_MAX_DELAY) != HAL_OK)
		Error_Handler();

	gx_raw = (int16_t)((raw[0] << 8) | raw[1]);
	gy_raw = (int16_t)((raw[2] << 8) | raw[3]);
	gz_raw = (int16_t)((raw[4] << 8) | raw[5]);

	*gx = (float)(gx_raw / sensitivity);
	*gy = (float)(gy_raw / sensitivity);
	*gz = (float)(gz_raw / sensitivity);
}


/**
 * @brief Calibre le gyroscope en mode "single point" (no turn).
 *
 * Cette fonction mesure N échantillons lorsque le capteur est immobile,
 * calcule les offsets pour chaque axe et les écrit dans les registres
 * matériels de compensation du MPU-9250.
 *
 * @param N Nombre d’échantillons utilisés pour la moyenne
 */
void GyrCalib(uint16_t N) {
	float gyro_data[N][3];
	float avg[3];

	// --- Acquisition des N mesures ---
	for (int i = 0; i < N; i++) {
		ReadGyro(&gyro_data[i][0], &gyro_data[i][1], &gyro_data[i][2]);
		HAL_Delay(10); // selon ODR
	}

	// --- Calcul des offsets moyens ---
	Average(gyro_data, N, avg);

	char msg[100];
	snprintf(msg, sizeof(msg), "Offsets mesurés : X=%.3f, Y=%.3f, Z=%.3f\r\n", avg[0], avg[1], avg[2]);
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	// --- Conversion en valeurs brutes ---
	int16_t off_x = (int16_t)(-avg[0]);
	int16_t off_y = (int16_t)(-avg[1]);
	int16_t off_z = (int16_t)(-avg[2]);

	uint8_t data[2];

	// --- Écriture dans les registres matériels ---
	data[0] = (off_x >> 8) & 0xFF;
	data[1] = off_x & 0xFF;
	HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x13, 1, data, 2, HAL_MAX_DELAY);

	data[0] = (off_y >> 8) & 0xFF;
	data[1] = off_y & 0xFF;
	HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x15, 1, data, 2, HAL_MAX_DELAY);

	data[0] = (off_z >> 8) & 0xFF;
	data[1] = off_z & 0xFF;
	HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x17, 1, data, 2, HAL_MAX_DELAY);

	HAL_UART_Transmit(&hlpuart1, (uint8_t*)"Calibration gyroscope terminée.\r\n", 34, HAL_MAX_DELAY);
}


/**
 * @brief  Lit les offsets matériels du gyroscope dans le MPU-9250 et les affiche.
 * @note   Permet de vérifier si les offsets écrits sont bien proches de 0 après calibration.
 * @retval None
 */
void ReadOffsetGyro(void){
	uint8_t data[6];
	int16_t gyro_offset_raw[3];
	float gyro_offset_dps[3];

	// Lire les 6 registres d'offset (X, Y, Z)
	if (HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, 0x13, 1, data, 6, HAL_MAX_DELAY) != HAL_OK){
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)"Erreur lecture offsets gyroscope\r\n", 35, HAL_MAX_DELAY);
		return;
	}

	// Conversion en valeurs signées
	gyro_offset_raw[0] = (int16_t)((data[0] << 8) | data[1]);
	gyro_offset_raw[1] = (int16_t)((data[2] << 8) | data[3]);
	gyro_offset_raw[2] = (int16_t)((data[4] << 8) | data[5]);

	// Conversion en °/s (selon sensibilité ±250°/s)
	gyro_offset_dps[0] = (float)gyro_offset_raw[0] / 131.0f;
	gyro_offset_dps[1] = (float)gyro_offset_raw[1] / 131.0f;
	gyro_offset_dps[2] = (float)gyro_offset_raw[2] / 131.0f;

	// Affichage des résultats
	char msg[128];
	snprintf(msg, sizeof(msg),"Offsets Gyro (°/s): X=%.3f | Y=%.3f | Z=%.3f\r\n",
			gyro_offset_dps[0], gyro_offset_dps[1], gyro_offset_dps[2]);
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	// Vérification : proche de 0 ?
	if (fabs(gyro_offset_dps[0]) < 0.2 && fabs(gyro_offset_dps[1]) < 0.2 && fabs(gyro_offset_dps[2]) < 0.2)
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)"Offsets gyroscope proches de 0\r\n", 36, HAL_MAX_DELAY);
	else
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)"Offsets gyroscope NON nuls\r\n", 33, HAL_MAX_DELAY);
}


// == Magnétomètre == \\


/**
 * @brief Lit le champ magnétique 3D directement depuis le magnétomètre AK8963
 *        sans afficher de message UART.
 *
 * @param mx Pointeur vers float pour stocker l'axe X (µT)
 * @param my Pointeur vers float pour stocker l'axe Y (µT)
 * @param mz Pointeur vers float pour stocker l'axe Z (µT)
 */
void ReadMag(float *mx, float *my, float *mz) {
	uint8_t raw[7];
	int16_t mx_raw, my_raw, mz_raw;
	double sens = 0.15; // Sensibilité 16 bits en µT/LSB (mode 16 bits)
	static double mag_adj[3] = {1.0, 1.0, 1.0};
	static uint8_t initialized = 0;

	// Initialisation une seule fois (lecture ASAX/ASAY/ASAZ)
	if (!initialized) {
		uint8_t asa[3];
		uint8_t mode = 0x0F; // Fuse ROM access mode
		if (HAL_I2C_Mem_Write(&hi2c1, 0x0C << 1, 0x0A, 1, &mode, 1, HAL_MAX_DELAY) != HAL_OK)
			Error_Handler();
		HAL_Delay(10);

		if (HAL_I2C_Mem_Read(&hi2c1, 0x0C << 1, 0x10, 1, asa, 3, HAL_MAX_DELAY) != HAL_OK)
			Error_Handler();

		for (int i = 0; i < 3; i++) {
			mag_adj[i] = ((double)(asa[i] - 128) / 256.0) + 1.0;
		}

		mode = 0x16; // Continuous measurement mode 2 (16 bits, 100 Hz)
		if (HAL_I2C_Mem_Write(&hi2c1, 0x0C << 1, 0x0A, 1, &mode, 1, HAL_MAX_DELAY) != HAL_OK)
			Error_Handler();
		HAL_Delay(10);

		initialized = 1;
	}

	// Attendre que les données soient prêtes (ST1.DRDY)
	/*uint8_t st1;
	do {
		if (HAL_I2C_Mem_Read(&hi2c1, 0x0C << 1, 0x02, 1, &st1, 1, HAL_MAX_DELAY) != HAL_OK)
			Error_Handler();
	} while (!(st1 & 0x01));*/

	// Lecture des registres HXL à HZH
	if (HAL_I2C_Mem_Read(&hi2c1, 0x0C << 1, 0x03, 1, raw, 7, HAL_MAX_DELAY) != HAL_OK)
		Error_Handler();

	// Vérification overflow (ST2)
	uint8_t st2;
	if (HAL_I2C_Mem_Read(&hi2c1, 0x0C << 1, 0x09, 1, &st2, 1, HAL_MAX_DELAY) != HAL_OK)
		Error_Handler();
	if (st2 & 0x08) { // overflow, ignorer lecture
		*mx = *my = *mz = 0.0f;
		return;
	}

	// Conversion des données brutes
	mx_raw = (int16_t)((raw[1] << 8) | raw[0]);
	my_raw = (int16_t)((raw[3] << 8) | raw[2]);
	mz_raw = (int16_t)((raw[5] << 8) | raw[4]);

	*mx = (float)(mx_raw * sens * mag_adj[0]);
	*my = (float)(my_raw * sens * mag_adj[1]);
	*mz = (float)(mz_raw * sens * mag_adj[2]);
}


/**
 * @brief Lit N mesures du magnétomètre et les stocke dans les tableaux Bx, By, Bz
 *
 * @param Bx Tableau de float pour stocker les mesures X
 * @param By Tableau de float pour stocker les mesures Y
 * @param Bz Tableau de float pour stocker les mesures Z
 * @param N Nombre de mesures à effectuer
 * @param delay_ms Délai entre chaque mesure en millisecondes
 */
void MagAcquireSamples(float *Bx, float *By, float *Bz, uint16_t N, uint16_t delay_ms)
{
	for(uint16_t i=0; i<N; i++)
	{
		float mag[3];
		ReadMag(&mag[0], &mag[1], &mag[2]);

		Bx[i] = (float)mag[0];
		By[i] = (float)mag[1];
		Bz[i] = (float)mag[2];

		HAL_Delay(delay_ms);
	}
}

/**
 * @brief Calcule les offsets moyens (hard iron) à partir des mesures
 *
 * @param Bx Tableau des mesures X
 * @param By Tableau des mesures Y
 * @param Bz Tableau des mesures Z
 * @param N Nombre de mesures
 * @param offsetX Pointeur vers float pour stocker l'offset X moyen
 * @param offsetY Pointeur vers float pour stocker l'offset Y moyen
 * @param offsetZ Pointeur vers float pour stocker l'offset Z moyen
 */
void MagComputeOffsets(float *Bx, float *By, float *Bz, uint16_t N, float *offsetX, float *offsetY, float *offsetZ)
{
	float sumX=0, sumY=0, sumZ=0;
	for(uint16_t i=0; i<N; i++)
	{
		sumX += Bx[i];
		sumY += By[i];
		sumZ += Bz[i];
	}
	*offsetX = sumX / N;
	*offsetY = sumY / N;
	*offsetZ = sumZ / N;
}

/**
 * @brief Affiche les offsets via UART
 *
 * @param offsetX Offset moyen sur l'axe X
 * @param offsetY Offset moyen sur l'axe Y
 * @param offsetZ Offset moyen sur l'axe Z
 */
void MagPrintOffsets(float offsetX, float offsetY, float offsetZ)
{
	char msg[100];
	snprintf(msg, sizeof(msg), "\r\nOffsets Mag (µT): X=%.2f | Y=%.2f | Z=%.2f \r\n",
			offsetX, offsetY, offsetZ);
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

/**
 * @brief Fonction complète pour la calibration « light » du magnétomètre :
 *        acquisition des mesures, calcul des offsets et affichage.
 *
 * @param N Nombre de mesures à acquérir
 * @param delay_ms Délai entre chaque mesure en millisecondes
 */
void MagCalibLite(uint16_t N, uint16_t delay_ms)
{
	float Bx[N], By[N], Bz[N];
	float offsetX, offsetY, offsetZ;

	MagAcquireSamples(Bx, By, Bz, N, delay_ms);
	MagComputeOffsets(Bx, By, Bz, N, &offsetX, &offsetY, &offsetZ);
	MagPrintOffsets(offsetX, offsetY, offsetZ);
}


/**
 * @brief  Calibre le magnétomètre pour compenser les hard iron et soft iron biases.
 * @param  hi2c         Pointeur vers la structure I2C (ex: &hi2c1)
 * @param  mag_bias     Tableau de 3 doubles pour stocker les offsets (hard iron) calculés
 * @param  mag_scale    Tableau de 3 doubles pour stocker les coefficients correcteurs (soft iron) calculés
 * @retval None
 *
 * @note   Utilise ReadMag() pour ne pas afficher de messages UART inutiles.
 */
void MagCalib(I2C_HandleTypeDef *hi2c, double mag_bias[3], double mag_scale[3]){


	double Bx_min =  DBL_MAX, Bx_max = -DBL_MAX;
	double By_min =  DBL_MAX, By_max = -DBL_MAX;
	double Bz_min =  DBL_MAX, Bz_max = -DBL_MAX;

	for(uint16_t i = 0; i < CALIB_MAG_SAMPLES; i++)
	{
		float mx, my, mz;
		ReadMag(&mx, &my, &mz);  // lecture sans UART

		if(mx < Bx_min) Bx_min = mx;
		if(mx > Bx_max) Bx_max = mx;

		if(my < By_min) By_min = my;
		if(my > By_max) By_max = my;

		if(mz < Bz_min) Bz_min = mz;
		if(mz > Bz_max) Bz_max = mz;

		HAL_Delay(CALIB_MAG_DELAY_MS);
	}

	// offsets / bias (hard iron)
	mag_bias[0] = (Bx_max + Bx_min) / 2.0;
	mag_bias[1] = (By_max + By_min) / 2.0;
	mag_bias[2] = (Bz_max + Bz_min) / 2.0;

	// rayons et coefficients correcteurs (soft iron simplifié)
	double R_x = (Bx_max - Bx_min) / 2.0;
	double R_y = (By_max - By_min) / 2.0;
	double R_z = (Bz_max - Bz_min) / 2.0;
	double R_mean = (R_x + R_y + R_z) / 3.0;

	mag_scale[0] = R_mean / R_x;
	mag_scale[1] = R_mean / R_y;
	mag_scale[2] = R_mean / R_z;
}


/**
 * @brief Mesure le champ magnétique corrigé par calibration (hard + soft iron)
 *
 * Cette fonction applique les offsets (hard iron) et les coefficients de
 * correction (soft iron) calculés lors de MagCalib(). Puis elle affiche dans la commande
 * les valeurs brutes et celles calibrées.
 *
 * @param mag_data Pointeur vers un tableau de 3 doubles [Bx, By, Bz] (µT)
 * @param buffer   Pointeur vers la chaîne de caractères à remplir (UART)
 * @param size     Taille du buffer
 * @param mag_bias Tableau [3] des offsets calculés
 * @param mag_scale Tableau [3] des coefficients correcteurs
 */
void MagMeasureMsgCalib(double *mag_data, char *buffer, size_t size,
		const double mag_bias[3], const double mag_scale[3]){
	double mx, my, mz;

	// 1. Lire la mesure brute
	float mx_f, my_f, mz_f;
	ReadMag(&mx_f, &my_f, &mz_f);

	mx = (double)mx_f;
	my = (double)my_f;
	mz = (double)mz_f;

	// 2. Correction hard iron (offset)
	mx -= mag_bias[0];
	my -= mag_bias[1];
	mz -= mag_bias[2];

	// 3. Correction soft iron (scale)
	mx *= mag_scale[0];
	my *= mag_scale[1];
	mz *= mag_scale[2];

	// 4. Remplir le tableau de sortie
	mag_data[0] = mx;
	mag_data[1] = my;
	mag_data[2] = mz;

	// 5. Calcul de la norme
	double norm = sqrt(mx*mx + my*my + mz*mz);

	// 6. Affichage UART
	if(buffer && size > 0)
	{
		memset(buffer, 0, size);
		snprintf(buffer, size,
				"Calib Mag (µT): X=%.2f | Y=%.2f | Z=%.2f \r\n||B|| = %.2f µT\r\n",
				mx, my, mz, norm);
		// Affichage pour MATLAB
		/*snprintf(buffer, size,
				"\r\n%.2f, %.2f, %.2f",
				mx, my, mz);*/
	}
}




