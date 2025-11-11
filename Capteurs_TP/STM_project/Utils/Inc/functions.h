/**
 * @file functions.h
 * @brief En-têtes des fonctions de gestion du MPU-9250.
 */

#ifndef INC_FUNCTIONS_H_
#define INC_FUNCTIONS_H_

#include "stm32g4xx_hal.h"
#include <stdint.h>

void checkMPU9250_ID(void);
void InitSensors(void);

double TempMeasureMsg(char *buffer, size_t size);
void AccMeasureMsg(double *acc_data, char *buffer, size_t size);
void GyroMeasureMsg(double *gyro_data, char *buffer, size_t size);
void MagMeasureMsg(double *mag_data, char *buffer, size_t size);

double ReadAccelSensitivity(void);
double ReadGyroSensitivity(void);
double ReadMagSensitivity(uint8_t bit16_mode);

void TestAccelSensZ(void);
void GyrNoise(double *gyro_rms, uint16_t nb_samples);
void TestGyroNoise(uint16_t nb_samples);
void SetGyroBandwidth(uint8_t bw_code);

void Average(float data[][3], int N, float avg[3]);
void Variance(float data[][3], int N, float avg[3], float var[3]);
void GyrCalib(uint16_t N);
void ReadGyro(float *gx, float *gy, float *gz);
void ReadOffsetGyro(void);

void ReadMag(float *mx, float *my, float *mz);
void MagAcquireSamples(float *Bx, float *By, float *Bz, uint16_t N, uint16_t delay_ms);
void MagComputeOffsets(float *Bx, float *By, float *Bz, uint16_t N, float *offsetX, float *offsetY, float *offsetZ);
void MagPrintOffsets(float offsetX, float offsetY, float offsetZ);
void MagCalibLite(uint16_t N, uint16_t delay_ms);
void MagCalib(I2C_HandleTypeDef *hi2c, double mag_bias[3], double mag_scale[3]);
void MagMeasureMsgCalib(double *mag_data, char *buffer, size_t size, const double mag_bias[3], const double mag_scale[3]);



#endif /* INC_FUNCTIONS_H_ */
