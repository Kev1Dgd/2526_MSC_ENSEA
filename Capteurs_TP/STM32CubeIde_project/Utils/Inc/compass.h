/*
 * compass.h
 *
 *  Created on: Nov 15, 2025
 *      Author: kevin
 */

#ifndef COMPASS_H
#define COMPASS_H

#include <stddef.h>   // pour size_t
#include "main.h"     // pour types STM32

// Structure pour stocker les offsets et gains
typedef struct {
    double offset[3];
    double gain[3];
} CompassCalib_t;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void CompassMeasureMsg(const double acc[3], char *msg, size_t msg_len, CompassCalib_t *calib);
void CompassCalibSinglePoint(const double acc[3], CompassCalib_t *calib);
void TiltCompensatedCompass(const double acc[3], const double mag[3], double *roll, double *pitch, double *heading);

#endif /* COMPASS_H */
