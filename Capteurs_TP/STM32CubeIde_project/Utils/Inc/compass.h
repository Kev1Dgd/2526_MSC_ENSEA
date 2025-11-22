/**
 * @file compass.h
 * @brief En-têtes des fonctions pour la boussole électronique
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

#define PI 3.14

/* Déclarations des paramètres (définis dans compass.c) */
extern double Te;    // Période d'échantillonnage
extern double tau;   // Constante de temps
extern double K;     // Coefficient complémentaire Te)

/* Fonctions */
void CompassMeasureMsg(const double acc[3], char *msg, size_t msg_len, CompassCalib_t *calib);
void CompassCalibSinglePoint(const double acc[3], CompassCalib_t *calib);

/* Calculs de la boussole / inclinaison */
void TiltCompensatedCompass(const double acc[3], const double mag[3], double *roll, double *pitch, double *heading);

/*
 * Filtre complémentaire réel : fournit roll/pitch en utilisant acc + gyro
 * - gyro_rates : tableau [gx, gy, gz] en deg/s (ou rad/s si tes lectures le sont — adapte si besoin)
 * - acc : accéléromètre calibré en g
 * - mag : magnétomètre calibré en µT (peut être NULL si on ne calcule pas heading ici)
 * Résultat : roll, pitch, heading (deg). Heading calculé via mag + compensation tilt.
 */
void TiltCompensatedComplementary(const double acc[3], const double gyro_rates[3], const double mag[3],
                                  double *roll, double *pitch, double *heading);

/**
 * @brief  Ancienne fonction TiltCompensatedCompassAngles.
 *         Conservée comme fallback : applique une lissage simple sur les angles
 *         calculés uniquement à partir de l'accéléromètre (sans gyro).
 *
 *         Pour un vrai filtre complémentaire (acc+gyro) utilise la fonction
 *         TiltCompensatedComplementary(...) ci-dessus.
 */
void TiltCompensatedCompassAngles(const double acc[3], const double mag[3], double *roll, double *pitch, double *heading);

#endif /* COMPASS_H */

