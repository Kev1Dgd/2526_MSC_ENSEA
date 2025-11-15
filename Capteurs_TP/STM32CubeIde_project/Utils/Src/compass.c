/*
 * compass.c
 *
 *  Created on: Nov 15, 2025
 *      Author: kevin
 */

#include "compass.h"
#include <math.h>
#include <stdio.h>

/**
 * @brief Calibre un accéléromètre avec une méthode simple single-point.
 *
 * Cette fonction utilise une calibration basique :
 * - X et Y supposés à 0 g
 * - Z supposé à 1 g
 * Elle calcule uniquement l'offset et applique un gain unitaire.
 *
 * @param acc Tableau de 3 doubles contenant les mesures brutes de l'accéléromètre (X, Y, Z)
 * @param calib Pointeur vers une structure CompassCalib_t où seront stockés les offsets et gains
 */
void CompassCalibSinglePoint(const double acc[3], CompassCalib_t *calib)
{
    calib->offset[0] = acc[0]; // X
    calib->offset[1] = acc[1]; // Y
    calib->offset[2] = acc[2] - 1.0; // Z en g

    calib->gain[0] = 1.0;
    calib->gain[1] = 1.0;
    calib->gain[2] = 1.0;
}

/**
 * @brief Calcule et formate les angles d'inclinaison à partir des données de l'accéléromètre.
 *
 * Cette fonction applique une calibration sur les axes, puis calcule :
 * - theta : angle dans le plan XY (atan2(Y, X))
 * - psi   : angle dans le plan XZ (atan2(X, Z))
 * - phi   : angle de rotation par rapport à l'axe Z (atan2(sqrt(X²+Y²), Z))
 *
 * Les angles sont ensuite convertis en degrés et stockés dans une chaîne de caractères formatée.
 *
 * @param acc Tableau de 3 doubles contenant les mesures brutes de l'accéléromètre (X, Y, Z)
 * @param msg Chaîne de caractères où sera écrit le message formaté
 * @param msg_len Taille maximale de la chaîne msg
 * @param calib Pointeur vers la structure CompassCalib_t contenant les offsets et gains à appliquer
 */
void CompassMeasureMsg(const double acc[3], char *msg, size_t msg_len, CompassCalib_t *calib)
{
    double ax = (acc[0] - calib->offset[0]) * calib->gain[0];
    double ay = (acc[1] - calib->offset[1]) * calib->gain[1];
    double az = (acc[2] - calib->offset[2]) * calib->gain[2];

    // Calcul des angles (radians)
    double theta = atan2(ay, ax);                      // Plan (XY)
    double psi   = atan2(ax, az);                      // Plan (XZ)
    double phi   = atan2(sqrt(ax*ax + ay*ay), az);     // Rotation par rapport à Z

    // Conversion en degrés
    theta *= (180.0 / M_PI);
    psi   *= (180.0 / M_PI);
    phi   *= (180.0 / M_PI);

    // Affichage formaté
    snprintf(msg, msg_len,
             "Theta(XY)=%.2f deg | Psi(XZ)=%.2f deg | Phi(Z)=%.2f deg\r\n",
             theta, psi, phi);
}


/**
 * @brief  Calcule l'orientation complète compensée en inclinaison
 * @param  acc Calibré, tableau de 3 valeurs de l'accéléromètre [X,Y,Z] en g
 * @param  mag Calibré, tableau de 3 valeurs du magnétomètre [X,Y,Z] en uT
 * @param  roll Pointeur vers variable qui recevra le roulis (deg)
 * @param  pitch Pointeur vers variable qui recevra le tangage (deg)
 * @param  heading Pointeur vers variable qui recevra l'azimut nord magnétique (deg)
 * @retval None
 *
 * @note  - Roulis et tangage sont calculés à partir de l'accéléromètre
 *        - Azimut (heading) est compensé pour l'inclinaison
 *        - Toutes les valeurs sont renvoyées en degrés
 */
void TiltCompensatedCompass(const double acc[3], const double mag[3],
                            double *roll, double *pitch, double *heading)
{
    // --- Calcul roulis et tangage à partir de l'accéléromètre ---
    // Roulis φ = rotation autour de X
    *roll  = atan2(acc[1], acc[2]) * (180.0 / M_PI);

    // Tangage θ = rotation autour de Y
    *pitch = atan2(-acc[0], sqrt(acc[1]*acc[1] + acc[2]*acc[2])) * (180.0 / M_PI);

    // Conversion en radians pour les calculs de compensation
    double roll_rad  = *roll  * (M_PI / 180.0);
    double pitch_rad = *pitch * (M_PI / 180.0);

    // --- Compensation magnétomètre ---
    double Xh = mag[0]*cos(pitch_rad) + mag[2]*sin(pitch_rad);
    double Yh = mag[0]*sin(roll_rad)*sin(pitch_rad) + mag[1]*cos(roll_rad) - mag[2]*sin(roll_rad)*cos(pitch_rad);

    // Azimut compensé (nord magnétique)
    *heading = atan2(Yh, Xh) * (180.0 / M_PI);

    // Ajuste pour avoir un angle positif 0–360°
    if(*heading < 0) *heading += 360.0;
}
