/**
 * @file compass.c
 * @brief Création d'une boussole électronique (mono et multi capteurs)
 * @author Kevin
 * @date Novembre 2025
 */

#include "compass.h"
#include <math.h>
#include <stdio.h>

/* --- paramètres (définis ici) --- */
double Te = 0.01;   // période d'échantillonnage (s)
double tau = 0.1;   // constante de temps (s)
double K = 0.0;     // sera initialisé dans CompassInitParameters()

/* valeur PI plus précise */
#ifndef PI
#define PI 3.14159265358979323846
#endif

/**
 * @brief  Initialise K si nécessaire.
 */
static void CompassInitParameters(void)
{
    if (K == 0.0) {
        if ((tau + Te) != 0.0) {
            K = tau / (tau + Te);
        } else {
            // fallback pour éviter la division par 0
            K = 0.98;
        }
    }
}

/**
 * @brief Calibre un accéléromètre avec une méthode simple single-point.
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
    theta *= (180.0 / PI);
    psi   *= (180.0 / PI);
    phi   *= (180.0 / PI);

    // Affichage
    snprintf(msg, msg_len,
             "Theta(XY) = %.2f deg | Psi(XZ) = %.2f deg | Phi(Z) = %.2f deg\r\n",
             theta, psi, phi);
}

/**
 * @brief  Calcule l'orientation complète compensée en inclinaison (acc + mag)
 *         (identique à ta version, conservée)
 */
void TiltCompensatedCompass(const double acc[3], const double mag[3],
                            double *roll, double *pitch, double *heading){
    /* Roulis et tangage depuis acc (deg) */
    *roll  = atan2(acc[1], acc[2]) * (180.0 / PI);
    *pitch = atan2(-acc[0], sqrt(acc[1]*acc[1] + acc[2]*acc[2])) * (180.0 / PI);

    /* conversion en rad pour compensation magnétomètre */
    double roll_rad  = *roll  * (PI / 180.0);
    double pitch_rad = *pitch * (PI / 180.0);

    double Xh = mag[0]*cos(pitch_rad) + mag[2]*sin(pitch_rad);
    double Yh = mag[0]*sin(roll_rad)*sin(pitch_rad) + mag[1]*cos(roll_rad) - mag[2]*sin(roll_rad)*cos(pitch_rad);

    *heading = atan2(Yh, Xh) * (180.0 / PI);
    if(*heading < 0.0) *heading += 360.0;
}


/**
 * @brief  Ancienne fonction TiltCompensatedCompassAngles.
 *         Conservée comme fallback : applique une lissage simple sur les angles
 *         calculés uniquement à partir de l'accéléromètre (sans gyro).
 *
 *         Pour un vrai filtre complémentaire (acc+gyro) utilise la fonction
 *         TiltCompensatedComplementary(...) ci-dessus.
 */
void TiltCompensatedCompassAngles(const double acc[3], const double mag[3],
                                  double *roll, double *pitch, double *heading) {
    static double roll_prev = 0.0;
    static double pitch_prev = 0.0;

    /* angles depuis acc (deg) */
    double roll_acc  = atan2(acc[1], acc[2]) * (180.0 / PI);
    double pitch_acc = atan2(-acc[0], sqrt(acc[1]*acc[1] + acc[2]*acc[2])) * (180.0 / PI);

    /* utilisation d'un simple alpha pour lisser (exponential smoothing) */
    CompassInitParameters(); /* récupère K (on peut réutiliser K comme alpha si tu veux) */
    double alpha = 1.0 - K; /* si tau grand -> K proche de 1 -> alpha petit -> plus lissage */

    *roll  = alpha * roll_acc  + (1.0 - alpha) * roll_prev;
    *pitch = alpha * pitch_acc + (1.0 - alpha) * pitch_prev;

    roll_prev  = *roll;
    pitch_prev = *pitch;

    /* heading via mag si disponible */
    if (mag != NULL && heading != NULL) {
        double roll_rad  = (*roll)  * (PI / 180.0);
        double pitch_rad = (*pitch) * (PI / 180.0);

        double Xh = mag[0]*cos(pitch_rad) + mag[2]*sin(pitch_rad);
        double Yh = mag[0]*sin(roll_rad)*sin(pitch_rad) + mag[1]*cos(roll_rad) - mag[2]*sin(roll_rad)*cos(pitch_rad);

        *heading = atan2(Yh, Xh) * (180.0 / PI);
        if (*heading < 0.0) *heading += 360.0;
    } else if (heading != NULL) {
        *heading = NAN;
    }
}


/**
 * @brief  (NOUVEAU) Filtre complémentaire réel en discrete-time.
 *
 *         theta[n] = K * (theta[n-1] + Te * omega[n]) + (1-K) * theta_acc[n]
 *
 * @param acc         accéléromètre calibré [ax,ay,az] en g
 * @param gyro_rates  gyroscope [gx,gy,gz] en deg/s  <--- **Vérifie l'unité** de tes lectures !
 * @param mag         magnétomètre calibré [mx,my,mz] en µT (peut être NULL si tu ne veux pas heading)
 * @param roll        sortie roulis (deg)
 * @param pitch       sortie tangage (deg)
 * @param heading     sortie azimut nord magnétique (deg) -- si mag == NULL, heading = NAN
 */
void TiltCompensatedComplementary(const double acc[3], const double gyro_rates[3], const double mag[3],
                                  double *roll, double *pitch, double *heading)
{
    static double roll_prev = 0.0;
    static double pitch_prev = 0.0;

    // Initialisation de K
    CompassInitParameters();

    // Angles à partir de l'accéléromètre
    double roll_acc  = atan2(acc[1], acc[2]) * (180.0 / PI);
    double pitch_acc = atan2(-acc[0], sqrt(acc[1]*acc[1] + acc[2]*acc[2])) * (180.0 / PI);

    // Intégration du gyroscope
    double roll_from_gyro  = roll_prev  + Te * gyro_rates[0];
    double pitch_from_gyro = pitch_prev + Te * gyro_rates[1];

    // Formule proposée par l'énoncé
    *roll  = K * (roll_from_gyro)  + (1.0 - K) * roll_acc;
    *pitch = K * (pitch_from_gyro) + (1.0 - K) * pitch_acc;

    // Sauvegarde des valeurs
    roll_prev  = *roll;
    pitch_prev = *pitch;

    // Calcul du heading par mag
    if (mag != NULL && heading != NULL) {
        double roll_rad  = (*roll)  * (PI / 180.0);
        double pitch_rad = (*pitch) * (PI / 180.0);

        double Xh = mag[0]*cos(pitch_rad) + mag[2]*sin(pitch_rad);
        double Yh = mag[0]*sin(roll_rad)*sin(pitch_rad) + mag[1]*cos(roll_rad) - mag[2]*sin(roll_rad)*cos(pitch_rad);

        *heading = atan2(Yh, Xh) * (180.0 / PI);
        if (*heading < 0.0) *heading += 360.0;
    } else if (heading != NULL) {
        *heading = NAN;
    }
}

