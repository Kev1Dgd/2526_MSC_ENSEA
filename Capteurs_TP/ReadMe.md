# 🛠️ TP MSC - Capteurs (G431-MSC-Sensors_Flandin&Dugard)

**Travaux Pratiques - F. Goutailler, T. Tang, N. Simond**

Ce projet a pour objectif de mettre en pratique les concepts de capteurs et systèmes embarqués sur la carte **STM32 Nucleo G431RB** avec la centrale inertielle **IMU9250**. Il couvre :

* Lecture et acquisition des données (accéléromètre, gyromètre, magnétomètre, température)
* Gestion du bus **I2C**
* Calibration des capteurs
* Conception d’une boussole électronique robuste et multi-capteurs
* Paramètres métrologiques : sensibilité, bande-passante, fréquence d’échantillonnage

## 📦 Matériel et logiciels utilisés

* Carte STM32 Nucleo-G431RB
* IMU9250 (accéléromètre, gyromètre, magnétomètre, capteur de température)
* **STM32Cube IDE** (configuration, développement et debug)
* **Doxygen 1.15.0** pour la documentation automatique
* Matlab ou Octave pour analyse et visualisation des données

## 📚 Documentation

La documentation Doxygen est disponible [ici](./STM32CubeIde_projet/html/index.html) 🌐
Elle contient :

* Description des fichiers sources (`main.c`, `functions.c`, `functions.h`, `compass.c`, `compass.h`)
* Prototypes des fonctions et commentaires Doxygen
* Diagrammes et hiérarchie des fonctions

> ⚠️ Assurez-vous que le chemin vers `index.html` correspond à votre arborescence locale.

## 📝 Livrables attendus

1. Fichiers de code : `main.c`, `functions.c`, `functions.h`, `compass.c`, `compass.h`
2. Documentation PDF générée par Doxygen
3. Document de synthèse (diaporama) avec réponses aux questions et captures de tests

## 🧩 Structure du projet

```
Capteurs_TP/
├── Docs/                    # Documentation 
├── MATLAB/                  # Fichiers MATLAB pour simulation
├── STM32CubeIde_project/    # Projet STM32CubeIDE
│   ├── Core/                # Fichiers générés automatiquement par STM32Cube
│   │   ├── main.c
│   │   └── main.h
│   ├── Utils/               # Votre code utilisateur
│   │   ├── functions.c
│   │   ├── functions.h
│   │   ├── utils.c
│   │   ├── utils.h
│   │   ├── compass.c
│   │   └── compass.h
│   ├── html/                # Documentation Doxygen
│   │   └── index.html
│   ├── .ioc                 # Fichier de configuration STM32Cube
└── README.md
```

## 🚀 Objectifs pédagogiques

* Maîtrise de **STM32Cube IDE**
* Utilisation de Doxygen pour la documentation du code
* Compréhension du protocole **I2C** et des capteurs embarqués
* Calibration et optimisation des capteurs
* Conception de filtres complémentaires pour la fusion de capteurs

