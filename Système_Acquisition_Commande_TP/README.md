# 🏎️ Projet MSC : Asservissement de Machine à Courant Continu (MCC)
**ENSEA 2025-2026** DUGARD Kevin & FLANDIN Elio  
**Encadrant :** Nicolas Papazoglou

---

## 📌 Présentation du Projet
Ce projet consiste à réaliser le pilotage complet d'un moteur à courant continu via un **hacheur en pont en H** et une carte **Nucleo-STM32G474RE**. 

L'objectif est d'implémenter :
1. La génération de signaux **PWM complémentaires décalés**.
2. L'acquisition de données capteurs (courant, vitesse).
3. L'asservissement en temps réel du système.

---

## 🛠️ Architecture du Logiciel
Le projet est structuré de manière modulaire pour faciliter la maintenance et la portabilité :

* **`acquisition/`** : Gestion des entrées analogiques (ADC/DMA) et des codeurs incrémentaux.
* **`motor_control/`** : Pilotage du hacheur (PWM) et algorithmes d'asservissement (PID).
* **`user_interface/`** : Interaction via le `shell` UART, gestion des LEDs et boutons.
* **`app.c`** : Point d'entrée de l'application (initialisation et boucle principale).



---

## 🚀 Fonctionnalités du Shell
Nous utilisons un terminal série (115200 baud) pour piloter le moteur en temps réel.

| Commande | Action |
| :--- | :--- |
| `help` | Affiche la liste des commandes disponibles. |
| `start` | Active les PWM et fixe le rapport cyclique à 50% (repos). |
| `stop` | Désactive immédiatement les sorties PWM. |
| `speed <val>` | Définit la vitesse cible avec une rampe d'accélération. |
| `test` | Vérifie la transmission des arguments. |

---

## 📈 État d'avancement du TP

### 1. Commande Basique (PWM)
- [x] Configuration du Timer à **20kHz**.
- [x] Insertion du **temps mort** (Deadtime) selon la datasheet.
- [x] Validation à l'oscilloscope des signaux complémentaires.
- [x] Implémentation de la rampe d'accélération pour éviter les pics de courant.

### 2. Acquisition de Données
- [ ] Mesure du courant par **ADC en mode DMA** (synchrone avec la PWM).
- [ ] Lecture de la vitesse via le **codeur incrémental**.
- [ ] Calcul de la fonction de transfert du capteur de vitesse.

### 3. Asservissement (En cours...)
- [ ] Identification de la constante de temps mécanique.
- [ ] Implémentation du correcteur PID.

---

## 🛠 Configuration Matérielle (Hardware)

### Hacheur & PWM
* **Fréquence :** 20 kHz
* **Résolution :** > 10 bits
* **Topologie :** Commande complémentaire décalée (4 transistors).





---

## 📝 Bonnes Pratiques de Code
Pour ce projet, nous respectons les règles suivantes :
* **Naming :** `snake_case` pour les variables, `UPPER_CASE` pour les macros.
* **Zéro "Nombre Magique" :** Utilisation systématique de `#define`.
* **Sécurité :** Vérification systématique du retour des fonctions HAL.
* **Modularité :** Fonctions factorisées pour éviter la duplication.

---

## ⚙️ Installation et Utilisation
1. Cloner le dépôt :
   ```bash
   git clone [https://github.com/votre_nom/2526_MSC_SAC_NOM1_NOM2.git](https://github.com/votre_nom/2526_MSC_SAC_NOM1_NOM2.git)
