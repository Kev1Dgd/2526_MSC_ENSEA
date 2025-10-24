## 🧠 Bus et Réseaux Industriels

### 📘 Présentation de la matière

La matière **Bus et Réseaux Industriels** aborde les différents moyens de communication utilisés dans les systèmes embarqués et les environnements industriels.
L’objectif est de comprendre **comment les composants électroniques échangent des données** à travers divers **protocoles de bus de terrain**.

Les points clés du cours :

* **Définition fonctionnelle d’un bus** : transfert de données, formats (uint8, int16, float…), communication série/parallèle, synchrone/asynchrone.
* **Signalisations** : simple (single-ended) ou différentielle.
* **Topologies et gestion de collisions** : simplex, half/full duplex, CSMA/CD, CSMA/CA, CSMA/CR.
* **Bus de terrain (Fieldbus)** :

  * **UART / RS232**
  * **SPI**
  * **I²C**
  * **CAN**
  * **LIN**

Chaque bus a ses propres caractéristiques (vitesse, distance, topologie, gestion des erreurs, nombre de nœuds, etc.), et leur maîtrise est essentielle pour concevoir des systèmes de communication embarqués fiables et performants.

---

### 🧪 TP – Communication I²C et UART avec le STM32 et le capteur BMP280

#### 🎯 Objectif

Mettre en œuvre la communication entre un **microcontrôleur STM32** et un **capteur BMP280** (température et pression) via le **bus I²C**, puis afficher les valeurs compensées sur le **port série UART (USB)**.

---

### ⚙️ Étapes de réalisation

#### 1. Configuration du STM32

* Logiciel utilisé : **STM32CubeIDE**
* Mise en place :

  * **I²C** : broches PB8 (SCL) et PB9 (SDA)
  * **UART sur USB** : broches PA2 (TX) et PA3 (RX)
* Activation des interruptions pour I²C et UART.
* Redirection du `printf()` vers la liaison série pour le débogage :

  ```c
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  ```
* Test de la chaîne de compilation avec un programme *heartbeat* : un `printf` toutes les secondes.

---

#### 2. Communication I²C – Capteur BMP280

* Capteur : **BMP280 (Bosch)** – capteur de pression et température.
* Adresse I²C : `0x77` (ou `0x76` selon la configuration).
* Étapes :

  * Lecture du registre ID (`0xD0`) → valeur attendue `0x58`.
  * Configuration du registre `ctrl_meas` en **mode normal** :
    `0x57 = 0b01010111` (Température ×2, Pression ×16).
  * Lecture des registres de **température** (`0xFA...`) et **pression** (`0xF7...`).

---

#### 3. Calcul des valeurs compensées

* Lecture des registres d’étalonnage (`0x88` à `0xA1`).
* Calcul de la **température** et de la **pression compensées** en **entier 32 bits** selon les formules de la datasheet Bosch.
* Transmission sur le port série :

  ```
  Température : XX.X °C
  Pression : XXXX.X Pa
  ```

---

### 🧩 Technologies et concepts utilisés

| Élément         | Description                                                       |
| --------------- | ----------------------------------------------------------------- |
| **STM32**       | Microcontrôleur ARM utilisé pour la communication embarquée       |
| **HAL Library** | Bibliothèque d’abstraction matérielle de STMicroelectronics       |
| **I²C**         | Bus série synchrone utilisé pour connecter le BMP280              |
| **UART**        | Communication série asynchrone pour transmettre les données au PC |
| **BMP280**      | Capteur de température et de pression, communication I²C          |

---

### 💻 Résultats attendus

* Communication I²C fonctionnelle (vérifiée à l’oscilloscope ou via logs).
* Valeurs brutes et compensées affichées sur le terminal série.
* Code organisé et commenté sous STM32CubeIDE.

---

### 📚 Références

* Cours : *Bus et Réseaux Industriels* – Christophe Barès, ENSEA
* Datasheet : [BMP280 – Bosch Sensortec](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp280/)
* Documentation STM32 HAL : [STMicroelectronics HAL API](https://www.st.com/en/embedded-software/stm32cubef4.html)

