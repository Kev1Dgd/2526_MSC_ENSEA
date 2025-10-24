## 🤖 Robotique – ROS2 (Robot Operating System)

### 📘 Présentation de la matière

La matière **Robotique (ROS2)** introduit le framework **Robot Operating System (ROS)**, un environnement logiciel open source conçu pour la programmation, la communication et la simulation de robots.

ROS2 n’est pas un véritable système d’exploitation, mais un **middleware** permettant la communication entre différents composants logiciels d’un robot, appelés **nœuds (nodes)**.
Il fournit une structure modulaire, une architecture distribuée, ainsi qu’un large ensemble d’outils pour la **simulation**, la **visualisation** et le **contrôle de robots**.

---

### 🧩 Concepts fondamentaux du cours

#### 🔹 ROS et son écosystème

* **ROS (Robot Operating System)** est un framework open source issu du *Stanford Artificial Intelligence Lab* et aujourd’hui maintenu par *Open Robotics*.
* Fonctionne principalement sous **Linux Ubuntu**, mais dispose de bindings pour d’autres langages (C++, Python, Java…).
* Fournit :

  * **Plomberie** : infrastructure de communication (nodes, topics, services, actions)
  * **Outils** : Rosbag, Rviz2, Gazebo, RQT
  * **Fonctionnalités** : perception, navigation, contrôle moteur, manipulation
  * **Communauté** : vaste réseau de chercheurs, développeurs et industriels

---

#### 🔹 ROS1 vs ROS2

| Élément          | ROS1                      | ROS2                                           |
| ---------------- | ------------------------- | ---------------------------------------------- |
| Communication    | Centralisée via `roscore` | Distribuée via DDS (Data Distribution Service) |
| Architecture     | Monorobot                 | Multi-robots (swarm-ready)                     |
| Fiabilité réseau | Limitée                   | Temps réel, tolérance aux fautes               |
| Support OS       | Linux principalement      | Linux, Windows, macOS                          |
| Langages         | C++, Python               | C++, Python                                    |

ROS2 repose sur le protocole **DDS**, qui gère automatiquement la distribution des données entre les nœuds, sans serveur central.

---

#### 🔹 Concepts clés de ROS2

| Concept         | Description                                                                         |
| --------------- | ----------------------------------------------------------------------------------- |
| **Node**        | Processus exécutable représentant un composant du robot (capteur, contrôleur, etc.) |
| **Topic**       | Canal de communication basé sur le modèle *publisher/subscriber*                    |
| **Service**     | Communication synchrone (requête/réponse)                                           |
| **Action**      | Service étendu pour tâches longues avec suivi de progression                        |
| **Package**     | Ensemble de code source, scripts et fichiers de configuration                       |
| **Parameter**   | Valeurs de configuration modifiables à chaud                                        |
| **Launch file** | Fichier (Python) permettant de lancer plusieurs nœuds simultanément                 |

---

#### 🧠 Outils ROS2

* **CLI (ros2)** : commandes pour gérer les nœuds, topics, services, actions et packages.
* **rqt / rqt_graph** : interface graphique pour visualiser les topics, nodes et paramètres.
* **rviz2** : visualisation 3D (cartes, nuages de points, positions…).
* **Gazebo** : simulateur physique complet pour tester les robots.
* **rosbag** : enregistrement et lecture de flux de messages ROS.

---

### 🧪 TP – Commande d’une tortue via joystick et développement d’un node ROS2

#### 🎯 Objectif

Mettre en œuvre un environnement **ROS2** pour piloter un robot virtuel (*turtlesim*) à l’aide d’un **joystick**, puis développer un **nœud personnalisé** simulant une commande de **chenille (type char d’assaut)** à l’aide de deux axes analogiques.

---

### ⚙️ Étapes de réalisation

#### 1. Installation de ROS2

* Téléchargement de la **VM fournie** par l’enseignant *(ROS2 déjà installé)*
  ou
* Installation manuelle sur **Ubuntu 22.04+** :
  👉 [Documentation officielle ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

---

#### 2. Dispositif de base

Lancer les nœuds suivants :

```bash
ros2 run joy joy_node
ros2 run teleop_twist_joy teleop_twist_joy_node
ros2 run turtlesim turtlesim_node
```

* `joy` → lit les données du joystick
* `teleop_twist_joy` → convertit les commandes joystick en messages `Twist`
* `turtlesim` → simule la tortue mobile

Vérification :

```bash
sudo dmesg           # détecter le joystick
ls /dev/input/       # vérifier la présence de jsX
rqt_graph            # visualiser la communication entre les nœuds
```

---

#### 3. Enregistrement et lecture avec **ROS Bag**

* Créer un répertoire :

  ```bash
  mkdir bag_files && cd bag_files
  ```
* Enregistrement :

  ```bash
  ros2 bag record /joy
  ```
* Arrêt puis suppression de la tortue :

  ```bash
  ros2 service call /reset std_srvs/srv/Empty {}
  ```
* Lecture de la séquence enregistrée :

  ```bash
  ros2 bag play <nom_du_fichier>
  ```

Cette méthode permet de **rejouer un scénario sans joystick physique**, pratique pour le débogage.

---

#### 4. Création d’un node personnalisé : *chenille_node*

Objectif : contrôler la tortue comme un véhicule à **deux chenilles indépendantes** (stick gauche/droite).

##### 🧱 Étapes de création

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python --node-name chenille_node chenille_msc
cd ~/ros2_ws
colcon build
source install/local_setup.bash
```

##### 💡 Exemple de code Python simplifié

```python
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import rclpy

class ChenilleNode(Node):
    def __init__(self):
        super().__init__('chenille_node')
        self.subscriber = self.create_subscription(Joy, '/joy', self.listener_callback, 10)
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def listener_callback(self, joy):
        twist = Twist()
        # Exemple : deux axes de joystick pour les chenilles
        left = joy.axes[1]
        right = joy.axes[2]
        twist.linear.x = (left + right) / 2.0
        twist.angular.z = (right - left) / 2.0
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ChenilleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

Exécution :

```bash
ros2 run chenille_msc chenille_node
```

---

### 📊 Résultats attendus

* Tortue **pilotée par joystick ou fichier rosbag**
* Node personnalisé fonctionnel (`chenille_node`)
* Communication ROS2 maîtrisée : Publisher, Subscriber, Topics
* Utilisation des outils **rqt**, **ros2 bag**, **turtlesim**

---

### 📚 Références

* Cours : *Robot Operating System (ROS2)* – Christophe Barès, ENSEA
* Documentation ROS2 : [https://docs.ros.org](https://docs.ros.org)
* Forum d’aide : [https://robotics.stackexchange.com](https://robotics.stackexchange.com)

