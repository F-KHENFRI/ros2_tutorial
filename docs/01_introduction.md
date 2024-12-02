# Introduction à ROS 2

## Sommaire

- [Introduction à ROS 2](#introduction-à-ros-2)
  - [Sommaire](#sommaire)
  - [Objectifs de ce document](#objectifs-de-ce-document)
  - [Qu'est-ce que ROS 2 ?](#quest-ce-que-ros-2-)
  - [Architecture de ROS 2](#architecture-de-ros-2)
    - [Préparation : Installation et Configuration](#préparation--installation-et-configuration)
      - [1. Vérifiez que ROS 2 est installé](#1-vérifiez-que-ros-2-est-installé)
      - [2. Installation de `colcon`](#2-installation-de-colcon)
      - [3. Configuration automatique](#3-configuration-automatique)
    - [Création d’un Workspace](#création-dun-workspace)
      - [1. Créez la structure du workspace](#1-créez-la-structure-du-workspace)
      - [2. Configurez le sourcing du workspace](#2-configurez-le-sourcing-du-workspace)
      - [3. Vérifiez la structure du workspace](#3-vérifiez-la-structure-du-workspace)
    - [Création d’un Package](#création-dun-package)
    - [Exercice : Créez votre Workspace et Package](#exercice--créez-votre-workspace-et-package)
      - [Objectif](#objectif)
      - [Instructions](#instructions)
  - [Outils Essentiels dans ROS 2](#outils-essentiels-dans-ros-2)
    - [1. **Turtlesim**](#1-turtlesim)
    - [2. **ROS 2 CLI (Command Line Interface)**](#2-ros-2-cli-command-line-interface)
    - [3. **Exemples Pratiques avec Turtlesim**](#3-exemples-pratiques-avec-turtlesim)
  
---

## Objectifs de ce document
- Comprendre ce qu'est ROS 2 et pourquoi il est essentiel dans le développement de robots.
- Identifier les principaux outils de ROS 2.
- Installer et configurer les outils nécessaires.
- Créer un **workspace**.
- Créer un **package** dans ROS 2.

---

## Qu'est-ce que ROS 2 ?

ROS 2 (Robot Operating System 2) est un **framework open-source** conçu pour simplifier le développement d'applications robotiques. Il fournit des bibliothèques et des outils pour gérer la communication entre différents composants d'un robot. ROS 2 est l'évolution de ROS 1, conçu pour répondre aux besoins modernes tels que :

- **Sécurité** : Gestion des données sensibles grâce au middleware DDS.
- **Temps réel** : Compatibilité avec des systèmes temps réel pour des robots critiques.
- **Modularité** : Déploiement sur des systèmes distribués, avec des nœuds fonctionnant sur plusieurs machines ou plateformes (robots, drones, serveurs cloud, etc.).
- **Portabilité** : Fonctionne sur des systèmes d'exploitation variés (Linux, Windows, macOS).

---

## Architecture de ROS 2

Les concepts principaux incluent :

- **Nœuds** : Petits programmes qui exécutent une tâche spécifique (ex. lire un capteur, contrôler un moteur).
- **Topics** : Canaux de communication unidirectionnels utilisés pour l'échange de messages.
- **Services** : Communication bidirectionnelle synchrone (requête-réponse).
- **Actions** : Comme les services, mais pour des tâches longues, avec des mises à jour d'état.
- **Paramètres** : Valeurs configurables pour ajuster dynamiquement le comportement des nœuds.

---

### Préparation : Installation et Configuration

#### 1. Vérifiez que ROS 2 est installé
Assurez-vous que ROS 2 est installé. La procédure peut varier en fonction de votre système d’exploitation et de la version de ROS 2 utilisée. Suivez les instructions officielles disponibles sur [le site de ROS 2](https://docs.ros.org/en/).

Pour vérifier si ROS 2 est correctement installé :
1. Sourcez votre installation ROS 2 :
   ```bash
   source /opt/ros/<votre_version>/setup.bash
   ```
   Remplacez `<votre_version>` par la version de ROS 2 installée, comme `foxy`, `galactic`, `humble`, etc.

2. Vérifiez la version :
   ```bash
   ros2 --version
   ```

#### 2. Installation de `colcon`
`colcon` est l’outil recommandé pour construire les workspaces dans ROS 2.

- Installez `colcon` si ce n’est pas déjà fait :
  ```bash
  sudo apt install python3-colcon-common-extensions
  ```

- Activez l’autocomplétion pour `colcon` :
  ```bash
  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
  ```

#### 3. Configuration automatique
Ajoutez les commandes nécessaires au fichier `~/.bashrc` pour éviter de les retaper à chaque session :

1. Ouvrez ou éditez le fichier `~/.bashrc` :
   ```bash
   gedit ~/.bashrc
   ```

2. Ajoutez les lignes suivantes à la fin du fichier :
   ```bash
   source /opt/ros/<votre_version>/setup.bash
   source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
   ```

3. Rechargez le fichier `~/.bashrc` pour appliquer les modifications immédiatement :
   ```bash
   source ~/.bashrc
   ```

---

### Création d’un Workspace

#### 1. Créez la structure du workspace
Un workspace est un répertoire structuré où les packages ROS 2 sont développés et compilés.

1. Créez un répertoire pour le workspace et son sous-dossier `src` :
   ```bash
   mkdir -p ~/<nom_du_workspace>/src
   cd ~/<nom_du_workspace>
   ```

2. Compilez le workspace (même s’il est encore vide) :
   ```bash
   colcon build
   ```

#### 2. Configurez le sourcing du workspace
Ajoutez le sourcing automatique de votre workspace à votre fichier `~/.bashrc` :
```bash
echo "source ~/<nom_du_workspace>/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 3. Vérifiez la structure du workspace
Une fois la compilation effectuée, le workspace doit avoir la structure suivante :
```
<nom_du_workspace>/
├── build/
├── install/
├── log/
└── src/
```

---

### Création d’un Package

1. Naviguez vers le dossier `src` de votre workspace :
   ```bash
   cd ~/<nom_du_workspace>/src
   ```

2. Créez un package avec le type de build souhaité (`ament_cmake` ou `ament_python`) :
   - Pour un package basé sur CMake :
     ```bash
     ros2 pkg create <nom_du_package> --build-type ament_cmake --dependencies rclcpp
     ```
   - Pour un package basé sur Python :
     ```bash
     ros2 pkg create <nom_du_package> --build-type ament_python --dependencies rclpy
     ```

3. Compilez de nouveau le workspace pour inclure le package :
   ```bash
   cd ~/<nom_du_workspace>
   colcon build
   ```

4. Sourcez votre workspace pour activer le nouveau package :
   ```bash
   source ~/<nom_du_workspace>/install/setup.bash
   ```

---

### Exercice : Créez votre Workspace et Package

#### Objectif
Créer un workspace ROS 2 et y ajouter un package simple pour poser les bases de vos futurs projets.

#### Instructions
1. Créez un workspace nommé `turtle_ws` dans votre répertoire personnel.
2. Configurez ce workspace pour qu’il soit sourcé automatiquement à chaque session.
3. Ajoutez un package nommé `turtle_controller` avec les dépendances nécessaires pour ROS 2.

---


## Outils Essentiels dans ROS 2

---

### 1. **Turtlesim**
**Turtlesim** est un outil éducatif et pratique pour apprendre les bases de ROS 2. C'est une simulation simple d'une tortue qui se déplace dans un environnement bidimensionnel. Turtlesim est souvent utilisé pour expérimenter les concepts de ROS 2, tels que les topics, les services, et les actions.

**Pourquoi utiliser Turtlesim ?**
- Il permet de visualiser et tester les concepts de ROS 2 en temps réel.
- Les commandes CLI de ROS 2 peuvent être appliquées facilement.
- Il fournit une base pour explorer la gestion des topics et des services.

---

### 2. **ROS 2 CLI (Command Line Interface)**
**Description** : Interface en ligne de commande permettant d’interagir avec les fonctionnalités de ROS 2, telles que les topics, services, nœuds, paramètres, et plus encore.

**Exemple avec Turtlesim** :
Avant de commencer, démarrez un nœud Turtlesim dans un terminal :
```bash
ros2 run turtlesim turtlesim_node
```

Voici un tableau regroupant les commandes principales du CLI de **ROS 2** avec des exemples appliqués à Turtlesim :

| **Commande**                       | **Description**                                                           | **Exemple appliqué à Turtlesim**                                                              |
| ---------------------------------- | ------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------- |
| **ros2 topic**                     | **Gestion des topics (pub/sub)**                                          |                                                                                               |
| `ros2 topic list`                  | Liste tous les topics actifs sur le système.                              | `ros2 topic list`                                                                             |
| `ros2 topic info /topic_name`      | Affiche des informations sur un topic spécifique.                         | `ros2 topic info /turtle1/cmd_vel`                                                            |
| `ros2 topic echo /topic_name`      | Affiche les messages publiés sur un topic en temps réel.                  | `ros2 topic echo /turtle1/pose`                                                               |
| `ros2 topic pub /topic_name`       | Publie un message sur un topic (nécessite le type de message).            | `ros2 topic pub /turtle1/cmd_vel geometry_msgs/Twist "{linear: {x: 2.0}, angular: {z: 1.0}}"` |
| **ros2 service**                   | **Gestion des services (request/response)**                               |                                                                                               |
| `ros2 service list`                | Liste tous les services disponibles.                                      | `ros2 service list`                                                                           |
| `ros2 service call /srv_name`      | Appelle un service spécifique (nécessite les arguments et le type).       | `ros2 service call /clear std_srvs/srv/Empty`                                                 |
| `ros2 service type /srv_name`      | Affiche le type d'un service.                                             | `ros2 service type /spawn`                                                                    |
| `ros2 service find srv_type`       | Trouve les services du type spécifié.                                     | `ros2 service find turtlesim/srv/TeleportAbsolute`                                            |
| **ros2 node**                      | **Gestion des nœuds**                                                     |                                                                                               |
| `ros2 node list`                   | Liste tous les nœuds en cours d'exécution.                                | `ros2 node list`                                                                              |
| `ros2 node info /node_name`        | Affiche des informations sur un nœud spécifique (topics, services, etc.). | `ros2 node info /turtlesim`                                                                   |
| **ros2 param**                     | **Gestion des paramètres des nœuds**                                      |                                                                                               |
| `ros2 param list`                  | Liste tous les paramètres d'un nœud ou de tous les nœuds.                 | `ros2 param list /turtlesim`                                                                  |
| `ros2 param set /node param value` | Définit une nouvelle valeur pour un paramètre d'un nœud.                  | `ros2 param set /turtlesim background_b 255`                                                  |
| **ros2 msg**                       | **Gestion des messages**                                                  |                                                                                               |
| `ros2 msg list`                    | Liste tous les types de messages disponibles.                             | `ros2 msg list`                                                                               |
| `ros2 msg show msg_type`           | Affiche la définition d'un type de message spécifique.                    | `ros2 msg show geometry_msgs/Twist`                                                           |

---

### 3. **Exemples Pratiques avec Turtlesim**

1. **Faire avancer la tortue** :
   - Publiez un message sur le topic `/turtle1/cmd_vel` pour déplacer la tortue :
     ```bash
     ros2 topic pub /turtle1/cmd_vel geometry_msgs/Twist "{linear: {x: 1.0}, angular: {z: 0.5}}"
     ```

2. **Réinitialiser le tableau** :
   - Appelez le service `/clear` pour nettoyer la fenêtre de simulation :
     ```bash
     ros2 service call /clear std_srvs/srv/Empty
     ```

3. **Changer la couleur d'arrière-plan** :
   - Changez la couleur d'arrière-plan en modifiant un paramètre du nœud :
     ```bash
     ros2 param set /turtlesim background_r 0
     ros2 param set /turtlesim background_g 0
     ros2 param set /turtlesim background_b 255
     ```
