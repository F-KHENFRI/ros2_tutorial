# Introduction à ROS 2

## Objectifs de ce document
- Comprendre ce qu'est ROS 2 et pourquoi il est essentiel dans le développement de robots.
- Identifier les principaux outils de ROS 2.
- Découvrir les packages reconnus pour des fonctionnalités courantes.

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

## Outils essentiels dans ROS 2

### 1. **Colcon**
- **Description** : Outil de build utilisé pour compiler des packages ROS 2.
- **Commandes utiles** :
  ```bash
  colcon build
  colcon test
  colcon clean

### 2. **ROS 2 CLI**
- **Description** : Interface en ligne de commande pour interagir avec ROS 2.
Voici un tableau regroupant les principales commandes de l'interface en ligne de commande (CLI) de **ROS 2**, organisées par catégorie et avec une description de chaque commande :

---

| **Commande**                              | **Description**                                                           | **Exemple**                                                                    |
| ----------------------------------------- | ------------------------------------------------------------------------- | ------------------------------------------------------------------------------ |
| **ros2 topic**                            | **Gestion des topics (pub/sub)**                                          |                                                                                |
| `ros2 topic list`                         | Liste tous les topics actifs sur le système.                              | `ros2 topic list`                                                              |
| `ros2 topic info /topic_name`             | Affiche des informations sur un topic spécifique.                         | `ros2 topic info /chatter`                                                     |
| `ros2 topic echo /topic_name`             | Affiche les messages publiés sur un topic en temps réel.                  | `ros2 topic echo /chatter`                                                     |
| `ros2 topic pub /topic_name`              | Publie un message sur un topic (nécessite le type de message).            | `ros2 topic pub /chatter std_msgs/String "data: Hello"`                        |
| `ros2 topic hz /topic_name`               | Mesure la fréquence à laquelle des messages sont publiés sur un topic.    | `ros2 topic hz /chatter`                                                       |
| **ros2 service**                          | **Gestion des services (request/response)**                               |                                                                                |
| `ros2 service list`                       | Liste tous les services disponibles.                                      | `ros2 service list`                                                            |
| `ros2 service call /srv_name`             | Appelle un service spécifique (nécessite les arguments et le type).       | `ros2 service call /add_two_ints example_interfaces/AddTwoInts "{a: 1, b: 2}"` |
| `ros2 service type /srv_name`             | Affiche le type d'un service.                                             | `ros2 service type /add_two_ints`                                              |
| `ros2 service find srv_type`              | Trouve les services du type spécifié.                                     | `ros2 service find example_interfaces/AddTwoInts`                              |
| **ros2 action**                           | **Gestion des actions (goal/result)**                                     |                                                                                |
| `ros2 action list`                        | Liste toutes les actions disponibles.                                     | `ros2 action list`                                                             |
| `ros2 action send_goal`                   | Envoie un objectif (goal) à une action.                                   | `ros2 action send_goal /move_to nav2_msgs/MoveBaseGoal "{x: 1.0, y: 2.0}"`     |
| `ros2 action info /action`                | Affiche des informations sur une action spécifique.                       | `ros2 action info /move_to`                                                    |
| **ros2 node**                             | **Gestion des nœuds**                                                     |                                                                                |
| `ros2 node list`                          | Liste tous les nœuds en cours d'exécution.                                | `ros2 node list`                                                               |
| `ros2 node info /node_name`               | Affiche des informations sur un nœud spécifique (topics, services, etc.). | `ros2 node info /my_node`                                                      |
| **ros2 param**                            | **Gestion des paramètres des nœuds**                                      |                                                                                |
| `ros2 param list`                         | Liste tous les paramètres d'un nœud ou de tous les nœuds.                 | `ros2 param list /my_node`                                                     |
| `ros2 param get /node param`              | Récupère la valeur d'un paramètre d'un nœud spécifique.                   | `ros2 param get /my_node parameter_name`                                       |
| `ros2 param set /node param`              | Définit une nouvelle valeur pour un paramètre d'un nœud.                  | `ros2 param set /my_node parameter_name 42`                                    |
| **ros2 launch**                           | **Lancement de fichiers launch**                                          |                                                                                |
| `ros2 launch package_name file.launch.py` | Lance un fichier launch Python spécifique dans un package.                | `ros2 launch my_package my_launch_file.launch.py`                              |
| **ros2 bag**                              | **Enregistrement et lecture de données**                                  |                                                                                |
| `ros2 bag record /topic1 /topic2`         | Enregistre les données de certains topics dans un fichier.                | `ros2 bag record /chatter /cmd_vel`                                            |
| `ros2 bag play bag_file`                  | Rejoue les données enregistrées d'un fichier bag.                         | `ros2 bag play my_bag.bag`                                                     |
| **ros2 pkg**                              | **Gestion des packages**                                                  |                                                                                |
| `ros2 pkg list`                           | Liste tous les packages disponibles.                                      | `ros2 pkg list`                                                                |
| `ros2 pkg create`                         | Crée un nouveau package ROS 2.                                            | `ros2 pkg create my_package --build-type ament_cmake`                          |
| `ros2 pkg executables package_name`       | Liste les exécutables disponibles dans un package donné.                  | `ros2 pkg executables turtlesim`                                               |
| **ros2 msg**                              | **Gestion des messages**                                                  |                                                                                |
| `ros2 msg list`                           | Liste tous les types de messages disponibles.                             | `ros2 msg list`                                                                |
| `ros2 msg show msg_type`                  | Affiche la définition d'un type de message spécifique.                    | `ros2 msg show std_msgs/String`                                                |
| **ros2 interface**                        | **Gestion générale des interfaces (topics, services, actions)**           |                                                                                |
| `ros2 interface list`                     | Liste toutes les interfaces disponibles (topics, services et actions).    | `ros2 interface list`                                                          |
| `ros2 interface show interface_type`      | Affiche la définition d'une interface spécifique.                         | `ros2 interface show example_interfaces/AddTwoInts`                            |


