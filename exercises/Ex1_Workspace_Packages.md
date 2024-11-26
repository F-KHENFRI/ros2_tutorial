# Exercice 1 : Apprendre à créer un Workspace et des Packages

## Sommaire
- [Exercice 1 : Apprendre à créer un Workspace et des Packages](#exercice-1--apprendre-à-créer-un-workspace-et-des-packages)
  - [Sommaire](#sommaire)
  - [Objectifs](#objectifs)
  - [Étape 1 : Vérifier le fichier **bashrc**](#étape-1--vérifier-le-fichier-bashrc)
  - [Étape 2 : Création d'un Workspace ROS 2](#étape-2--création-dun-workspace-ros-2)
    - [2.1 Structure du Workspace](#21-structure-du-workspace)
    - [2.2 Commandes pour créer le Workspace](#22-commandes-pour-créer-le-workspace)
    - [2.3 Commandes pour créer un package](#23-commandes-pour-créer-un-package)
  - [Conclusion](#conclusion)

---

## Objectifs

- Créer un **workspace** ROS 2.
- Créer des **packages**.

---

## Étape 1 : Vérifier le fichier **bashrc**

1. **Sourcez ROS 2 :**  
   Dans chaque terminal, vous devez exécuter cette commande pour utiliser les outils ROS 2 :  
   ```bash
   source /opt/ros/humble/setup.bash
   ```
   Pour automatiser cette étape, ajoutez cette commande à la fin de votre fichier **~/.bashrc**. Utilisez la commande suivante :
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   ```
   Ensuite, rechargez le fichier **~/.bashrc** :
   ```bash
   source ~/.bashrc
   ```

2. **Sourcez `colcon` :**  
   Ajoutez le support pour l'autocomplétion de `colcon` avec cette commande :
   ```bash
   source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
   ```
   Pour automatiser cette étape, ajoutez cette commande à la fin de votre fichier **~/.bashrc**. Utilisez la commande suivante :
   ```bash
   echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
   ```
   Ensuite, rechargez le fichier **~/.bashrc** :
   ```bash
   source ~/.bashrc
   ```

---

## Étape 2 : Création d'un Workspace ROS 2

### 2.1 Structure du Workspace
Un workspace ROS 2 typique comprend les éléments suivants :
- **src/** : Contient les packages ROS 2 que vous développez.
- **build/** : Généré automatiquement pour stocker les fichiers intermédiaires de compilation.
- **install/** : Contient les fichiers finaux des packages compilés.
- **log/** : Enregistre les journaux des processus de construction.

---

### 2.2 Commandes pour créer le Workspace

1. **Créer le répertoire du workspace avec le dossier `src/` :**  
   ```bash
   mkdir -p ~/ros_tutorial_ws/src
   ```

2. **Compiler (build) le workspace :**  
   ```bash
   cd ~/ros_tutorial_ws
   colcon build
   ```

3. **Vérifier la structure du workspace :**  
   Utilisez la commande suivante pour afficher les dossiers créés :  
   ```bash
   cd ..
   tree -d -L 1 ros_tutorial_ws/
   ```
   Si la commande tree n'est définie, exécutez la commande suivante:
   ```bash
   apt install tree
   ```
   **Résultat attendu :**  
   ```
   ros_tutorial_ws/
   ├── build
   ├── install
   ├── log
   └── src
   ```

4. **Sourcez le workspace automatiquement :**  
   Ajoutez cette ligne à la fin de votre fichier **~/.bashrc** :  
   ```bash
   echo "source /workspaces/ros2_tutorial/ros_tutorial_ws/install/setup.bash" >> ~/.bashrc
   ```
   Rechargez le fichier **~/.bashrc** pour appliquer les modifications immédiatement :  
   ```bash
   source ~/.bashrc
   ```

---

### 2.3 Commandes pour créer un package

1. **Se déplacer dans le dossier `src/` du workspace :**  
   ```bash
   cd ros_tutorial_ws/src/
   ```

2. **Créer un package ROS 2 nommé `turtle_controller` :**  
   ```bash
   ros2 pkg create turtle_controller --build-type ament_python --dependencies rclpy
   ```

3. **Vérifier la structure du package :**  
   ```bash
   tree turtle_controller/
   ```
   **Résultat attendu :**  
   ```
   turtle_controller/
   ├── package.xml
   ├── resource
   │   └── turtle_controller
   ├── setup.cfg
   ├── setup.py
   ├── test
   │   ├── test_copyright.py
   │   ├── test_flake8.py
   │   └── test_pep257.py
   └── turtle_controller
       └── __init__.py
   ```

4. **Recompiler le workspace :**  
   ```bash
   cd ..
   colcon build
   ```

5. **Sourcez le workspace :**  
   ```bash
   source ~/.bashrc
   ```

---

## Conclusion

Dans cet exercice, nous avons suivi plusieurs étapes clés pour apprendre à configurer un **workspace ROS 2** et créer un package fonctionnel pour un projet robotique :

1. **Configuration de l'environnement :** Nous avons vérifié et configuré le fichier **bashrc** pour sourcer ROS 2 et activer les fonctionnalités de `colcon`, ce qui garantit un environnement de travail stable et prêt à l'emploi.

2. **Création du workspace :** Nous avons structuré un workspace ROS 2 en créant le dossier `src/` pour y accueillir les packages, puis nous avons utilisé la commande `colcon build` pour générer les répertoires nécessaires, tels que `build/`, `install/` et `log/`.

3. **Automatisation :** Nous avons configuré le sourcing automatique du workspace dans le fichier **bashrc**, facilitant ainsi son utilisation dans tous les nouveaux terminaux.

4. **Création d’un package :** Nous avons appris à créer un package ROS 2 nommé `turtle_controller` avec les dépendances nécessaires et avons exploré sa structure par défaut, incluant les fichiers essentiels comme `package.xml` et `setup.py`.

5. **Compilation et vérification :** Enfin, nous avons recompilé le workspace pour inclure le nouveau package et confirmé que tout était correctement configuré.
