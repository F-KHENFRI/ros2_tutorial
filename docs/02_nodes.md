# Les Nœuds dans ROS2

Dans ROS2 (Robot Operating System 2), les **nœuds** sont des processus indépendants qui exécutent des tâches spécifiques. Ils constituent les blocs de base d'une application ROS2, permettant aux développeurs de concevoir des systèmes robotiques modulaires et distribués.

---

## Sommaire


- [Les Nœuds dans ROS2](#les-nœuds-dans-ros2)
  - [Sommaire](#sommaire)
  - [1. Introduction aux Nœuds ROS2](#1-introduction-aux-nœuds-ros2)
  - [2. Créer un Nœud Simple](#2-créer-un-nœud-simple)
    - [Exemple : Nœud Simple](#exemple--nœud-simple)
  - [3. Lancer et Tester un Nœud](#3-lancer-et-tester-un-nœud)
    - [Étape 1 : Créer un Package ROS2](#étape-1--créer-un-package-ros2)
    - [Étape 2 : Ajouter le Nœud](#étape-2--ajouter-le-nœud)
    - [Étape 3 : Mettre à Jour `package.xml`](#étape-3--mettre-à-jour-packagexml)
    - [Étape 4 : Mettre à Jour `setup.py`](#étape-4--mettre-à-jour-setuppy)
    - [Étape 5 : Construire le Package](#étape-5--construire-le-package)
    - [Étape 6 : Sourcer le workspace](#étape-6--sourcer-le-workspace)
    - [Étape 7 : Exécuter le Nœud](#étape-7--exécuter-le-nœud)
  - [4. Bonnes Pratiques pour les Nœuds ROS2](#4-bonnes-pratiques-pour-les-nœuds-ros2)
  - [5. Commandes Utiles](#5-commandes-utiles)
  - [Exercice : Créez un nœud `pid_regulator`](#exercice--créez-un-nœud-pid_regulator)
    - [Objectif](#objectif)
    - [Instructions](#instructions)

---

## 1. Introduction aux Nœuds ROS2

Un **nœud** dans ROS2 est une instance autonome qui :
- Communique avec d'autres nœuds via des **topics**, des **services**, ou des **actions**.
- Effectue des tâches spécifiques, comme la collecte de données depuis un capteur ou le contrôle d'un robot.

Les nœuds permettent de diviser une application complexe en plusieurs processus plus simples et indépendants. Cela facilite le développement, le débogage et la réutilisation des composants.

---

## 2. Créer un Nœud Simple

Un nœud ROS2 peut être créé en Python à l'aide du module `rclpy`. Voici un exemple de base pour créer un nœud.

### Exemple : Nœud Simple

**Code :**

```python
import rclpy
from rclpy.node import Node

class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')  # Nom du nœud
        self.get_logger().info('Le nœud est démarré !')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode()  # Création du nœud
    rclpy.spin(node)  # Exécution du nœud
    node.destroy_node()  # Destruction du nœud lors de l'arrêt
    rclpy.shutdown()  # Arrêt de ROS2

if __name__ == '__main__':
    main()
```

**Explications :**
- `Node` : Classe de base pour créer des nœuds ROS2.
- `get_logger()` : Utilisé pour afficher des messages dans la console.
- `rclpy.spin(node)` : Maintient le nœud actif jusqu'à son arrêt.

---

## 3. Lancer et Tester un Nœud

### Étape 1 : Créer un Package ROS2
Créez un package ROS2 si ce n'est pas encore fait :
```bash
ros2 pkg create --build-type ament_python <nom_du_package>
```

### Étape 2 : Ajouter le Nœud
Ajoutez votre script de nœud dans le dossier `/<nom_du_package>/<nom_du_package>/`.

### Étape 3 : Mettre à Jour `package.xml`
Modifiez le fichier `package.xml` pour définir les dépendances si nécessaire :
```xml
<!-- Après la balise <license>, ajoutez ceci : -->
<depend>rclpy</depend>
```

### Étape 4 : Mettre à Jour `setup.py`
Modifiez le fichier `setup.py` pour enregistrer le nœud :
```python
entry_points={
    'console_scripts': [
        '<nom_de_votre_node> = <nom_du_package>.<nom_fichier_py>:main',
    ],
},
```

### Étape 5 : Construire le Package
```bash
colcon build --symlink-install
```
L'option --symlink-install dans la commande colcon build est utilisée pour optimiser le processus de développement dans les workspaces ROS 2. Elle permet de créer des liens symboliques (symlinks) vers les fichiers d'installation, plutôt que de copier ces fichiers dans le répertoire install.
**Pas besoin de reconstruire le workspace après chaque modification dans les fichiers source.**
### Étape 6 : Sourcer le workspace 
```bash
source install/setup.bash
```

### Étape 7 : Exécuter le Nœud
```bash
ros2 run <nom_du_package> <nom_de_votre_node>
```

---

## 4. Bonnes Pratiques pour les Nœuds ROS2

1. **Utilisez des noms descriptifs** : Donnez des noms clairs à vos nœuds pour identifier leur rôle.
2. **Structurez le code** : Placez les Publishers, Subscribers et autres composants dans des méthodes distinctes.
3. **Gérez les exceptions** : Ajoutez des blocs `try` pour éviter les plantages inattendus.
4. **Évitez les boucles infinies** : Utilisez les Timers et `rclpy.spin()` pour gérer les tâches récurrentes.
5. **Respectez les conventions ROS2** : Utilisez les types de messages appropriés et documentez les topics.

---

## 5. Commandes Utiles

- **Lister les nœuds actifs** :
  ```bash
  ros2 node list
  ```

- **Obtenir des informations sur un nœud** :
  ```bash
  ros2 node info <nom_du_nœud>
  ```

---

## Exercice : Créez un nœud `pid_regulator`

### Objectif
Créer un nœud simple nommé `pid_regulator` dans le package `turtle_controller` situé dans le workspace `turtle_ws`. Ce nœud affichera uniquement un message dans la console indiquant qu'il a été correctement créé.

### Instructions

1. Assurez-vous d’avoir un workspace nommé `turtle_ws` contenant un package appelé `turtle_controller`. Si ce n’est pas encore fait, suivez les étapes précédentes pour créer un workspace et un package.

2. Naviguez dans le répertoire `src` du workspace `turtle_ws` :
   ```bash
   cd ~/turtle_ws/src/turtle_controller
   ```

3. Implémentez un nœud nommé `pid_regulator` dans ce package :
   - créez un fichier Python dans le dossier `turtle_controller` avec le contenu approprié pour un nœud ROS 2.

4. Le nœud doit afficher un simple message comme :
   ```
   [INFO] [pid_regulator]: Node pid_regulator has been created successfully!
   ```

5. Modifiez les fichiers nécessaires pour inclure ce nœud :
   - Mettez à jour le fichier `package.xml` si des dépendances supplémentaires sont nécessaires.
   - Mettez à jour le fichier  `setup.py` pour inclure ce nœud.

6. Compilez de nouveau le workspace pour inclure le nœud `pid_regulator` :
   ```bash
   cd ~/turtle_ws
   colcon build
   ```

7. Lancez le nœud avec la commande :
   ```bash
   ros2 run turtle_controller pid_regulator
   ```
