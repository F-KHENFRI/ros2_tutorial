# **Concept des Paramètres dans ROS 2**

## **Sommaire**
- [**Concept des Paramètres dans ROS 2**](#concept-des-paramètres-dans-ros-2)
  - [**Sommaire**](#sommaire)
  - [**1. Qu’est-ce qu’un paramètre ?**](#1-quest-ce-quun-paramètre-)
    - [**Cas d’utilisation courants :**](#cas-dutilisation-courants-)
    - [**Méthodes de définition des paramètres :**](#méthodes-de-définition-des-paramètres-)
  - [**2. Types de paramètres acceptés dans ROS 2**](#2-types-de-paramètres-acceptés-dans-ros-2)
  - [**3. Utilisation des paramètres dans un nœud ROS 2**](#3-utilisation-des-paramètres-dans-un-nœud-ros-2)
    - [**a. Déclaration des paramètres**](#a-déclaration-des-paramètres)
    - [**b. Récupération des valeurs**](#b-récupération-des-valeurs)
    - [**c. Exemple d’utilisation de différents types**](#c-exemple-dutilisation-de-différents-types)
  - [**4. Utilisation des paramètres via les commandes CLI**](#4-utilisation-des-paramètres-via-les-commandes-cli)
    - [**Commandes CLI principales**](#commandes-cli-principales)
  - [**5. Exercice : Ajouter des paramètres de PID pour le nœud `pid_regulator`**](#5-exercice--ajouter-des-paramètres-de-pid-pour-le-nœud-pid_regulator)
    - [**Objectif**](#objectif)
    - [**Instructions**](#instructions)


--- 

## **1. Qu’est-ce qu’un paramètre ?**

Un paramètre dans **ROS 2** est une valeur configurable utilisée pour adapter dynamiquement le comportement d’un nœud sans recompilation ni modification du code. Cela permet de personnaliser et d’ajuster les fonctionnalités de manière flexible.

### **Cas d’utilisation courants :**
- Définir des constantes dans des algorithmes (ex. : gains PID, seuils, limites de vitesse).
- Configurer des chemins, adresses réseau ou autres variables statiques.
- Modifier dynamiquement des objectifs ou des cibles dans des applications robotiques.

### **Méthodes de définition des paramètres :**
1. **Fichier de configuration YAML** : Chargement au démarrage.
2. **CLI ROS 2** : Modification dynamique via des commandes.
3. **Directement dans le code** : Paramètres déclarés et définis dans le nœud.

---

## **2. Types de paramètres acceptés dans ROS 2**

ROS 2 supporte différents types de paramètres, y compris des structures simples et des collections :
1. **Types primitifs** :
   - `bool` : `True` ou `False`.
   - `int` : Nombres entiers.
   - `float` : Nombres à virgule flottante.
   - `string` : Chaînes de caractères.

2. **Collections** :
   - **Listes** : Ensemble d’éléments du même type (ex. `[1.0, 0.5, 0.1]`).
   - **Structures imbriquées (via YAML)** : Simulées comme des dictionnaires dans des fichiers YAML.

---

## **3. Utilisation des paramètres dans un nœud ROS 2**

### **a. Déclaration des paramètres**
Avant de les utiliser, chaque paramètre doit être déclaré dans le nœud :
```python
self.declare_parameter('param_name', default_value)
```

### **b. Récupération des valeurs**
Une fois déclarés, les paramètres peuvent être lus comme suit :
```python
param_value = self.get_parameter('param_name').value
```

### **c. Exemple d’utilisation de différents types**
Voici un exemple intégrant plusieurs types de paramètres dans un nœud ROS 2 :
```python
import rclpy
from rclpy.node import Node

class ParameterExampleNode(Node):
    def __init__(self):
        super().__init__('parameter_example_node')

        # Déclaration de différents types de paramètres
        self.declare_parameter('bool_param', True)
        self.declare_parameter('int_param', 42)
        self.declare_parameter('float_param', 3.14)
        self.declare_parameter('string_param', 'Hello ROS 2')
        self.declare_parameter('list_param', [1.0, 0.5, 0.1])

        # Lecture des paramètres
        bool_value = self.get_parameter('bool_param').value
        int_value = self.get_parameter('int_param').value
        float_value = self.get_parameter('float_param').value
        string_value = self.get_parameter('string_param').value
        list_value = self.get_parameter('list_param').value

        self.get_logger().info(f'Parameters: {bool_value}, {int_value}, {float_value}, {string_value}, {list_value}')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterExampleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## **4. Utilisation des paramètres via les commandes CLI**

### **Commandes CLI principales**
1. **Afficher tous les paramètres d’un nœud** :
   ```bash
   ros2 param list /node_name
   ```
2. **Définir un paramètre** :
   ```bash
   ros2 param set /node_name param_name value
   ```
3. **Récupérer la valeur d’un paramètre** :
   ```bash
   ros2 param get /node_name param_name
   ```
4. **Exporter les paramètres d’un nœud vers un fichier YAML** :
   ```bash
   ros2 param dump /node_name --output-dir /path/to/save/
   ```
5. **Charger un fichier YAML contenant des paramètres** :
   ```bash
   ros2 param load /node_name file.yaml
   ```
---

## **5. Exercice : Ajouter des paramètres de PID pour le nœud `pid_regulator`**

### **Objectif**
Ajouter des paramètres pour définir les gains PID de la vitesse linéaire (**Kp_lin**, **Ki_lin**, **Kd_lin**) et angulaire (**Kp_ang**, **Ki_ang**, **Kd_ang**) dans le nœud `pid_regulator`.

### **Instructions**
1. **Déclarez les paramètres :**
   - Pour les gains PID de la vitesse linéaire : `Kp_lin`, `Ki_lin`, `Kd_lin`.
   - Pour les gains PID de la vitesse angulaire : `Kp_ang`, `Ki_ang`, `Kd_ang`.

2. **Implémentez la récupération des paramètres :**
   - Lisez les valeurs des paramètres pour les utiliser dans les calculs de régulation PID.

3. **Ajoutez la gestion dynamique :**
   - Configurez un callback pour réagir aux modifications des paramètres à l’exécution.

4. **Testez avec la CLI :**
   - Changez les valeurs des gains et observez leur effet :
     ```bash
     ros2 param set /pid_regulator Kp_lin 1.5
     ros2 param set /pid_regulator Ki_ang 0.1
     ```

