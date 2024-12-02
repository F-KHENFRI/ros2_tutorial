# **Création des Interfaces personalisées** 

Les **interfaces dans ROS 2** sont des structures standardisées qui définissent la manière dont les données sont échangées entre les différentes parties d’un système ROS 2, comme les nœuds ou les services. Elles permettent de spécifier le contenu et le format des messages, services, et actions utilisés dans les communications.  

- [**Création des Interfaces personalisées**](#création-des-interfaces-personalisées)
  - [1. **Types d’interfaces en ROS 2**](#1-types-dinterfaces-en-ros-2)
  - [2. **Pourquoi les interfaces sont importantes dans ROS 2 ?**](#2-pourquoi-les-interfaces-sont-importantes-dans-ros-2-)
  - [3. **Résumé des différences entre les types d’interfaces**](#3-résumé-des-différences-entre-les-types-dinterfaces)
  - [4. **Création des nouvelles interfaces**](#4-création-des-nouvelles-interfaces)
    - [1. **Créer le dossier du package**](#1-créer-le-dossier-du-package)
    - [2. **Créer les fichiers d'interface**](#2-créer-les-fichiers-dinterface)
      - [a. **Créer un fichier `.msg`**](#a-créer-un-fichier-msg)
      - [b. **Créer un fichier `.srv`**](#b-créer-un-fichier-srv)
      - [c. **Créer un fichier `.action`**](#c-créer-un-fichier-action)
    - [3. **Mettre à jour le fichier `CMakeLists.txt`**](#3-mettre-à-jour-le-fichier-cmakeliststxt)
    - [4. **Mettre à jour `package.xml`**](#4-mettre-à-jour-packagexml)
    - [5. **Compiler le package**](#5-compiler-le-package)
    - [6. **Utilisation dans d'autres packages**](#6-utilisation-dans-dautres-packages)
    - [7. **Vérification**](#7-vérification)

---

## 1. **Types d’interfaces en ROS 2**
1. **Messages (`msg`)**
   - Les messages sont utilisés pour la communication **asynchrone** entre les nœuds via des **topics**.
   - Un message est une structure de données qui contient des champs avec des types spécifiques (par exemple, entiers, chaînes, tableaux, etc.).
   - Exemple : un message pour transmettre la position d’un robot pourrait inclure des champs comme `float64 x`, `float64 y`, `float64 theta`.

   **Exemple :**
   ```plaintext
   msg/Position.msg
   float64 x
   float64 y
   float64 theta
   ```

2. **Services (`srv`)**
   - Les services permettent une communication **synchrone** entre les nœuds sous la forme d'une **requête-réponse**.
   - Un service est défini par deux parties : une requête (les données envoyées au service) et une réponse (les données renvoyées par le service).
   - Utilisation typique : demander un calcul, déclencher une action unique, ou récupérer des informations.

   **Exemple :**
   ```plaintext
   srv/ComputePath.srv
   # Requête
   float64 start_x
   float64 start_y
   float64 goal_x
   float64 goal_y
   ---
   # Réponse
   float64[] path_x
   float64[] path_y
   ```

3. **Actions (`action`)**
   - Les actions permettent de gérer des tâches **longues ou complexes** en fournissant une structure qui inclut un objectif (goal), un retour d’état (feedback), et un résultat final.
   - Les actions sont particulièrement utiles pour les opérations qui peuvent prendre du temps, comme déplacer un bras robotique ou exécuter une trajectoire.

   **Exemple :**
   ```plaintext
   action/Move.action
   # Goal
   float64 target_x
   float64 target_y
   float64 target_theta
   ---
   # Feedback
   float64 current_x
   float64 current_y
   float64 progress
   ---
   # Result
   bool success
   string message
   ```

---

## 2. **Pourquoi les interfaces sont importantes dans ROS 2 ?**
1. **Standardisation :** Elles garantissent que les données échangées entre les nœuds sont bien définies et comprises par tous les participants.
2. **Interopérabilité :** Les interfaces permettent à des nœuds développés indépendamment de communiquer sans ambiguïté.
3. **Flexibilité :** En utilisant des messages, services, ou actions, vous pouvez choisir le mode de communication adapté aux besoins de votre application.
4. **Modularité :** Les interfaces rendent les systèmes ROS 2 modulaires et faciles à maintenir.

---

## 3. **Résumé des différences entre les types d’interfaces**
| Type      | Utilisation principale                   | Communication        | Exemple typique                      |
|-----------|------------------------------------------|----------------------|---------------------------------------|
| **`msg`** | Transfert de données entre nœuds         | Asynchrone (topics)  | Position d’un robot                  |
| **`srv`** | Requêtes ponctuelles avec une réponse    | Synchrone            | Calculer une trajectoire             |
| **`action`** | Tâches longues avec état intermédiaire | Asynchrone avec suivi| Déplacer un bras robotique vers un point |


---
## 4. **Création des nouvelles interfaces**
Consultez les interfaces communes de ROS 2 avant de créer une nouvelle interface : https://github.com/ros2/common_interfaces

### 1. **Créer le dossier du package**
Commencez par créer un package ROS 2 qui contiendra vos interfaces. Utilisez la commande suivante :

```bash
ros2 pkg create my_interfaces --build-type ament_cmake
```

---

### 2. **Créer les fichiers d'interface**
Dans le dossier de votre package (`my_interfaces`), créez des sous-dossiers pour stocker vos fichiers `msg`, `srv`, et `action` :

```bash
cd my_interfaces
mkdir msg srv action
```

#### a. **Créer un fichier `.msg`**
Un fichier `msg` définit les données échangées dans les messages ROS. Exemple : un message contenant un entier et un booléen.

Fichier : `msg/MyMessage.msg`
```plaintext
int32 my_number
bool my_flag
```

#### b. **Créer un fichier `.srv`**
Un fichier `srv` définit une requête et une réponse pour un service.

Fichier : `srv/MyService.srv`
```plaintext
# Requête
string request_data
---
# Réponse
bool success
string response_message
```

#### c. **Créer un fichier `.action`**
Un fichier `action` définit un objectif, un retour d'état, et un résultat pour une action.

Fichier : `action/MyAction.action`
```plaintext
# Goal
string target_name
int32 repetitions
---
# Feedback
float32 progress
string current_status
---
# Result
bool success
string result_summary
```

---

### 3. **Mettre à jour le fichier `CMakeLists.txt`**
Ajoutez les sections nécessaires pour inclure les fichiers d'interface :

Dans `CMakeLists.txt` :
```cmake
find_package(rosidl_default_generators REQUIRED)

# Déclarer les fichiers d'interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MyMessage.msg"
  "srv/MyService.srv"
  "action/MyAction.action"
)

# Ajouter la dépendance pour les messages
ament_export_dependencies(rosidl_default_runtime)
```

---

### 4. **Mettre à jour `package.xml`**
Ajoutez les dépendances nécessaires pour générer et utiliser les interfaces :

Dans `package.xml` :
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

---

### 5. **Compiler le package**
Compilez le package pour générer les interfaces :

```bash
colcon build --packages-select my_interfaces
```
N.B: il faut sourcer le fichier de configuration `setup.bash` de le workspace

---

### 6. **Utilisation dans d'autres packages**
Dans un autre package, ajoutez une dépendance vers `my_interfaces` et importez les interfaces :

- **`CMakeLists.txt`** :
```cmake
find_package(my_interfaces REQUIRED)
```

- **Code Python** :
```python
from my_interfaces.msg import MyMessage
from my_interfaces.srv import MyService
from my_interfaces.action import MyAction
```

---

### 7. **Vérification**
- Pour lister les messages générés :  
  ```bash
  ros2 interface show my_interfaces/msg/MyMessage
  ```
- Pour les services :  
  ```bash
  ros2 interface show my_interfaces/srv/MyService
  ```
- Pour les actions :  
  ```bash
  ros2 interface show my_interfaces/action/MyAction
  ```
