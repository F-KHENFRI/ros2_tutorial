# **Concept des Services dans ROS 2**
## **Sommaire**
- [**Concept des Services dans ROS 2**](#concept-des-services-dans-ros-2)
  - [**Sommaire**](#sommaire)
  - [**Qu’est-ce qu’un Service dans ROS 2 ?**](#quest-ce-quun-service-dans-ros-2-)
    - [**Caractéristiques des Services :**](#caractéristiques-des-services-)
  - [**Structure d’un Service**](#structure-dun-service)
    - [**Exemple d’un fichier `.srv` :**](#exemple-dun-fichier-srv-)
  - [**Utilisation des Services dans un Nœud ROS 2**](#utilisation-des-services-dans-un-nœud-ros-2)
    - [**1. Implémenter un Serveur de Service**](#1-implémenter-un-serveur-de-service)
      - [Exemple de Serveur :](#exemple-de-serveur-)
    - [**2. Implémenter un Client de Service**](#2-implémenter-un-client-de-service)
      - [Exemple de Client :](#exemple-de-client-)
  - [**Utilisation des Services avec la CLI**](#utilisation-des-services-avec-la-cli)
    - [**Commandes CLI principales :**](#commandes-cli-principales-)
  - [**Exercice : Modifier les coordonnées cibles dans le nœud `pid_regulator` via un Service**](#exercice--modifier-les-coordonnées-cibles-dans-le-nœud-pid_regulator-via-un-service)
    - [**Objectif**](#objectif)
    - [**Instructions**](#instructions)

---

## **Qu’est-ce qu’un Service dans ROS 2 ?**

Un **service** dans ROS 2 est un mécanisme de communication bidirectionnelle entre deux nœuds, basé sur une interaction **requête-réponse**. Contrairement aux topics qui fonctionnent en mode asynchrone (pub/sub), les services permettent à un client de demander une action spécifique à un serveur et de recevoir une réponse immédiate.

### **Caractéristiques des Services :**
1. **Synchrones** : Le client attend une réponse après avoir envoyé une requête.
2. **Typés** : Les services utilisent une interface définie contenant :
   - Les champs pour la **requête**.
   - Les champs pour la **réponse**.
3. **Durée de vie limitée** : La communication existe uniquement pendant l’échange.

---

## **Structure d’un Service**

Les services sont définis par des fichiers `.srv` dans ROS 2, similaires aux fichiers `.msg` pour les messages.

### **Exemple d’un fichier `.srv` :**
Voici un service nommé `SetTarget` utilisé pour définir une position cible.

```srv
float64 x         # Position X cible
float64 y         # Position Y cible
float64 theta     # Angle cible
---
bool success      # Indique si l'opération a réussi
string message    # Message de retour
```

- La partie avant `---` définit les champs de la **requête**.
- La partie après `---` définit les champs de la **réponse**.

---

## **Utilisation des Services dans un Nœud ROS 2**

Un nœud peut être configuré pour agir en tant que **client** ou **serveur** de service.

### **1. Implémenter un Serveur de Service**
Le serveur gère les requêtes reçues et retourne une réponse.

#### Exemple de Serveur :
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool  # Exemple de service prédéfini

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')

        # Créer le serveur de service
        self.service = self.create_service(SetBool, 'toggle_service', self.handle_request)
        self.get_logger().info('Service toggle_service is ready.')

    def handle_request(self, request, response):
        # Logique pour traiter la requête
        if request.data:  # Accéder au champ `data` de la requête
            response.success = True
            response.message = 'Service activated!'
        else:
            response.success = False
            response.message = 'Service deactivated!'

        return response

def main(args=None):
    rclpy.init(args=args)
    node = ServiceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### **2. Implémenter un Client de Service**
Le client envoie une requête au serveur et attend une réponse.

#### Exemple de Client :
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')
        self.client = self.create_client(SetBool, 'toggle_service')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.get_logger().info('Service available. Sending request...')
        self.send_request(data=True)

    def send_request(self, data):
        request = TeleportAbsolute.Request()
        request.data = data
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Response: success = {response.success}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ClientNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```
---

## **Utilisation des Services avec la CLI**

ROS 2 propose des commandes pour interagir avec les services disponibles.

### **Commandes CLI principales :**
1. **Lister les services actifs :**
   ```bash
   ros2 service list
   ```

2. **Afficher le type d’un service :**
   ```bash
   ros2 service type /service_name
   ```

3. **Afficher les détails d’un type de service :**
   ```bash
   ros2 interface show <service_type>
   ```
   Exemple :
   ```bash
   ros2 interface show example_interfaces/srv/SetBool
   ```

4. **Appeler un service :**
   ```bash
   ros2 service call /service_name <service_type> <arguments>
   ```
   Exemple :
   ```bash
   ros2 service call /toggle_service example_interfaces/srv/SetBool "{data: true}"
   ```

---

## **Exercice : Modifier les coordonnées cibles dans le nœud `pid_regulator` via un Service**

### **Objectif**
Ajouter un service dans le nœud `pid_regulator` pour permettre la modification des coordonnées cibles (**x**, **y**) et de l’angle **θ** via une requête. 

### **Instructions**
1. **Ajoutez un serveur de service dans `pid_regulator` :**
   - Créez le serveur `/turtle1/set_target_pose`de service pour gérer les requêtes entrantes avec l'interface service  `turtlesim/srv/TeleportAbsolute`.
   - Mettez à jour les cibles **x_d**, **y_d**, et **θ_d** lorsque la requête est reçue.

3. **Testez le service avec la CLI :**
   - Appelez le service avec différentes coordonnées et observez les résultats dans le comportement de la tortue.

   Exemple d’appel CLI :
   ```bash
   ros2 service call /turtle1/set_target_pose turtlesim/srv/TeleportAbsolute "{x: 8.0, y: 8.0, theta: 1.57}"
   ```
4. **Ajoutez le nœud client `turtle_navigation` :**  
   - Créez un nouveau nœud client nommé `turtle_navigation`, qui effectue la planification de trajectoire et envoie des requêtes au nœud `pid_regulator`.  
   - Testez d'abord avec une seule requête, puis avec une série de requêtes où la suivante est envoyée uniquement lorsque la précédente a été correctement exécutée.
   - Que remarquez-vous, et quelle solution proposez-vous ?
---
