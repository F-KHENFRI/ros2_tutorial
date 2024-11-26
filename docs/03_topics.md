# Les Topics dans ROS2

Les **topics** sont un concept central dans ROS2 pour permettre aux nœuds de communiquer entre eux. Ils utilisent un modèle **Publisher/Subscriber** pour transmettre des données de manière asynchrone. Dans cette annexe, nous allons explorer en détail leur fonctionnement, les arguments nécessaires à leur configuration, et les bonnes pratiques pour leur utilisation.

---

## Sommaire

- [Les Topics dans ROS2](#les-topics-dans-ros2)
  - [Sommaire](#sommaire)
  - [1. Qu'est-ce qu'un Topic ?](#1-quest-ce-quun-topic-)
  - [2. Créer un Publisher pour un Topic](#2-créer-un-publisher-pour-un-topic)
    - [Code : Publisher Simple](#code--publisher-simple)
    - [Explication des Arguments de `create_publisher`](#explication-des-arguments-de-create_publisher)
  - [3. Créer un Subscriber pour un Topic](#3-créer-un-subscriber-pour-un-topic)
    - [Code : Subscriber Simple](#code--subscriber-simple)
    - [Explication des Arguments de `create_subscription`](#explication-des-arguments-de-create_subscription)
  - [4. Lister et Inspecter les Topics](#4-lister-et-inspecter-les-topics)
  - [5. Commandes Utiles pour les Topics](#5-commandes-utiles-pour-les-topics)
    - [Écouter les Messages](#écouter-les-messages)
    - [Publier un Message](#publier-un-message)
    - [Inspecter la Fréquence des Messages](#inspecter-la-fréquence-des-messages)
  - [6. Exemples de Communication](#6-exemples-de-communication)
  - [7. Meilleures Pratiques](#7-meilleures-pratiques)
  - [9. Conclusion](#9-conclusion)

---

## 1. Qu'est-ce qu'un Topic ?

Un **topic** est un canal de communication dans ROS2. Il permet :
- À un **Publisher** d'envoyer des messages.
- À un ou plusieurs **Subscribers** d'écouter ces messages.

Chaque topic est identifié par un nom unique (ex. : `/cmd_vel`, `/sensor_data`) et a un type de message associé (ex. : `std_msgs/String`, `geometry_msgs/Twist`).

Caractéristiques principales :
- **Asynchrone** : Les Publishers et Subscribers fonctionnent indépendamment.
- **Multipoint** : Un topic peut avoir plusieurs Publishers et Subscribers simultanés.
- **Type strict** : Tous les messages sur un topic doivent être du même type.

---

## 2. Créer un Publisher pour un Topic

Un Publisher permet d'envoyer des messages sur un topic. Voici un exemple :

### Code : Publisher Simple
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')  # Nom du nœud
         # Crée un Publisher
        self.publisher = self.create_publisher(String, 'chatter', 10) 
        # Timer pour publier périodiquement
        self.timer = self.create_timer(1.0, self.publish_message)  

    def publish_message(self):
        msg = String()
        msg.data = "Bonjour depuis ROS2 !"  # Contenu du message
        self.publisher.publish(msg)  # Publie le message
        self.get_logger().info(f'Publié : "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### Explication des Arguments de `create_publisher`

```python
self.create_publisher(message_type, topic_name, qos)
```

- **`message_type`** : Le type de message publié. Vous pouvez importer ces types depuis les bibliothèques ROS2 standard, comme `std_msgs.msg` ou `geometry_msgs.msg`. Par exemple :
  - `std_msgs/String` : Chaînes de caractères.
  - `geometry_msgs/Twist` : Commandes de vitesse.

- **`topic_name`** : Le nom du topic sur lequel le message sera publié. Assurez-vous qu'il est unique et descriptif, comme `/robot/cmd_vel`.

- **`qos`** : Le profil de qualité de service (QoS). Nous explorerons cette notion plus loin.

---

## 3. Créer un Subscriber pour un Topic

Un Subscriber permet de recevoir des messages publiés sur un topic.

### Code : Subscriber Simple
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')  # Nom du nœud
        self.subscription = self.create_subscription(
            String,  # Type du message
            'chatter',  # Nom du topic
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Reçu : "{msg.data}"')  # Affiche le message reçu

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### Explication des Arguments de `create_subscription`

```python
self.create_subscription(message_type, topic_name, callback, qos)
```

- **`message_type`** : Le type du message que le Subscriber attend. Il doit correspondre au type utilisé par le Publisher. Exemples :
  - `std_msgs/String` pour les chaînes de caractères.
  - `sensor_msgs/Image` pour des images.

- **`topic_name`** : Le nom du topic à écouter. Assurez-vous qu'il correspond exactement au nom utilisé par le Publisher.

- **`callback`** : Une fonction appelée chaque fois qu'un message est reçu. Elle doit prendre un seul argument, le message reçu.

- **`qos`** : Le profil de qualité de service, qui contrôle la fiabilité et les délais des messages.

---

## 4. Lister et Inspecter les Topics

- **Lister tous les topics actifs :**
  ```bash
  ros2 topic list
  ```

- **Afficher le type de message utilisé par un topic :**
  ```bash
  ros2 topic type /chatter
  ```

- **Écouter les messages publiés sur un topic :**
  ```bash
  ros2 topic echo /chatter
  ```

- **Publier un message manuel sur un topic :**
  ```bash
  ros2 topic pub /chatter std_msgs/String '{data: "Hello ROS2"}'
  ```

---

## 5. Commandes Utiles pour les Topics

### Écouter les Messages
```bash
ros2 topic echo /nom_du_topic
```

### Publier un Message
```bash
ros2 topic pub /nom_du_topic std_msgs/String '{data: "Hello ROS2"}'
```

### Inspecter la Fréquence des Messages
```bash
ros2 topic hz /nom_du_topic
```

---

## 6. Exemples de Communication

1. **Lancer un Publisher :**
   ```bash
   ros2 run my_package publisher_node
   ```
2. **Lancer un Subscriber :**
   ```bash
   ros2 run my_package subscriber_node
   ```
3. **Observez les messages avec `echo` :**
   ```bash
   ros2 topic echo /chatter
   ```

---

## 7. Meilleures Pratiques

1. **Nommez vos topics de manière descriptive :**
   - Préfixez les noms des topics pour mieux organiser le système. Par exemple : `/robot/cmd_vel`.

2. **Optimisez la fréquence de publication :**
   - Évitez de publier trop fréquemment, sauf si nécessaire.

3. **Testez avec `ros2 topic` :**
   - Avant de connecter les nœuds, utilisez les commandes ROS2 pour vérifier les messages publiés.

---

## 9. Conclusion

Les **topics** sont un outil puissant pour structurer la communication dans ROS2. Ils permettent un échange de données asynchrone et flexible, essentiel pour la conception de systèmes robotiques modulaires. Avec une configuration bien pensée des **arguments** (comme le type de message et les profils QoS pour des applications avancées) et en suivant des pratiques optimales, les topics peuvent être adaptés à une grande variété de scénarios, des flux de données en temps réel aux commandes critiques.

Les topics offrent :
- Une flexibilité pour connecter plusieurs nœuds à travers des **Publishers** et **Subscribers**.
- Des options avancées comme la **Qualité de Service (QoS)** pour répondre aux besoins spécifiques des applications.
- Une facilité de test et de diagnostic grâce aux commandes de ROS2.

En maîtrisant les concepts et outils associés aux topics, vous pouvez concevoir des systèmes robustes et évolutifs pour répondre aux besoins complexes de vos projets robotiques. Que ce soit pour collecter des données capteurs, envoyer des commandes à des robots ou coordonner plusieurs composants, les topics sont la pierre angulaire d'une communication efficace dans ROS2.

