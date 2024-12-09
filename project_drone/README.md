# **Développement d'un Pont ROS pour le Contrôle du Drone Tello avec DJITelloPy**

## **Introduction :**  
Les drones Tello, produits par Ryze Robotics en collaboration avec DJI, sont largement utilisés dans l'éducation et la recherche grâce à leur coût abordable et leurs capacités polyvalentes. Cependant, leur intégration dans des environnements robotiques avancés, tels que ceux utilisant le Robot Operating System (ROS), nécessite une interface robuste et personnalisable.

##  **Objectif du projet :**  
L’objectif principal de ce projet est de développer un pont (bridge) ROS permettant de contrôler et de surveiller le drone Tello à l’aide de la bibliothèque DJITelloPy. Le pont doit permettre l'intégration transparente de ce drone dans un environnement ROS, en facilitant la communication entre le drone et les autres composants du système (capteurs, planificateurs de mission, etc.).

##  **Éléments clés du projet :**  

1. **Recherche et Analyse :**  
   - Étudier les capacités et limitations du drone Tello.
   - Comprendre l’architecture et les fonctionnalités de DJITelloPy.

2. **Développement du Pont ROS :**  
   - Créer des nœuds ROS pour piloter le drone (décollage, atterrissage, navigation, etc.).
   - Développer des messages et services ROS pour la communication avec le drone.
   - Intégrer les flux de données des capteurs du drone (caméra, altimètre, etc.) dans le système ROS.

3. **Tests et Validation :**  
   - Tester le pont avec des commandes de base (déplacement, capture d’images/vidéos).
   - Valider la performance et la latence dans un environnement ROS.

4. **Documentation et Guide d'Utilisation :**  
   - Fournir une documentation claire pour l’installation et l’utilisation du pont.
   - Inclure des exemples pratiques et des cas d’utilisation.

##  **Résultat attendu :**  
Un système fonctionnel permettant de contrôler le drone Tello via ROS, en tirant parti de la simplicité et de la flexibilité de DJITelloPy, avec des applications potentielles dans l’éducation, la recherche et le prototypage.

##  **Technologies utilisées :**  
- Langage : Python (pour DJITelloPy et ROS bindings)
- Environnement : ROS2, Ubuntu
- Outils de développement : VSCode et simulateur Unity Tello téléchargabe avec le lien suivant https://github.com/PYBrulin/UAV-Tello-Simulator
  
## **Livrables :**  
- Code source du pont ROS.
- Documentation technique et guide d’utilisateur.
- Rapport de projet décrivant le processus de développement et les résultats obtenus.

