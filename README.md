# ROS 2 Tutorial

Bienvenue dans ce tutorial **ROS 2** ! Ce dépôt contient tout les informations nécessaires pour réaliser un projet sous ROS 2.

---

## Objectifs du cours

- **Apprendre les bases de ROS 2** : exploration des nœuds, des topics et des messages.
- **Comprendre les concepts avancés** : gestion des services, actions, et paramètres.
- **Développer un projet final** : mise en œuvre d'une navigation autonome.

---

## Prérequis

- **Système d'exploitation** :
  - **Ubuntu 22.04** (recommandé).
  - **Windows 10/11** (via WSL2 et Docker NVIDIA).
  
- **Logiciels requis** :
  - **Git**
  - **Docker** avec prise en charge de NVIDIA (si GPU disponible).
  - **WSL2 et X11 Server** (pour Windows uniquement).
  - **Visual Studio Code** avec l'extension **Remote - Containers**.

---

## Structure du dépôt

- 📂 **`docs/`** : Documentation détaillée du cours avec des exercices.
- 📂 **`docker/`** : Fichiers de configuration Docker pour ROS 2.
- 📂 **`.devcontainer/`** : Configurations pour Visual Studio Code.

---

## Instructions d'installation

### Sous Windows

1. **Configurer X11 Server** :
   - Installez un serveur X11 comme [VcXsrv](https://sourceforge.net/projects/vcxsrv/) ou [Xming](https://sourceforge.net/projects/xming/).
   - Lancez X11 Server avec la configuration par défaut (aucune modification requise).

2. **Cloner le dépôt** :
   Ouvrez un terminal (PowerShell ou cmd) et exécutez la commande suivante :
   ```bash
   git clone https://github.com/F-KHENFRI/ros2_tutorial.git -b win_env
   cd ros2_tutorial
   ```

3. **Ouvrir le projet dans VSCode** :
   - Lancez Visual Studio Code.
   - Ouvrez le dossier **ros2_tutorial**.
   - Ouvrez la *Command Palette* (`Ctrl+Shift+P`), recherchez **"Rebuild and Reopen in Container"**, et appuyez sur **Entrée**.

4. **Tester l'environnement** :
   - Depuis le terminal intégré dans VSCode, testez une application graphique comme suit :
     ```bash
     gedit
     ```
   - L'application GUI devrait s'afficher.

---

### Sous Ubuntu

1. **Cloner le projet** :
   Ouvrez un terminal et exécutez :
   ```bash
   git clone https://github.com/F-KHENFRI/ros2_tutorial.git
   cd ros2_tutorial
   ```

2. **Configurer Docker NVIDIA** :
   - Rendez le script de configuration exécutable :
     ```bash
     chmod +x ./config_nvidia_docker.sh
     ```
   - Exécutez le script :
     ```bash
     ./config_nvidia_docker.sh
     ```

3. **Ouvrir le projet dans VSCode** :
   - Lancez Visual Studio Code et ouvrez le dossier contenant ce projet. Vous pouvez aussi utiliser la commande suivante :
     ```bash
     code .
     ```
   - Si VSCode détecte automatiquement le fichier **`.devcontainer/devcontainer.json`**, une notification apparaîtra pour **Reopen in Container**. Si ce n'est pas le cas :
     - Ouvrez la *Command Palette* (`Ctrl+Shift+P`).
     - Recherchez **"Rebuild and Reopen in Container"** et appuyez sur **Entrée**.

4. **Tester l'environnement** :
   - Depuis le terminal intégré dans VSCode, testez une application graphique comme suit :
     ```bash
     gedit
     ```
   - L'application GUI devrait s'afficher.

---

## Résolution des problèmes

### 1. Erreur "Cannot open display"
- Assurez-vous que X11 est configuré correctement :
  ```bash
  xhost +local:
  ```
- Vérifiez que la variable `DISPLAY` est définie :
  ```bash
  echo $DISPLAY
  ```
  Par défaut, cela devrait être `:0`.

### 2. Problèmes de permissions Docker
- Assurez-vous que votre utilisateur est dans le groupe Docker :
  ```bash
  sudo usermod -aG docker $USER
  ```
- Déconnectez-vous et reconnectez-vous pour appliquer les changements.



