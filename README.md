dadou_control_ros
=================

Contrôleur ROS2 et outils associés pour piloter le robot (interfaces d’entrée, mapping manette USB, configuration, UI, logs, etc.). Ce dépôt contient un module Python `controller` ainsi qu’un workspace ROS2 minimal (`ros2_ws`).

Fonctionnalités
---------------
- Entrées: gestion d’une manette USB via `pygame` (`controller/input/usb_gamepad.py`).
- Configuration centralisée: `controller/control_config.py` (chemins, logs, playlists, mappings…).
- Tests unitaires: `unittest` dans `controller/tests`.
- Intégration ROS2: workspace `ros2_ws` (sources côté contrôleur/robot).

Prérequis
---------
- Python 3.11+
- Virtualenv recommandé
- (Optionnel) ROS2 installé pour l’exécution complète côté robot

Installation rapide
-------------------
```
# 1) Cloner le dépôt
# git clone https://github.com/<votre-org>/dadou_control_ros.git
cd dadou_control_ros

# 2) Créer un environnement virtuel
python3 -m venv venv
source venv/bin/activate

# 3) Installer les dépendances
pip install -U pip
pip install -r requirements.txt
```

Exécuter les tests
------------------
- Tous les tests:
```
python -m unittest -v -s controller/tests -p "test_*.py"
```
- Une classe précise:
```
python -m unittest -v controller.tests.test_usb_gamepad.TestUSBGamePad
```
- Une méthode précise:
```
python -m unittest -v controller.tests.test_usb_gamepad.TestUSBGamePad.test_check
```

Astuce VS Code: la configuration est fournie dans `.vscode/settings.json` (découverte `unittest` pointant sur `controller/tests`, top-level sur la racine du repo).

Utilisation (manette USB)
-------------------------
Le module `USBGamepad` initialisera `pygame` et listera les contrôleurs disponibles. En l’absence de manette, il reste silencieux et ne génère pas d’erreur bloquante, ce qui permet d’exécuter les tests en environnement CI/headless.

Fichier principal: `controller/input/usb_gamepad.py`.

Structure du projet
-------------------
- `controller/`: code source Python
  - `input/usb_gamepad.py`: gestion manette USB via `pygame`
  - `control_config.py`: configuration du contrôleur (chemins, logs…)
  - `tests/`: tests unitaires
- `ros2_ws/`: workspace ROS2 (sources complémentaires)
- `conf/`, `json/`, `medias/`: assets et configs
- `test_logs/`: dossier cible pour les logs de test

Développement
-------------
- Format: suivre le style existant.
- Tests: ajouter des tests dans `controller/tests`.
- Logs: les tests écrivent dans `test_logs/controller-test.log` (évite les permissions système).

Contributions
-------------
- Issues et PR bienvenues. Merci de décrire clairement le contexte et la reproduction.

Licence
-------
- À définir par le propriétaire du dépôt.
