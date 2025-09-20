#!/bin/bash
set -e

echo "[🛠] Initialisation de l'environnement ROS 2..."

# Vérifie que DISPLAY est bien défini
if [ -z "$DISPLAY" ]; then
  export DISPLAY=:0
  echo "[⚠️] DISPLAY non défini, utilisation de :0 par défaut"
fi

# Autorise l'accès X11 côté hôte si possible (optionnel si lancé manuellement)
if command -v xhost &> /dev/null; then
  echo "[🔓] Autorisation X11..."
  xhost +local:root
fi

# Setup ROS 2 et build du workspace
cd /home/ros2_ws/
source /opt/ros/humble/setup.bash

echo "[🔧] Compilation du workspace ROS 2..."
colcon build

source /home/ros2_ws/install/setup.bash

# Exécution du nœud principal
echo "[🚀] Lancement du nœud controller_node..."
ros2 run controller controller_node
