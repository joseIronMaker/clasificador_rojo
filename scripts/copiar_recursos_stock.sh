#!/usr/bin/env bash
# Copia los recursos stock (UR5e + MoveIt, TurtleBot + Nav2) al paquete clasificador_rojo.
# Ejecutar UNA VEZ tras instalar ros-jazzy-webots-ros2-universal-robot / -turtlebot / -moveit.
#   bash scripts/copiar_recursos_stock.sh
set -euo pipefail

PKG="$(cd "$(dirname "$0")/.." && pwd)"
UR="/opt/ros/jazzy/share/webots_ros2_universal_robot"
TB="/opt/ros/jazzy/share/webots_ros2_turtlebot"

echo "Paquete:   $PKG"
echo "UR (stock): $UR"
echo "TB (stock): $TB"
mkdir -p "$PKG/urdf" "$PKG/config"

copia() {  # copia $1 -> $2 si existe el origen
  if [[ -f "$1" ]]; then cp -v "$1" "$2"; else echo "  (falta) $1"; fi
}

echo "== UR5e + MoveIt =="
copia "$UR/resource/ur5e_with_gripper.urdf"        "$PKG/urdf/ur5e_with_gripper.urdf"
copia "$UR/resource/ros2_control_config.yaml"      "$PKG/config/ros2_control_ur5e.yaml"
copia "$UR/resource/moveit_ur5e.srdf"              "$PKG/config/moveit_ur5e.srdf"
copia "$UR/resource/moveit_kinematics.yaml"        "$PKG/config/moveit_kinematics.yaml"
copia "$UR/resource/moveit_movegroup.yaml"         "$PKG/config/moveit_movegroup.yaml"
copia "$UR/resource/moveit_controllers.yaml"       "$PKG/config/moveit_controllers.yaml"

echo "== TurtleBot + Nav2 =="
copia "$TB/resource/turtlebot_webots.urdf"         "$PKG/urdf/turtlebot_webots.urdf"
copia "$TB/resource/ros2control.yml"               "$PKG/config/ros2control_turtlebot.yml"
copia "$TB/resource/nav2_params.yaml"              "$PKG/config/nav2_params.yaml"
copia "$TB/resource/turtlebot3_burger_example_map.yaml" "$PKG/config/mapa.yaml"
copia "$TB/resource/turtlebot3_burger_example_map.pgm"  "$PKG/config/mapa.pgm"

echo
echo "Hecho. Revisa los nombres reales en \$UR/resource y \$TB/resource si algo aparece como (falta):"
echo "  ls $UR/resource ; ls $UR/launch/robot_launch ; ls $TB/resource"
echo "Luego sigue POST_INSTALL.md (parchear nav2_params use_sim_time, ajustar move_group, etc.)."
