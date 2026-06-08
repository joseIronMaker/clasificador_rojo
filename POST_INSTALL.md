# POST_INSTALL — pasos tras instalar las dependencias

Checklist para cerrar la integración cuando termine el `apt install`. Construimos y
verificamos **por etapas**.

## 0. Construir el paquete

```bash
cd ~/proyecto_banda_ws
colcon build --packages-select clasificador_rojo --symlink-install
source install/setup.bash
```

Posibles ajustes de compilación (los reviso yo al construir):
- **Header de MoveIt**: si Jazzy usa `.hpp`, cambiar en `src/brazo.cpp`
  `#include <moveit/move_group_interface/move_group_interface.h>` → `...move_group_interface.hpp`.
- **Headers de Webots**: el plugin `controlador_banda` incluye `<webots/supervisor.h>`, que aporta
  `webots_ros2_driver`. Si no resuelve, falta que `ros-jazzy-webots-ros2` esté instalado.

## 1. Etapa 1 — banda + cámara + MQTT (verificar primero)

```bash
ros2 launch clasificador_rojo lanzamiento_etapa1.py
```
Verificación:
- Abre Webots (mundo_banda) con banda, cámara cenital, caja roja + señuelos.
- `ros2 topic hz /camera/image_color` → fluye. (Si no, revisar orientación de la `Camera`
  en el `.wbt`: `rotation 0 1 0 1.5708`; invertir el signo si mira hacia arriba.)
- `ros2 topic echo /caja_roja` → `true` cuando la caja roja pasa bajo la cámara.
- `mosquitto_pub -h localhost -p 1883 -t banda/comando -m stop` → la banda se detiene;
  `-m start` → se reanuda. (Verifica el **esquema de `config/mqtt_params.yaml`** contra la versión
  instalada de mqtt_client; si el puente no entrega `/banda/comando`, ajustar el YAML.)
- Dashboard EMQX `http://localhost:18083` → topic `planta/plc/estado` (telemetría del PLC).

Si las cajas no se mueven en la banda: la `ConveyorBelt` arrastra por fricción; subir `belt_speed`
o, si es inestable en WSL, mover la caja por Supervisor desde el plugin.

## 2. Etapa 2 — UR5e + MoveIt2 + agarre

1. Copiar recursos stock:
   ```bash
   bash scripts/copiar_recursos_stock.sh
   # revisar nombres reales si algo sale como (falta):
   ls /opt/ros/jazzy/share/webots_ros2_universal_robot/resource
   ls /opt/ros/jazzy/share/webots_ros2_universal_robot/launch/robot_launch
   ```
2. Cotejar contra el stock `robot_moveit_nodes_launch.py` y afinar `launch/lanzamiento_etapa2.py`
   (marcas `# VERIFICAR`): construcción de parámetros de `move_group` (pipelines OMPL,
   `moveit_simple_controller_manager`), y el gateo del driver del UR5e tras el spawn.
3. Verificar nombres en los configs copiados:
   - Grupo de planificación = `ur5e_arm` (en `moveit_ur5e.srdf`) — usado por `brazo.cpp`.
   - Joints del gripper (`finger_1_joint_1`, `finger_2_joint_1`, `finger_middle_joint_1`) y
     valores abrir/cerrar (`ur5e_controller.py` del stock) — usados por `brazo.cpp`.
   - Acción `/ur5e/ur_joint_trajectory_controller/follow_joint_trajectory`.
4. Ajustar pose de spawn del UR5e (`translation/rotation`) para que quede junto al extremo de la
   banda, y el `static_transform_publisher world→base_link` para que **coincida** con ese spawn
   (de eso depende el agarre por TF). Verificar el nombre del link raíz (`base_link` vs `base`).
5. Afinar poses de `brazo` (parámetros `pick`, `place`, `gripper_down`, `approach_dz`) con el robot
   en escena: `ros2 param set /brazo pick "[x,y,z]"`, etc.

```bash
ros2 launch clasificador_rojo lanzamiento_etapa2.py
ros2 control list_controllers -c /ur5e/controller_manager   # ambos 'active'
ros2 action list | grep -E "follow_joint_trajectory|move_action"
```
Esperado: caja roja → banda para → `brazo` planifica y recoge → `/agarre attach` → la caja sigue
al gripper → la deja en el sitio → banda reanuda.

## 3. Etapa 3 — TurtleBot + Nav2

1. Añadir el TurtleBot al mundo `worlds/mundo_banda.wbt` (antes de la estación):
   ```
   EXTERNPROTO "webots://projects/robots/robotis/turtlebot/protos/TurtleBot3Burger.proto"
   ...
   DEF TURTLEBOT TurtleBot3Burger {
     translation 0.9 -1.0 0
     controller "<extern>"
     extensionSlot [ ... InertialUnit, GPS, RobotisLds01 ... ]
   }
   ```
   (Copiar el bloque exacto del stock `webots_ros2_turtlebot/worlds/turtlebot3_burger_example.wbt`.)
2. **Parchear `config/nav2_params.yaml`**: poner `use_sim_time: true` en TODOS los nodos;
   `set_initial_pose: true` con la pose de spawn del TurtleBot; `scan_topic: scan`.
3. **Colisión de frames**: el UR5e y el TurtleBot publican ambos `base_link`. Dar `frame_prefix`
   (p.ej. `turtlebot/`) al `robot_state_publisher` del TurtleBot y ajustar `tb_frame` del plugin
   y el `static_transform_publisher base_link→base_footprint` en consecuencia.
4. `static_transform_publisher world→map`: identidad si el origen del mapa coincide con el del mundo.
5. Mapa: el de ejemplo no corresponde a `mundo_banda`. Generar uno con `slam_toolbox` una vez, o
   usar área despejada + buen `set_initial_pose` (el costmap local con `/scan` esquiva obstáculos).

```bash
ros2 launch clasificador_rojo lanzamiento_etapa3.py
ros2 run tf2_tools view_frames        # map -> odom -> base_link
# goal manual de prueba:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -2.0, y: -1.5}}}}"
```
Flujo completo: caja roja → para banda → pick&place al TurtleBot → `/agarre to_turtlebot` →
Nav2 lleva el TurtleBot a la estación → al llegar, banda reanuda. `stop`/`start` por MQTT
interrumpe/reanuda en cualquier momento.

## Riesgos clave (recordatorio)
- **Rendimiento WSL** (llvmpipe): `basicTimeStep 32`, cámara 320×240, sin RViz, `use_sim_time:true`.
- **Esquema de mqtt_client**: verificar `mqtt_params.yaml` contra la versión instalada.
- **MoveItPy vs trayectorias**: si `move_group`/MoveGroupInterface va inestable, fallback a metas de
  joints precalculadas por `FollowJointTrajectory` (la demo se completa igual).
- **Dirección de la banda / fricción**: ajustar signo de `belt_speed` y la física de las cajas.
