"""Etapa 2 — UR5e pick&place (MoveIt2) + agarre por Supervisor.

Mi Webots (mundo_banda) + cámara + supervisor (Etapa 1) + UR5e por URDFSpawner (patrón del
stock robot_nodes_launch.py) + ros2_control + move_group (estructura exacta del stock
robot_moveit_nodes_launch.py) + brazo + orquestador(etapa 2). Sin RViz (rendimiento WSL).

Geometría (UR_*) y poses pick/place son parámetros — se afinan en vivo con el brazo en escena.
"""
import os
import pathlib
import subprocess

import yaml
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.events import Shutdown
from launch_ros.actions import Node

from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection

# --- Arreglo de red WSL2 (ver Etapa 1) ---
import webots_ros2_driver.utils as _wru  # noqa: E402


def _gateway_ip():
    try:
        out = subprocess.run(["ip", "route"], capture_output=True, text=True).stdout
        for line in out.splitlines():
            f = line.split()
            if len(f) >= 3 and f[0] == "default":
                return f[2]
    except Exception:
        pass
    return "127.0.0.1"


_wru.get_wsl_ip_address = _gateway_ip
# -----------------------------------------

# Pose de spawn del UR5e (junto a la banda). El TF estático world->base_link debe COINCIDIR.
# y=-0.45 (no -0.65): acerca el brazo a la banda (caja en y=0) -> alcance del pick holgado (~0.57 m).
UR_X, UR_Y, UR_Z, UR_YAW = -0.10, -0.45, 0.60, 1.5708


def generate_launch_description():
    pkg = get_package_share_directory("clasificador_rojo")
    world = os.path.join(pkg, "worlds", "mundo_banda.wbt")
    ur5e_urdf = os.path.join(pkg, "urdf", "ur5e_with_gripper.urdf")
    ros2_control = os.path.join(pkg, "config", "ros2_control_ur5e.yaml")

    def rd(path):
        return pathlib.Path(path).read_text()

    def ry(path):
        return yaml.safe_load(rd(path))

    webots = WebotsLauncher(world=world, ros2_supervisor=True)

    camara = WebotsController(
        robot_name="camera_robot",
        parameters=[{"robot_description": os.path.join(pkg, "urdf", "camara.urdf"),
                     "use_sim_time": True}])
    supervisor = WebotsController(
        robot_name="robot_supervisor",
        parameters=[{"robot_description": os.path.join(pkg, "urdf", "robot_supervisor.urdf"),
                     "use_sim_time": True}])

    # --- UR5e: spawn + driver (gateado) + controladores (patrón stock) ---
    # init_pos: arranca el brazo YA en `hover_pick` (gripper sobre el frente de la banda, top-down)
    # -> sin el gran despliegue desde "pointed_up"; los movimientos del pick son cortos y limpios.
    # SIN box_collision: las cajas envolventes gruesas se auto-colisionaban (cientos de contactos);
    # con la malla stock el brazo es estable como en el demo oficial. (El agarre es por teletransporte,
    # así que el brazo no necesita colisionar con nada.) Los 6 valores = joints del brazo = hover_pick.
    spawn_ur5e = URDFSpawner(
        name="UR5e", urdf_path=ur5e_urdf,
        translation=f"{UR_X} {UR_Y} {UR_Z}", rotation=f"0 0 1 {UR_YAW}",
        init_pos="-0.3007 -1.6833 1.9549 -1.8424 -1.5708 -1.8715")
    ur5e_driver = WebotsController(
        robot_name="UR5e", namespace="ur5e",
        parameters=[{"robot_description": ur5e_urdf}, {"use_sim_time": True},
                    {"set_robot_state_publisher": True}, ros2_control])
    cmt = ["--controller-manager-timeout", "100"]
    traj_spawner = Node(package="controller_manager", executable="spawner", output="screen",
                        arguments=["ur_joint_trajectory_controller", "-c", "ur5e/controller_manager"] + cmt)
    jsb_spawner = Node(package="controller_manager", executable="spawner", output="screen",
                       arguments=["ur_joint_state_broadcaster", "-c", "ur5e/controller_manager"] + cmt)
    # rsp con el URDF REAL desde el arranque (evita la carrera "no ros2_control tag":
    # el controller_manager lee /ur5e/robot_description del topic y nunca debe ver un dummy).
    # rsp SIN namespace -> publica el árbol base_link->...->tool0 al /tf GLOBAL (no /ur5e/tf),
    # conectado con el world->base_link estático -> el plugin de agarre puede mirar world->tool0.
    # Lee /ur5e/joint_states (remapeado) y no pisa /robot_description global.
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               output="screen", parameters=[{"robot_description": rd(ur5e_urdf), "use_sim_time": True}],
               remappings=[("joint_states", "/ur5e/joint_states"),
                           ("robot_description", "/ur5e/robot_description")])

    # TF world -> base_link del UR5e (coincide con el spawn): para el agarre por TF y planificar en 'world'
    tf_world_ur5e = Node(package="tf2_ros", executable="static_transform_publisher", output="screen",
                         arguments=[str(UR_X), str(UR_Y), str(UR_Z), str(UR_YAW), "0", "0", "world", "base_link"])

    # --- MoveIt move_group (estructura EXACTA del stock) ---
    description = {"robot_description": rd(ur5e_urdf)}
    description_semantic = {"robot_description_semantic": rd(os.path.join(pkg, "config", "moveit_ur5e.srdf"))}
    description_kinematics = {"robot_description_kinematics": ry(os.path.join(pkg, "config", "moveit_kinematics.yaml"))}
    # Límites de joints con ACELERACIÓN (la URDF no la trae) -> sin esto TOTG falla y move() aborta.
    description_planning = {"robot_description_planning": ry(os.path.join(pkg, "config", "joint_limits.yaml"))}
    sim_time = {"use_sim_time": True}
    movegroup = {"move_group": ry(os.path.join(pkg, "config", "moveit_movegroup.yaml"))}
    moveit_controllers = {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": ry(os.path.join(pkg, "config", "moveit_controllers.yaml"))}
    move_group = Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        parameters=[description, description_semantic, description_kinematics, description_planning,
                    moveit_controllers, movegroup, sim_time],
        remappings=[("/joint_states", "/ur5e/joint_states")])

    # --- Nodo brazo (ESPACIO DE JOINTS; 5 configs calculadas con /compute_ik, rama consistente) ---
    # hover_pick/pick: top-down sobre la caja en frente del robot (tool0 en world ~(-0.10,0,0.83)).
    # hover_place/place: top-down sobre el sitio de descarga (world ~(-0.55,-0.45,0.80)).
    # Afinar en vivo:  ros2 param set /brazo pick "[...6 joints...]"
    brazo = Node(
        package="clasificador_rojo", executable="brazo", output="screen",
        parameters=[description, description_semantic, description_kinematics, description_planning, sim_time,
                    {"vel_scaling": 0.15,
                     "hover_pick":  [-0.3007, -1.6833, 1.9549, -1.8424, -1.5708, -1.8715],
                     "pick":        [-0.3007, -1.4919, 2.2051, -2.2839, -1.5708, -1.8715],
                     "hover_place": [ 1.2701, -1.6596, 2.0143, -1.9255, -1.5708, -0.3007],
                     "place":       [ 1.2701, -1.4307, 2.2395, -2.3796, -1.5708, -0.3007]}])

    # --- Nodos de Etapa 1 (cámara, detección, banda, MQTT) ---
    deteccion = Node(package="clasificador_rojo", executable="deteccion_rojo",
                     parameters=[{"use_sim_time": True}], output="screen")
    orquestador = Node(package="clasificador_rojo", executable="orquestador",
                       parameters=[{"use_sim_time": True, "etapa": 2}], output="screen")
    mqtt = Node(package="mqtt_client", executable="mqtt_client", name="mqtt_client",
                parameters=[os.path.join(pkg, "config", "mqtt_params.yaml")], output="screen")
    plc = Node(package="clasificador_rojo", executable="plc_sim.py", output="screen")

    return LaunchDescription([
        webots, webots._supervisor,
        camara, supervisor,
        rsp, tf_world_ur5e,
        # Cuando la cámara/supervisor conectan (el servicio de spawn ya existe) ->
        # spawnear el UR5e y arrancar los nodos de la Etapa 1.
        WaitForControllerConnection(
            target_driver=camara,
            nodes_to_start=[spawn_ur5e, deteccion, orquestador, mqtt, plc]),
        # Cuando el spawn imprime éxito -> arrancar el driver del UR5e.
        RegisterEventHandler(OnProcessIO(
            target_action=spawn_ur5e,
            on_stdout=lambda event: get_webots_driver_node(event, ur5e_driver))),
        # Cuando el UR5e conecta -> controladores + move_group + brazo.
        WaitForControllerConnection(
            target_driver=ur5e_driver,
            nodes_to_start=[traj_spawner, jsb_spawner, move_group, brazo]),
        # Cerrar todo solo si Webots muere (no si el brazo falla, para poder depurar).
        RegisterEventHandler(OnProcessExit(target_action=webots, on_exit=[EmitEvent(event=Shutdown())])),
    ])
