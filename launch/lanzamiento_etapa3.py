"""Etapa 3 — flujo completo: + TurtleBot3 (Nav2) lleva la caja a la estación.

DRAFT: finalizar tras instalar (ver POST_INSTALL.md). Requiere:
  - Añadir 'DEF TURTLEBOT TurtleBot3Burger {...}' al mundo (worlds/mundo_banda.wbt).
  - Recursos stock copiados: turtlebot_webots.urdf, ros2control_turtlebot.yml, nav2_params.yaml (parcheado), mapa.{yaml,pgm}.
Estructura = Etapa 2 + TurtleBot (WebotsController) + Nav2 (nav2_bringup) + orquestador etapa 3.
Puntos a verificar marcados con  # VERIFICAR.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.urdf_spawner import URDFSpawner
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection

# --- Arreglo de red WSL2: webots_ros2 deriva la IP de Webots del nameserver de resolv.conf
# (proxy DNS no ruteable); Webots es accesible por el gateway por defecto. Ver Etapa 1. ---
import subprocess  # noqa: E402
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


def _read(path):
    with open(path) as f:
        return f.read()


def _yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    pkg = get_package_share_directory("clasificador_rojo")
    world = os.path.join(pkg, "worlds", "mundo_banda.wbt")
    ur5e_urdf = os.path.join(pkg, "urdf", "ur5e_with_gripper.urdf")
    tb_urdf = os.path.join(pkg, "urdf", "turtlebot_webots.urdf")

    webots = WebotsLauncher(world=world, ros2_supervisor=True)

    camara = WebotsController(
        robot_name="camera_robot",
        parameters=[{"robot_description": os.path.join(pkg, "urdf", "camara.urdf"),
                     "use_sim_time": True}],
    )
    supervisor = WebotsController(
        robot_name="robot_supervisor",
        parameters=[{"robot_description": os.path.join(pkg, "urdf", "robot_supervisor.urdf"),
                     "use_sim_time": True}],
    )

    # --- UR5e (igual que Etapa 2) ---
    spawn_ur5e = URDFSpawner(name="UR5e", urdf_path=ur5e_urdf,
                             translation="0.9 0 0.6", rotation="0 0 1 -1.5708")
    ur5e_driver = WebotsController(
        robot_name="UR5e", namespace="ur5e", respawn=True,
        parameters=[{"robot_description": ur5e_urdf, "use_sim_time": True,
                     "set_robot_state_publisher": True},
                    os.path.join(pkg, "config", "ros2_control_ur5e.yaml")],
    )
    ur5e_driver_delayed = TimerAction(period=6.0, actions=[ur5e_driver])
    traj_spawner = Node(package="controller_manager", executable="spawner",
                        arguments=["ur_joint_trajectory_controller", "-c", "/ur5e/controller_manager",
                                   "--controller-manager-timeout", "120"])
    jsb_spawner = Node(package="controller_manager", executable="spawner",
                       arguments=["ur_joint_state_broadcaster", "-c", "/ur5e/controller_manager",
                                  "--controller-manager-timeout", "120"])

    rd = {"robot_description": _read(ur5e_urdf)}
    rds = {"robot_description_semantic": _read(os.path.join(pkg, "config", "moveit_ur5e.srdf"))}
    kin = _yaml(os.path.join(pkg, "config", "moveit_kinematics.yaml"))
    ompl = _yaml(os.path.join(pkg, "config", "moveit_movegroup.yaml"))
    mctrl = _yaml(os.path.join(pkg, "config", "moveit_controllers.yaml"))
    move_group = Node(package="moveit_ros_move_group", executable="move_group", output="screen",
                      parameters=[rd, rds, kin, ompl, mctrl,
                                  {"use_sim_time": True, "publish_robot_description_semantic": True}],
                      remappings=[("/joint_states", "/ur5e/joint_states")])
    tf_world_ur5e = Node(package="tf2_ros", executable="static_transform_publisher",
                         arguments=["0.9", "0", "0.6", "-1.5708", "0", "0", "world", "base_link"])
    brazo = Node(package="clasificador_rojo", executable="brazo", output="screen",
                 parameters=[rd, rds, kin, {"use_sim_time": True}])

    # --- TurtleBot3 ---
    # VERIFICAR: prefijo de frames para evitar colisión con base_link del UR5e
    # (p.ej. frame_prefix 'turtlebot/' en su robot_state_publisher y tb_frame en el plugin).
    footprint = Node(package="tf2_ros", executable="static_transform_publisher",
                     arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"])
    tb_driver = WebotsController(
        robot_name="TurtleBot3Burger", respawn=True,
        parameters=[{"robot_description": tb_urdf, "use_sim_time": True,
                     "set_robot_state_publisher": True},
                    os.path.join(pkg, "config", "ros2control_turtlebot.yml")],
        # Jazzy: cmd_vel/odom van con remap del controlador diff_drive (TwistStamped).
        remappings=[("/diffdrive_controller/cmd_vel", "/cmd_vel"),
                    ("/diffdrive_controller/odom", "/odom")],
    )
    diff_spawner = Node(package="controller_manager", executable="spawner",
                        arguments=["diffdrive_controller", "--controller-manager-timeout", "120"])
    jsb_tb = Node(package="controller_manager", executable="spawner",
                  arguments=["joint_state_broadcaster", "--controller-manager-timeout", "120"])

    # static TF world -> map (identidad si el mapa coincide con el origen del mundo).
    tf_world_map = Node(package="tf2_ros", executable="static_transform_publisher",
                        arguments=["0", "0", "0", "0", "0", "0", "world", "map"])

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("nav2_bringup"), "launch", "bringup_launch.py")),
        launch_arguments={
            "map": os.path.join(pkg, "config", "mapa.yaml"),
            "params_file": os.path.join(pkg, "config", "nav2_params.yaml"),  # parcheado use_sim_time:true
            "use_sim_time": "true",
            "autostart": "true",
        }.items(),
    )

    deteccion = Node(package="clasificador_rojo", executable="deteccion_rojo",
                     parameters=[{"use_sim_time": True}], output="screen")
    orquestador = Node(package="clasificador_rojo", executable="orquestador",
                       parameters=[{"use_sim_time": True, "etapa": 3,
                                    "estacion_x": -2.0, "estacion_y": -1.5}], output="screen")
    mqtt = Node(package="mqtt_client", executable="mqtt_client", name="mqtt_client",
                parameters=[os.path.join(pkg, "config", "mqtt_params.yaml")], output="screen")
    plc = Node(package="clasificador_rojo", executable="plc_sim.py", output="screen")

    wait_ur5e = WaitForControllerConnection(
        target_driver=ur5e_driver,
        nodes_to_start=[traj_spawner, jsb_spawner, move_group, brazo])
    wait_tb = WaitForControllerConnection(
        target_driver=tb_driver,
        nodes_to_start=[diff_spawner, jsb_tb, nav2])
    wait_cam = WaitForControllerConnection(
        target_driver=camara,
        nodes_to_start=[deteccion, orquestador, mqtt, plc])

    return LaunchDescription([
        webots, webots._supervisor, camara, supervisor,
        spawn_ur5e, ur5e_driver_delayed, tf_world_ur5e,
        footprint, tb_driver, tf_world_map,
        wait_ur5e, wait_tb, wait_cam,
        RegisterEventHandler(
            OnProcessExit(target_action=webots, on_exit=[EmitEvent(event=Shutdown())])),
    ])
