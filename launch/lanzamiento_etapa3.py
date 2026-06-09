"""Etapa 3 — flujo completo: Etapa 2 (UR5e pick&place) + TurtleBot3 (Nav2) lleva la caja a la estación.

Construido SOBRE la Etapa 2 que ya funciona (mismo UR5e/MoveIt/agarre). Se añade el TurtleBot3 Burger
(ya está en el mundo como DEF TURTLEBOT) con frames PREFIJADOS `tb_` para NO chocar con el `base_link`
del UR5e (que vive en el /tf global para el agarre). Los dos árboles TF coexisten:
  UR5e:      world -> base_link -> ... -> tool0          (global /tf)
  TurtleBot: map -> tb_odom -> tb_base_link -> {tb_base_footprint, tb_base_scan}

Nav2 (3b) usa map->tb_odom ESTÁTICO (sin AMCL: el odom del simulador no deriva en un trayecto corto)
para que map == world y los goals vayan en coords del mundo; el costmap local del /scan esquiva la
banda/pedestal. La caja sobre el TurtleBot la sigue el plugin por Supervisor (no por TF).
"""
import os
import pathlib
import subprocess

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
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

# Pose de spawn del UR5e (= Etapa 2). El TF estático world->base_link debe COINCIDIR.
UR_X, UR_Y, UR_Z, UR_YAW = -0.10, -0.45, 0.60, 1.5708
# Dock del TurtleBot (= sitio de descarga del UR5e). Estación de recolección (goal de Nav2).
TB_X, TB_Y = -0.55, -0.45
EST_X, EST_Y = -2.0, -1.5


def generate_launch_description():
    pkg = get_package_share_directory("clasificador_rojo")
    world = os.path.join(pkg, "worlds", "mundo_banda.wbt")
    ur5e_urdf = os.path.join(pkg, "urdf", "ur5e_with_gripper.urdf")
    tb_urdf = os.path.join(pkg, "urdf", "turtlebot_webots.urdf")
    ros2_control = os.path.join(pkg, "config", "ros2_control_ur5e.yaml")
    tb_control = os.path.join(pkg, "config", "ros2control_turtlebot.yml")
    nav2_params = os.path.join(pkg, "config", "nav2_params.yaml")
    mapa = os.path.join(pkg, "config", "mapa_celda.yaml")  # recinto con paredes (scripts/genera_mapa.py)

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

    # ================= UR5e (idéntico a Etapa 2) =================
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
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               output="screen", parameters=[{"robot_description": rd(ur5e_urdf), "use_sim_time": True}],
               remappings=[("joint_states", "/ur5e/joint_states"),
                           ("robot_description", "/ur5e/robot_description")])
    tf_world_ur5e = Node(package="tf2_ros", executable="static_transform_publisher", output="screen",
                         arguments=[str(UR_X), str(UR_Y), str(UR_Z), str(UR_YAW), "0", "0", "world", "base_link"])

    description = {"robot_description": rd(ur5e_urdf)}
    description_semantic = {"robot_description_semantic": rd(os.path.join(pkg, "config", "moveit_ur5e.srdf"))}
    description_kinematics = {"robot_description_kinematics": ry(os.path.join(pkg, "config", "moveit_kinematics.yaml"))}
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
    brazo = Node(
        package="clasificador_rojo", executable="brazo", output="screen",
        parameters=[description, description_semantic, description_kinematics, description_planning, sim_time,
                    {"vel_scaling": 0.15,
                     "hover_pick":  [-0.3007, -1.6833, 1.9549, -1.8424, -1.5708, -1.8715],
                     "pick":        [-0.3007, -1.4919, 2.2051, -2.2839, -1.5708, -1.8715],
                     "hover_place": [ 1.2701, -1.6596, 2.0143, -1.9255, -1.5708, -0.3007],
                     "place":       [ 1.2701, -1.4307, 2.2395, -2.3796, -1.5708, -0.3007]}])

    # ================= TurtleBot3 Burger (Etapa 3) =================
    # El TurtleBot ya está en el mundo (DEF TURTLEBOT) -> su driver conecta de una (no se spawnea).
    # NAMESPACE "tb": CRÍTICO. Sin namespace, su rsp publica /robot_description GLOBAL (URDF con ruedas)
    # y choca con el del UR5e -> el driver del UR5e agarra ese URDF y crashea ("Cannot find Motor right
    # wheel motor"). Con namespace, publica /tb/robot_description y su controller_manager es
    # /tb/controller_manager -> aislado del UR5e (igual que el UR5e está aislado en /ur5e).
    # El hardware se enlaza por robot_name="TurtleBot3Burger" (el namespace solo afecta tópicos ROS).
    # SIN respawn: con respawn, si el driver re-arranca re-imprime "Controller successfully connected"
    # y WaitForControllerConnection (que no tiene guarda) re-dispara los spawners -> "executed more
    # than once" -> excepción del launch -> shutdown en cascada (Webots se cierra).
    tb_driver = WebotsController(
        robot_name="TurtleBot3Burger", namespace="tb",
        parameters=[{"robot_description": tb_urdf, "use_sim_time": True,
                     "set_robot_state_publisher": False},
                    tb_control])
    # rsp explícito del TurtleBot -> publica /tb/robot_description (lo lee su /tb/controller_manager
    # para inicializar el diff_drive). El URDF lleva un <link> raíz mínimo para que rsp lo parsee.
    tb_rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
                  namespace="tb", output="screen",
                  parameters=[{"robot_description": rd(tb_urdf), "use_sim_time": True}])
    tbcmt = ["--controller-manager-timeout", "120"]
    diff_spawner = Node(package="controller_manager", executable="spawner", output="screen",
                        arguments=["diffdrive_controller", "-c", "/tb/controller_manager"] + tbcmt)
    jsb_tb = Node(package="controller_manager", executable="spawner", output="screen",
                  arguments=["joint_state_broadcaster", "-c", "/tb/controller_manager"] + tbcmt)
    # (El árbol tb_base_link->tb_base_footprint/tb_base_scan/ruedas lo publica tb_rsp desde el URDF.)

    # ================= Nav2 (Etapa 3b) =================
    # map->tb_odom ESTATICO = pose del dock (TB_X,TB_Y), yaw 0 -> map == world (el diff_drive arranca su
    # odom en (0,0) DONDE está físicamente el robot, que es el dock; sin AMCL este TF cierra map->...->base).
    # Así los goals van en coords del mundo (la estación EST_X,EST_Y). Cadena TF completa:
    #   map -(static)-> tb_odom -(diff_drive)-> tb_base_link -(tb_rsp)-> tb_base_scan (frame del /scan).
    tf_map_odom = Node(package="tf2_ros", executable="static_transform_publisher", output="screen",
                       arguments=[str(TB_X), str(TB_Y), "0", "0", "0", "0", "map", "tb_odom"])

    # Nav2 MINIMO (sin namespace: hay UN solo stack de navegación y los frames ya son tb_; el
    # orquestador llama /navigate_to_pose global). cmd_vel de RPP/behaviors -> diff_drive (TwistStamped).
    nav2_lifecycle = ["map_server", "planner_server", "controller_server", "behavior_server", "bt_navigator"]
    cmd_remap = [("cmd_vel", "/tb/diffdrive_controller/cmd_vel")]
    # Los 5 nodos worker arrancan PRIMERO (quedan en estado 'unconfigured', casi sin CPU) para que
    # completen el DISCOVERY de fastrtps mientras Webots/MoveIt aún cargan.
    nav2_workers = [
        Node(package="nav2_map_server", executable="map_server", name="map_server", output="screen",
             parameters=[nav2_params, {"yaml_filename": mapa, "use_sim_time": True}]),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", output="screen",
             parameters=[nav2_params]),
        Node(package="nav2_controller", executable="controller_server", name="controller_server",
             output="screen", parameters=[nav2_params], remappings=cmd_remap),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server",
             output="screen", parameters=[nav2_params], remappings=cmd_remap),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator",
             output="screen", parameters=[nav2_params]),
    ]
    # El lifecycle_manager (autostart) arranca RETRASADO 25 s: para entonces MoveIt ya inicializó y el
    # sistema está tranquilo, así las llamadas change_state no se pierden por la carga (si arranca en
    # el pico de carga, la respuesta de configure se DESCARTA y el manager se queda colgado -> Nav2
    # nunca activa). El orquestador (etapa 3) NO arranca la banda hasta ver /navigate_to_pose, así que
    # este retraso solo demora el inicio del ciclo, no lo rompe.
    nav2_manager = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_nav", output="screen",
        parameters=[{"use_sim_time": True, "autostart": True, "node_names": nav2_lifecycle}])

    # ================= Etapa 1 (cámara, detección, banda, MQTT) =================
    deteccion = Node(package="clasificador_rojo", executable="deteccion_rojo",
                     parameters=[{"use_sim_time": True}], output="screen")
    # (3b) orquestador en etapa 3: caja roja -> detiene banda -> UR5e pick&place sobre el TurtleBot ->
    # NavigateToPose a la estación -> banda queda DETENIDA (reanuda con "start" por MQTT).
    orquestador = Node(package="clasificador_rojo", executable="orquestador",
                       parameters=[{"use_sim_time": True, "etapa": 3,
                                    "estacion_x": EST_X, "estacion_y": EST_Y}], output="screen")
    mqtt = Node(package="mqtt_client", executable="mqtt_client", name="mqtt_client",
                parameters=[os.path.join(pkg, "config", "mqtt_params.yaml")], output="screen")
    # Monitor de condición = el "PLC de planta": modela la temperatura del motor, dispara la alarma
    # y el enfriamiento, y publica la telemetría por MQTT (sustituye a plc_sim, que daba temperatura
    # aleatoria sin lazo). El panel y RViz leen /planta/temperatura, /planta/alarma, /planta/enfriamiento.
    # use_sim_time=False A PROPÓSITO: el modelo térmico corre en RELOJ REAL, así la subida de
    # temperatura no depende del "factor de tiempo real" de Webots (que en WSL2 sin GPU va a ~0.3x
    # y haría la demo lentísima). Con reloj real: ~8 s a la alarma y enfriamiento visible al instante.
    monitor = Node(package="clasificador_rojo", executable="monitor_condicion.py",
                   parameters=[{"use_sim_time": False}], output="screen")

    # ================= Visualización (opcional: gui:=true) =================
    # RViz (cámara/detección + mapa/costmaps/ruta/scan de Nav2) + panel de control (Tkinter).
    gui = LaunchConfiguration("gui")
    rviz = Node(package="rviz2", executable="rviz2", name="rviz2", output="screen",
                arguments=["-d", os.path.join(pkg, "config", "etapa3.rviz")],
                parameters=[{"use_sim_time": True}], condition=IfCondition(gui))
    panel = Node(package="clasificador_rojo", executable="control_gui.py", output="screen",
                 condition=IfCondition(gui))

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false",
                              description="abre RViz (cámara/detección + Nav2) y el panel de control"),
        webots, webots._supervisor,
        camara, supervisor,
        rsp, tf_world_ur5e, tf_map_odom,
        tb_driver, tb_rsp,
        rviz, panel,
        WaitForControllerConnection(
            target_driver=camara,
            nodes_to_start=[spawn_ur5e, deteccion, orquestador, mqtt, monitor]),
        RegisterEventHandler(OnProcessIO(
            target_action=spawn_ur5e,
            on_stdout=lambda event: get_webots_driver_node(event, ur5e_driver))),
        WaitForControllerConnection(
            target_driver=ur5e_driver,
            nodes_to_start=[traj_spawner, jsb_spawner, move_group, brazo]),
        WaitForControllerConnection(
            target_driver=tb_driver,
            nodes_to_start=[diff_spawner, jsb_tb]),
        # Los workers de Nav2 arrancan cuando diff_spawner SALE (diff_drive ya activo -> publica odom y
        # el TF tb_odom->tb_base_link). El lifecycle_manager se retrasa 25 s (sistema ya tranquilo) para
        # que el bringup no se cuelgue por una respuesta change_state perdida bajo carga.
        RegisterEventHandler(OnProcessExit(
            target_action=diff_spawner,
            on_exit=nav2_workers + [TimerAction(period=25.0, actions=[nav2_manager])])),
        RegisterEventHandler(OnProcessExit(target_action=webots, on_exit=[EmitEvent(event=Shutdown())])),
    ])
