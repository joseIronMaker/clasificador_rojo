"""Etapa 1 — banda + cámara (detección de rojo) + start/stop por MQTT.

Lanza Webots con el mundo combinado y conecta SOLO los drivers de esta etapa:
  - camera_robot   (publica /camera/image_color)
  - robot_supervisor (plugin C++ que controla la banda)
y los nodos: deteccion_rojo, orquestador (etapa 1), mqtt_client, plc_sim.
El UR5e y el TurtleBot del mundo quedan inertes (sin controlador) en esta etapa.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection

# --- Arreglo de red WSL2 ---------------------------------------------------------------
# En WSL2, webots_ros2 deriva la IP de Webots (Windows) del nameserver de /etc/resolv.conf
# (p.ej. 10.255.255.254), que aquí es un proxy DNS NO ruteable. Webots es accesible por la
# IP del gateway por defecto. Sobreescribimos get_wsl_ip_address para usar el gateway
# (dinámico: válido aunque cambie la subred al reiniciar WSL). Esto corrige tanto los
# WebotsController como el Ros2Supervisor (ambos calculan la IP vía esta función).
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
# ---------------------------------------------------------------------------------------


def generate_launch_description():
    pkg = get_package_share_directory("clasificador_rojo")
    world = os.path.join(pkg, "worlds", "mundo_banda.wbt")

    webots = WebotsLauncher(world=world, ros2_supervisor=True)

    camara = WebotsController(
        robot_name="camera_robot",
        parameters=[
            {"robot_description": os.path.join(pkg, "urdf", "camara.urdf"),
             "use_sim_time": True},
        ],
    )
    supervisor = WebotsController(
        robot_name="robot_supervisor",
        parameters=[
            {"robot_description": os.path.join(pkg, "urdf", "robot_supervisor.urdf"),
             "use_sim_time": True},
        ],
    )

    deteccion = Node(
        package="clasificador_rojo", executable="deteccion_rojo",
        parameters=[{"use_sim_time": True}], output="screen",
    )
    orquestador = Node(
        package="clasificador_rojo", executable="orquestador",
        parameters=[{"use_sim_time": True, "etapa": 1}], output="screen",
    )
    mqtt = Node(
        package="mqtt_client", executable="mqtt_client", name="mqtt_client",
        parameters=[os.path.join(pkg, "config", "mqtt_params.yaml")], output="screen",
    )
    plc = Node(
        package="clasificador_rojo", executable="plc_sim.py", output="screen",
    )

    wait = WaitForControllerConnection(
        target_driver=camara,
        nodes_to_start=[deteccion, orquestador, mqtt, plc],
    )

    return LaunchDescription([
        webots,
        webots._supervisor,
        camara,
        supervisor,
        wait,
        RegisterEventHandler(
            OnProcessExit(target_action=webots, on_exit=[EmitEvent(event=Shutdown())])
        ),
    ])
