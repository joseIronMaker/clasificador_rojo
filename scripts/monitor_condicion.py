#!/usr/bin/env python3
"""Monitor de condición del motor de la banda — mantenimiento predictivo (Industria 4.0).

Cierra el lazo IIoT que faltaba: *sensor -> análisis -> decisión -> actuador*.

Webots NO tiene un device de temperatura, así que la temperatura del motor se MODELA
(modelo térmico de 1er orden), que es justo lo correcto en un gemelo digital:
  - sube con la CARGA del motor: banda encendida (+1.0) y brazo trabajando / PROCESANDO (+0.5),
  - baja hacia el ambiente por pérdida natural,
  - baja MUCHO más rápido cuando entra el SISTEMA DE ENFRIAMIENTO (ventilador).

Lógica de planta:
  - al cruzar T_ALARM se dispara ALARMA y el enfriamiento AUTOMÁTICO (histéresis: se limpia
    por debajo de T_CLEAR),
  - el ventilador es un controlador P: MODULA su potencia para sostener la consigna T_SET,
    así la temperatura se ESTABILIZA en una zona segura en vez de oscilar,
  - el panel puede forzar el ventilador a mano (/planta/enfriamiento_cmd).

Interfaces ROS (para el panel de control y RViz):
  pub  /planta/temperatura       std_msgs/Float32   °C del motor (10 Hz)
  pub  /planta/alarma            std_msgs/Bool      sobre-temperatura
  pub  /planta/enfriamiento      std_msgs/Bool      ventilador ON/OFF
  sub  /conveyor/enable          std_msgs/Bool      carga del motor (banda)
  sub  /estado                   std_msgs/String    estado del ciclo (calor extra en PROCESANDO)
  sub  /planta/enfriamiento_cmd  std_msgs/Bool      forzar el ventilador desde el panel

Telemetría MQTT (EMQX :1883, dashboard :18083) — es el PLC de planta reportando:
  pub  planta/plc/estado  JSON 1 Hz {temperatura, alarma, enfriamiento, banda, cajas} QoS1 retained
  LWT  planta/plc/estado  {"estado":"OFFLINE"}  si el monitor cae (Last Will & Testament)
"""
import json
import os
import random
import sqlite3
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

import paho.mqtt.client as mqtt

# --- Modelo térmico (1er orden) y umbrales (°C) ---
T_AMB = 25.0       # temperatura ambiente / reposo
T_WARN = 70.0      # zona naranja (aviso)
T_ALARM = 78.0     # dispara ALARMA + enfriamiento automático
T_CLEAR = 68.0     # histéresis: la alarma se limpia por debajo de esto
T_SET = 58.0       # consigna del controlador de enfriamiento
T_FAN_OFF = 50.0   # el enfriamiento auto NO se suelta hasta enfriar de verdad (motor en reposo):
#                    así el control P sostiene una temperatura PLANA (~61°C) en vez de oscilar.

A_HEAT = 12.0      # ganancia de calentamiento por carga (°C/s) — alto: ~8 s a la alarma (demo)
B_AMB = 0.16       # pérdida natural hacia el ambiente (1/s) — A/B = 75 -> equilibrio sin fan ~100°C
C_FAN = 0.40       # capacidad máxima de enfriamiento del ventilador (1/s)
KP_FAN = 0.15      # ganancia P del controlador de enfriamiento (1/°C)
DT = 0.1           # paso del modelo (s) -> timer a 10 Hz

BROKER, PORT, TOPIC = "localhost", 1883, "planta/plc/estado"


def make_client():
    # Compatibilidad paho-mqtt v2 (requiere CallbackAPIVersion) y v1.
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="monitor_condicion")
    except AttributeError:
        return mqtt.Client(client_id="monitor_condicion")


class MonitorCondicion(Node):
    def __init__(self):
        super().__init__("monitor_condicion")

        # estado del modelo
        self.temp = T_AMB
        self.belt_on = False
        self.estado = ""
        self.fan_manual = False
        self.fan_auto = False
        self.fan_power = 0.0
        self.alarma = False
        self.cajas = 0
        self._prev_proc = False

        # ROS — publicadores para el panel / RViz
        self.pub_temp = self.create_publisher(Float32, "/planta/temperatura", 10)
        self.pub_alarma = self.create_publisher(Bool, "/planta/alarma", 10)
        self.pub_fan = self.create_publisher(Bool, "/planta/enfriamiento", 10)
        # ROS — entradas (carga del motor, estado del ciclo, orden manual de enfriar)
        self.create_subscription(Bool, "/conveyor/enable", self._on_belt, 10)
        self.create_subscription(String, "/estado", self._on_estado, 10)
        self.create_subscription(Bool, "/planta/enfriamiento_cmd", self._on_fan_cmd, 10)

        # MQTT (EMQX) con Last Will -> el dashboard ve OFFLINE si el monitor cae
        self.mqtt = make_client()
        self.mqtt.will_set(TOPIC, json.dumps({"estado": "OFFLINE", "alarma": True}),
                           qos=1, retain=True)
        self.mqtt_ok = False
        try:
            self.mqtt.connect_async(BROKER, PORT, keepalive=30)
            self.mqtt.loop_start()
            self.mqtt_ok = True
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"MQTT no disponible (sigo en ROS): {e}")

        # --- Historian local (SQLite): guarda la temperatura cada `db_interval_s` para
        #     ENTRENAR DESPUÉS un modelo de IA (mantenimiento predictivo / detección de anomalías) ---
        default_db = os.path.join(os.path.expanduser("~"), "proyecto_banda_ws", "telemetria_motor.sqlite")
        self.declare_parameter("db_path", default_db)
        self.declare_parameter("db_interval_s", 60.0)   # "cada minuto"; bájalo para un dataset más denso
        self.db_path = self.get_parameter("db_path").get_parameter_value().string_value
        self.db_interval = float(self.get_parameter("db_interval_s").value)
        self._last_db = time.time()
        self.db = None
        try:
            os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
            self.db = sqlite3.connect(self.db_path)
            self.db.execute(
                "CREATE TABLE IF NOT EXISTS telemetria ("
                "id INTEGER PRIMARY KEY AUTOINCREMENT, ts REAL, fecha TEXT, "
                "temperatura_c REAL, alarma INTEGER, enfriamiento REAL, enfriamiento_on INTEGER, "
                "banda TEXT, cajas INTEGER, estado TEXT)")
            self.db.commit()
            self.get_logger().info(f"Historian SQLite -> {self.db_path} (cada {self.db_interval:.0f}s)")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"No pude abrir SQLite ({self.db_path}): {e}")
            self.db = None

        self.create_timer(DT, self._step)           # modelo térmico 10 Hz
        self.create_timer(1.0, self._publish_mqtt)  # telemetría de planta 1 Hz (+ historian por reloj real)
        self.get_logger().info(
            "Monitor de condición LISTO: modelo térmico + alarma + enfriamiento "
            f"(WARN {T_WARN:.0f}°C, ALARM {T_ALARM:.0f}°C).")

    # ---------- callbacks ROS ----------
    def _on_belt(self, m):
        self.belt_on = bool(m.data)

    def _on_fan_cmd(self, m):
        self.fan_manual = bool(m.data)
        self.get_logger().info(f"Enfriamiento manual -> {'ON' if self.fan_manual else 'OFF'}")

    def _on_estado(self, m):
        self.estado = m.data
        proc = "PROCESANDO" in m.data
        if proc and not self._prev_proc:   # flanco de subida: una caja entró a proceso
            self.cajas += 1
        self._prev_proc = proc

    # ---------- modelo térmico + control ----------
    def _step(self):
        # carga del motor: banda (motor girando) + brazo trabajando (PROCESANDO)
        load = (1.0 if self.belt_on else 0.0) + (0.5 if "PROCESANDO" in self.estado else 0.0)

        # ¿enfriar? automático por alarma (con histéresis) u orden manual del panel
        if self.temp >= T_ALARM:
            self.fan_auto = True
        elif self.temp <= T_FAN_OFF:
            self.fan_auto = False
        fan_on = self.fan_auto or self.fan_manual

        # controlador P del ventilador: modula la potencia para sostener T_SET (estabiliza)
        self.fan_power = max(0.0, min(1.0, KP_FAN * (self.temp - T_SET))) if fan_on else 0.0

        # dinámica: dT/dt = A·carga − (B + C·fan)·(T − T_amb)
        dT = A_HEAT * load - (B_AMB + C_FAN * self.fan_power) * (self.temp - T_AMB)
        self.temp += DT * dT + random.gauss(0.0, 0.08)   # ruido pequeño de sensor
        self.temp = max(T_AMB - 1.0, self.temp)

        # alarma con histéresis
        if self.temp >= T_ALARM:
            self.alarma = True
        elif self.temp <= T_CLEAR:
            self.alarma = False

        self.pub_temp.publish(Float32(data=float(self.temp)))
        self.pub_alarma.publish(Bool(data=bool(self.alarma)))
        self.pub_fan.publish(Bool(data=bool(fan_on)))

    # ---------- historian SQLite (una fila cada db_interval s de reloj real) ----------
    def _db_log(self):
        if self.db is None:
            return
        fan_on = bool(self.fan_auto or self.fan_manual)
        try:
            self.db.execute(
                "INSERT INTO telemetria (ts, fecha, temperatura_c, alarma, enfriamiento, "
                "enfriamiento_on, banda, cajas, estado) VALUES (?,?,?,?,?,?,?,?,?)",
                (round(time.time(), 2), time.strftime("%Y-%m-%d %H:%M:%S"),
                 round(self.temp, 2), int(self.alarma), round(self.fan_power, 2),
                 int(fan_on), "ON" if self.belt_on else "OFF", self.cajas,
                 "ALARMA" if self.alarma else "RUN"))
            self.db.commit()
            self.get_logger().info(f"[BD] fila guardada  T={self.temp:.1f}°C  alarma={self.alarma}")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"[BD] error al guardar: {e}")

    # ---------- telemetría MQTT (PLC de planta) ----------
    def _publish_mqtt(self):
        now = time.time()
        if self.db is not None and (now - self._last_db) >= self.db_interval:
            self._last_db = now
            self._db_log()
        if not self.mqtt_ok:
            return
        payload = json.dumps({
            "timestamp": round(time.time(), 2),
            "temperatura_motor_c": round(self.temp, 1),
            "alarma": self.alarma,
            "enfriamiento": round(self.fan_power, 2),
            "enfriamiento_on": bool(self.fan_auto or self.fan_manual),
            "banda": "ON" if self.belt_on else "OFF",
            "cajas_procesadas": self.cajas,
            "estado": "ALARMA" if self.alarma else "RUN",
        })
        try:
            self.mqtt.publish(TOPIC, payload, qos=1, retain=True)
        except Exception:  # noqa: BLE001
            pass


def main():
    rclpy.init()
    node = MonitorCondicion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.mqtt.loop_stop()
            node.mqtt.disconnect()
        except Exception:  # noqa: BLE001
            pass
        try:
            if node.db is not None:
                node.db.close()
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
