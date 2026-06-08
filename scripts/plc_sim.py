#!/usr/bin/env python3
"""PLC simulado: publica telemetría del proceso a EMQX por MQTT (paho-mqtt).

Equivalente al `plc_sim.py` del curso: un proceso externo que reporta datos de planta
(velocidad de banda, cajas procesadas, temperatura del motor, estado) al broker, visibles
en el dashboard de EMQX (http://localhost:18083, topic `planta/plc/estado`).
"""
import json
import random
import time

import paho.mqtt.client as mqtt

BROKER = "localhost"
PORT = 1883
TOPIC = "planta/plc/estado"


def make_client():
    # Compatibilidad paho-mqtt v2 (requiere CallbackAPIVersion) y v1.
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="plc_sim")
    except AttributeError:
        return mqtt.Client(client_id="plc_sim")


def main():
    client = make_client()
    client.connect(BROKER, PORT, keepalive=60)
    client.loop_start()
    print(f"[plc_sim] publicando en '{TOPIC}' -> {BROKER}:{PORT}")

    cajas = 0
    try:
        while True:
            cajas += random.randint(0, 1)
            estado = {
                "timestamp": round(time.time(), 2),
                "banda_velocidad_mps": round(random.uniform(0.20, 0.26), 3),
                "cajas_procesadas": cajas,
                "temperatura_motor_c": round(random.uniform(35.0, 45.0), 1),
                "estado": "RUN",
            }
            client.publish(TOPIC, json.dumps(estado))
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
