# 🚀 Cómo ejecutar — Clasificador Rojo

Guía paso a paso para correr el sistema completo, **especialmente tras reiniciar la computadora**.

---

## ✅ 0. Requisitos que deben estar corriendo (tras reboot)

| Qué | Cómo verificar | Si NO está |
|---|---|---|
| **WSL2 / Ubuntu 24.04** | abre una terminal Ubuntu | inicia WSL desde Windows |
| **Docker** | `docker ps` responde | abre Docker Desktop en Windows |
| **EMQX** (broker MQTT) | `docker ps \| grep emqx` → `smartlabs-emqx ... Up (healthy)` | `docker start smartlabs-emqx` |
| **Webots** (en Windows) | — (el launch lo abre solo) | instálalo / revisa la ruta en Windows |

> **EMQX** tiene política `restart: unless-stopped`, así que **arranca solo** cuando Docker inicia. Solo confírmalo con `docker ps`. Puertos: **1883** (MQTT) y **18083** (dashboard).

---

## 🖥️ 1. Abrir terminal y cargar el entorno

En **cada terminal nueva** (el `.bashrc` ya carga ROS 2 Jazzy; el workspace hay que sourcearlo):

```bash
source /opt/ros/jazzy/setup.bash               # (normalmente ya está en .bashrc)
source ~/proyecto_banda_ws/install/setup.bash  # workspace del proyecto
```

> 💡 Si quieres que sea automático, agrega esa segunda línea al final de tu `~/.bashrc`.

---

## 🔨 2. (Opcional) recompilar

El `install/` **persiste tras el reboot**, así que normalmente **NO hace falta**. Solo si cambiaste código C++:

```bash
cd ~/proyecto_banda_ws
colcon build --packages-select clasificador_rojo --symlink-install
source install/setup.bash
```

---

## ▶️ 3. Lanzar el sistema completo (con GUI + RViz)

```bash
ros2 launch clasificador_rojo lanzamiento_etapa3.py gui:=true
```

- Se abren **Webots** (ventana en Windows), **RViz** y el **panel de control** (Tkinter).
- **Espera ~90 s**: Webots carga, los robots conectan, MoveIt 2 y Nav 2 inicializan (Nav 2 arranca con un retraso a propósito).
- La banda queda en estado **`LISTO`** — **NO se mueve sola**, espera tu orden.

---

## 🎛️ 4. Operar (desde el panel o por MQTT)

| Panel | MQTT equivalente | Qué hace |
|---|---|---|
| **▶ Iniciar / Reanudar** | `mosquitto_pub -t banda/comando -m start` | arranca la banda y el ciclo |
| **⏸ Pausar** | `mosquitto_pub -t banda/comando -m stop` | detiene la banda |
| **⟳ Reiniciar** | `mosquitto_pub -t banda/comando -m reset` | caja a la banda + robot al dock → repite el ciclo |

**El ciclo corre solo:** detecta la caja roja → para la banda → el UR5e la toma (MoveIt 2) y la pone en el TurtleBot → el TurtleBot va a la estación esquivando el cilindro (Nav 2) → la banda queda detenida.

---

## 📊 5. Visualización

- **RViz**: imagen de la cámara con la caja en un recuadro verde + mapa, costmaps, ruta `/plan` de Nav 2, y láser `/scan`.
- **Webots** (Windows): la célula 3D — banda, UR5e, TurtleBot, recinto, obstáculo.
- **Dashboard EMQX**: http://localhost:18083 (ver el tráfico MQTT en vivo).
- **Panel — Monitor de condición** (mantenimiento predictivo): la **temperatura del motor** sube cuando la banda corre. *Demo:* con la banda corriendo de continuo, en **~8 s** cruza **ALARM (78 °C)** → entra el **enfriamiento** solo y la curva **se estabiliza** en verde; o pulsa **❄ Enfriamiento** a mano. La telemetría se guarda **cada minuto** en `~/proyecto_banda_ws/telemetria_motor.sqlite` (dataset para entrenar IA después).

---

## ⏹️ 6. Parar todo

`Ctrl+C` en la terminal del launch. Si quedan procesos colgados:

```bash
pkill -9 -f "[l]anzamiento_etapa3"; pkill -9 -f "[w]ebots_ros2_driver"; pkill -9 -f "[m]ove_group"
pkill -9 -f "[c]ontroller_server"; pkill -9 -f "[r]viz2"; pkill -9 -f "[c]ontrol_gui"
/mnt/c/Windows/System32/taskkill.exe /F /IM webots-bin.exe /T
```

---

## 🧩 Lanzamientos por etapa (demos parciales)

```bash
ros2 launch clasificador_rojo lanzamiento_etapa1.py             # banda + cámara + detección + MQTT
ros2 launch clasificador_rojo lanzamiento_etapa2.py             # + UR5e pick&place (MoveIt 2)
ros2 launch clasificador_rojo lanzamiento_etapa3.py gui:=true   # COMPLETO + TurtleBot/Nav 2 + RViz + panel
```

---

## 🛠️ Solución de problemas

| Síntoma | Causa / arreglo |
|---|---|
| La banda no se mueve al arrancar | Está en **LISTO** — presiona **▶ Iniciar** (es a propósito). |
| `Cannot connect ... Giving up` (controladores) | IP de WSL2 no ruteable. El launch ya usa el **gateway** (`ip route`). Mata y relanza. |
| No responde a comandos MQTT | EMQX caído → `docker start smartlabs-emqx`. |
| El panel / RViz no abren | Falta el display de WSLg. Verifica `echo $DISPLAY` (`:0`) y `echo $WAYLAND_DISPLAY`. |
| Webots no abre | Webots no instalado o ruta incorrecta en Windows (`/mnt/c/Program Files/Webots`). |
| El reinicio no arranca la banda | El robot aún no confirmó que llegó al dock; espera o presiona **⟳** de nuevo. |
| Todo se cierra al arrancar | Excepción en el launch → revisa `/tmp/etapa3.log` (busca `exception` / `executed more than once`). |
| `command not found: ros2` | Falta `source /opt/ros/jazzy/setup.bash` + `source ~/proyecto_banda_ws/install/setup.bash`. |

---

## 🔁 Receta mínima tras reiniciar (TL;DR)

```bash
# 1. Verifica el broker MQTT (arranca solo, pero confírmalo)
docker ps | grep emqx            # o: docker start smartlabs-emqx

# 2. Nueva terminal: carga el entorno
source /opt/ros/jazzy/setup.bash
source ~/proyecto_banda_ws/install/setup.bash

# 3. Lanza (espera ~90 s a que quede en LISTO)
ros2 launch clasificador_rojo lanzamiento_etapa3.py gui:=true

# 4. En el panel: ▶ Iniciar   (o: mosquitto_pub -t banda/comando -m start)
```

---

*Repo: github.com/joseIronMaker/clasificador_rojo · Stack: WSL2 · ROS 2 Jazzy · Webots R2025a · EMQX · MoveIt 2 · Nav 2*
