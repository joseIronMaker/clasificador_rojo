# 🎬 Guion del video — Clasificador Rojo (máx. 15 min)

> **Robótica y Automatización · MQTT + ROS-Industrial · Gemelo digital en Webots**
>
> Convención: **[PANTALLA: …]** = qué mostrar · **[DEMO: …]** = acción en vivo · *texto normal* = lo que se dice.
> Ritmo objetivo ≈ 135–140 palabras/min → ~13–14 min de narración + pausas de demo.

---

## ✅ Checklist antes de grabar
- [ ] `ros2 launch clasificador_rojo lanzamiento_etapa3.py gui:=true` corriendo (Webots + RViz + panel visibles).
- [ ] Una terminal libre para el demo de MQTT (`mosquitto_pub`).
- [ ] Dashboard de EMQX abierto en `localhost:18083` (opcional, se ve bien).
- [ ] El sistema en estado **LISTO** (banda detenida, esperando ▶).
- [ ] README abierto para mostrar los diagramas.

---

## [0:00 – 1:30] Apertura — la fábrica conectada

[PANTALLA: el mundo de Webots con toda la célula — banda, UR5e, TurtleBot, paredes]

*Hola. En este video les muestro el proyecto final: una **célula industrial completa** — una banda transportadora, una cámara de visión, un brazo robótico **UR5e** y un robot móvil **TurtleBot 3** — todo coordinado por **ROS 2** y controlado por **MQTT**.*

*Y lo más importante: está implementada **cien por ciento en software**, como un **gemelo digital**, sin un solo componente de hardware real.*

*¿Por qué importa? Porque es exactamente el patrón de la **Industria 4.0**: un PLC genera datos, los manda por MQTT, ROS 2 los procesa y decide, y devuelve comandos. La fábrica conectada. Todo lo que veremos — el protocolo MQTT, los drivers ROS-Industrial, el gemelo digital — son los conceptos del curso, pero funcionando **juntos**, en un sistema que de verdad corre.*

---

## [1:30 – 3:00] Qué construimos — el ciclo

[PANTALLA: el diagrama "La célula en una imagen" del README]

*El ciclo es este. Una **caja roja** viaja por la banda. Una **cámara cenital** la detecta por color. Cuando la caja llega frente al robot, la **banda se detiene**. El **UR5e** la toma con **MoveIt 2** y la coloca sobre el **TurtleBot**. El TurtleBot **navega con Nav 2** hasta la estación de recolección, esquivando un obstáculo en el camino. Y todo — arrancar, pausar, reiniciar — se controla por **MQTT**, desde un panel o desde cualquier cliente IoT.*

*Lo construí por **etapas verificables**: primero banda y visión, luego el brazo, y al final el robot móvil con navegación. Todo en C++ y Python, en un paquete de ROS 2.*

---

## [3:00 – 6:00] Parte A — MQTT, el protocolo del IIoT

[PANTALLA: diagrama "El puente MQTT ↔ ROS 2"]

*Empecemos por **MQTT**, el protocolo del Internet de las Cosas industrial.*

*MQTT es **publish/subscribe**: nadie se llama directamente. El operador **publica** un comando en un topic — `banda/comando` — y no le importa quién escucha. Del otro lado, ROS 2 está **suscrito** y reacciona. Ese desacople es la clave: el operador **no sabe nada de ROS**, habla MQTT.*

*Como broker uso **EMQX**, no Mosquitto, porque es de **grado industrial**: escala a millones de conexiones y trae dashboard web. El puente lo hace el paquete `mqtt_client`, que traduce un mensaje de texto MQTT a un topic de ROS 2.*

[DEMO: en la terminal]
```bash
mosquitto_pub -t banda/comando -m start   # ▶ la banda arranca
mosquitto_pub -t banda/comando -m stop    # ⏸ se detiene
```
*Miren: publico `start` por MQTT desde la terminal… y la banda arranca. Publico `stop`… y se detiene. El operador **nunca tocó ROS**.*

*Y los conceptos del curso están aplicados de verdad:*
- *Los **comandos** van con **QoS 1**, "at least once" — perder un `stop` es inaceptable.*
- *La **telemetría** del PLC, que el nodo `plc_sim` publica cada segundo, va con **QoS 0** — si se pierde un dato de temperatura, no pasa nada.*
- *Y aprendí en carne propia lo de los **mensajes retained**: un comando viejo quedó retenido en el broker y reapareció solo. Tuve que limpiarlo. Eso es MQTT real, con sus detalles.*

---

## [6:00 – 9:00] Parte B — ROS-Industrial

[PANTALLA: diagrama "La abstracción que hace todo intercambiable"]

*Ahora la parte robótica: **ROS-Industrial**, la capa que lleva ROS 2 a la manufactura.*

*Su mayor logro es una **abstracción**: todos los brazos industriales — UR, FANUC, KUKA, ABB — se exponen con el **mismo Hardware Interface estándar**. Por eso **MoveIt 2 planifica igual** contra un UR5e real que contra el simulado. El código de planificación no cambia. Esa es, justamente, la propiedad que hace posible un gemelo digital.*

*En el proyecto, el nodo `brazo` usa `MoveGroupInterface` de MoveIt 2 y planifica en **espacio de articulaciones** — con cinco configuraciones que calculé una sola vez por **cinemática inversa**, para que el brazo no se enrede en cada movimiento.*

[DEMO: el UR5e tomando la caja]
*El **agarre** lo resolví de forma elegante para un gemelo digital: en vez de simular física de contacto — que es frágil y colapsa el solver al cerrar el gripper — **teletransporto** la caja para que siga al gripper usando el **Supervisor de Webots**. Es controlado, reproducible y nunca falla.*

[PANTALLA: RViz con la ruta de Nav 2 curvándose alrededor del cilindro]
*Para la navegación, el TurtleBot usa **Nav 2 completo**: planificación global, control local y un costmap que se alimenta del **lidar**. Le puse un **obstáculo** — un cilindro — en mitad del camino, y Nav 2 lo **bordea**. En RViz se ve la ruta curvándose. Eso es navegación autónoma real.*

*Y el **puente al PLC** cierra el lazo: `plc_sim` manda datos por MQTT, ROS los procesa, y los comandos vuelven. **PLC → MQTT → ROS 2 → decisión → de vuelta al PLC.** El flujo de Industria 4.0, completo, en software.*

---

## [9:00 – 11:30] Parte C — El gemelo digital, y por qué Webots

[PANTALLA: tabla "Planta física vs Gemelo digital"]

*Esto nos lleva al concepto central: el **gemelo digital**.*

*Un gemelo digital no es una animación bonita. Es una **réplica que corre el mismo software** que correría sobre el hardware real. Esa es la propiedad valiosa: lo que valido aquí, lo **despliego en planta sin reescribir**. El mismo URDF del UR5e, el mismo stack de Nav 2, el mismo flujo MQTT.*

*Ahora — el curso propone **Gazebo** para esto. Yo usé **Webots**, y quiero justificar por qué, porque fue una decisión técnica.*

[PANTALLA: tabla comparativa Webots vs Gazebo]

*Primero: Webots trae los **componentes industriales listos** — el UR5e, el TurtleBot, una **banda transportadora**, cámaras, lidar — como modelos nativos. Armar una célula heterogénea es casi inmediato.*

*Segundo: `webots_ros2` expone la **misma abstracción** `ros2_control`, MoveIt 2 y Nav 2 que Gazebo — así que la propiedad del "mismo código" se mantiene intacta.*

*Tercero, y esto fue decisivo: la **API de Supervisor** me dejó manipular el mundo con **ground-truth** — el agarre por teletransporte que mencioné, e inyectar escenarios. Ideal para un gemelo.*

*Cuarto: Webots es **estable y multiplataforma** — corre en Windows y se enlaza a ROS 2 en WSL2 — mientras que Gazebo pasó por una transición de versiones bastante fragmentada.*

*Y quinto: **funciona en hardware modesto, sin GPU**, que es donde corrí todo esto.*

*En resumen: para una célula industrial completa, en hardware limitado, Webots me dio **más componentes listos, una API que habilita escenarios de gemelo digital, y la misma abstracción de ROS 2** — sin perder nada. Es una elección igual de válida que Gazebo y, para este caso, **más productiva**.*

---

## [11:30 – 13:30] Demo completo en vivo

[DEMO en vivo — narrar mientras ocurre]

*Veámoslo todo junto.*

*El sistema arranca en estado **LISTO** — la banda **no se mueve sola**; espera mi orden, como una línea real que el operador arranca. Presiono **▶ Iniciar** en el panel.*

*La caja avanza… la cámara la detecta — vean el **recuadro verde** en RViz — y la banda se detiene.*

*El UR5e planifica con MoveIt 2, baja, toma la caja y la coloca sobre el TurtleBot.*

*Ahora el TurtleBot arranca, navega hacia la estación, **bordea el cilindro**… y llega. La banda queda detenida: fin del ciclo.*

*Y si presiono **⟳ Reiniciar**: el robot **regresa solo al dock**, y — esto es importante — la banda **solo vuelve a arrancar cuando el robot confirma que llegó** a la zona donde el brazo coloca la pieza. Si la vuelta falla, reintenta; nunca arranca a ciegas. El ciclo se repite.*

---

## [13:30 – 14:45] Edge, cloud y cierre

[PANTALLA: diagrama "Edge vs Cloud"]

*Una última pieza: ¿dónde corre la inteligencia?*

*Todo el lazo de control está en el **edge** — la visión, las decisiones, la planificación — por **latencia** y porque debe funcionar **sin internet**. Pero el broker MQTT ya publica telemetría que una **nube** podría consumir para histórico, analítica y **entrenar** modelos de anomalías, que luego se despliegan de vuelta al edge. Edge para el control en tiempo real; cloud para el aprendizaje. El patrón completo de Industria 4.0.*

*Para cerrar: este proyecto une **todo lo del curso** en un sistema que funciona — **MQTT** como sistema nervioso, **ROS-Industrial** como cerebro robótico, y un **gemelo digital en Webots** que corre el mismo código que correría una planta real. De la caja roja a la estación de recolección, sin tocar un solo cable.*

*Gracias.*

---

### ⏱️ Resumen de tiempos
| Bloque | Tiempo | Contenido |
|---|---|---|
| Apertura | 0:00–1:30 | la fábrica conectada |
| El ciclo | 1:30–3:00 | qué construimos |
| **Parte A** | 3:00–6:00 | MQTT + demo start/stop |
| **Parte B** | 6:00–9:00 | ROS-Industrial + Nav 2 |
| **Parte C** | 9:00–11:30 | gemelo digital + por qué Webots |
| Demo | 11:30–13:30 | ciclo completo en vivo |
| Cierre | 13:30–14:45 | edge/cloud + conclusión |
