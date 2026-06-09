#!/usr/bin/env python3
"""Panel de control (Tkinter) del clasificador rojo.

Botones del ciclo:
  ▶ Iniciar / Reanudar  -> /banda/comando "start"   (arranca la banda y el ciclo)
  ⏸ Pausar              -> /banda/comando "stop"    (detiene la banda / pausa el ciclo)
  ⟳ Reiniciar           -> /banda/comando "reset"   (caja a la banda + robot al dock -> repite)

Monitor de condición (mantenimiento predictivo) — del nodo `monitor_condicion`:
  - 🌡 temperatura del motor en vivo (lectura + barra + gráfica desplazable con umbrales),
  - foco de ALARMA por sobre-temperatura,
  - botón ❄ Enfriamiento (manual) -> /planta/enfriamiento_cmd, y foco del ventilador (AUTO/ON).

Muestra además el estado del orquestador (/estado) y si la cámara ve la caja (/caja_roja).

rclpy spinea en un hilo aparte; los callbacks solo guardan valores y un after() de Tkinter (hilo
principal, único seguro para la GUI) refresca todo. Publicar desde el hilo de Tkinter es seguro.
"""
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

import tkinter as tk

# Umbrales (deben coincidir con monitor_condicion.py) — solo para dibujar.
T_WARN, T_ALARM = 70.0, 78.0
T_LO, T_HI = 20.0, 105.0   # rango vertical de la gráfica/barra

BG = "#1e1e1e"
GW, GH = 426, 150          # tamaño de la gráfica
BW, BH = 426, 20           # tamaño de la barra


def zona_color(t):
    if t >= T_ALARM:
        return "#ea4335"   # rojo
    if t >= T_WARN:
        return "#f9ab00"   # naranja
    return "#34a853"       # verde


class PanelNode(Node):
    def __init__(self):
        super().__init__("control_gui")
        self.pub_cmd = self.create_publisher(String, "/banda/comando", 10)
        self.pub_fan = self.create_publisher(Bool, "/planta/enfriamiento_cmd", 10)

        self.estado = "(esperando…)"
        self.caja = False
        self.temp = 25.0
        self.alarma = False
        self.fan_on = False
        self.fan_manual = False

        self.create_subscription(String, "/estado", self._on_estado, 10)
        self.create_subscription(Bool, "/caja_roja", self._on_caja, 10)
        self.create_subscription(Float32, "/planta/temperatura", self._on_temp, 10)
        self.create_subscription(Bool, "/planta/alarma", self._on_alarma, 10)
        self.create_subscription(Bool, "/planta/enfriamiento", self._on_fan, 10)

    def _on_estado(self, m): self.estado = m.data
    def _on_caja(self, m): self.caja = bool(m.data)
    def _on_temp(self, m): self.temp = float(m.data)
    def _on_alarma(self, m): self.alarma = bool(m.data)
    def _on_fan(self, m): self.fan_on = bool(m.data)

    def enviar(self, cmd):
        m = String(); m.data = cmd
        self.pub_cmd.publish(m)
        self.get_logger().info(f"comando -> {cmd}")

    def toggle_enfriamiento(self):
        self.fan_manual = not self.fan_manual
        self.pub_fan.publish(Bool(data=self.fan_manual))
        self.get_logger().info(f"enfriamiento manual -> {'ON' if self.fan_manual else 'OFF'}")


def main():
    rclpy.init()
    node = PanelNode()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    root = tk.Tk()
    root.title("Clasificador Rojo — Panel de Control")
    root.configure(bg=BG)
    root.geometry("470x712")

    tk.Label(root, text="Clasificador Rojo", bg=BG, fg="#ffffff",
             font=("Helvetica", 16, "bold")).pack(pady=(12, 2))
    tk.Label(root, text="banda · cámara · UR5e · TurtleBot/Nav2", bg=BG,
             fg="#9aa0a6", font=("Helvetica", 9)).pack(pady=(0, 8))

    botones = tk.Frame(root, bg=BG); botones.pack()

    def boton(parent, txt, color, cmd):
        b = tk.Button(parent, text=txt, font=("Helvetica", 12, "bold"), bg=color, fg="white",
                      activebackground=color, relief="flat", width=20, height=1,
                      command=cmd)
        b.pack(pady=3)
        return b

    boton(botones, "▶  Iniciar / Reanudar", "#188038", lambda: node.enviar("start"))
    boton(botones, "⏸  Pausar", "#b06000", lambda: node.enviar("stop"))
    boton(botones, "⟳  Reiniciar ciclo", "#1a73e8", lambda: node.enviar("reset"))

    estado_lbl = tk.Label(root, text="", bg=BG, fg="#8ab4f8", font=("Consolas", 11))
    estado_lbl.pack(pady=(10, 1))
    caja_lbl = tk.Label(root, text="", bg=BG, fg="#9aa0a6", font=("Consolas", 10))
    caja_lbl.pack(pady=(0, 6))

    # ---------------- Monitor de condición ----------------
    tk.Frame(root, bg="#333333", height=1, width=440).pack(pady=4)
    tk.Label(root, text="🌡  MONITOR DE CONDICIÓN — motor de la banda", bg=BG, fg="#e8eaed",
             font=("Helvetica", 11, "bold")).pack(pady=(2, 2))

    temp_lbl = tk.Label(root, text="-- °C", bg=BG, fg="#34a853", font=("Consolas", 22, "bold"))
    temp_lbl.pack()
    barra = tk.Canvas(root, width=BW, height=BH, bg=BG, highlightthickness=0)
    barra.pack(pady=(2, 4))
    grafica = tk.Canvas(root, width=GW, height=GH, bg="#0e0e0e", highlightthickness=1,
                        highlightbackground="#333")
    grafica.pack(pady=(0, 6))

    fila = tk.Frame(root, bg=BG); fila.pack(pady=(0, 4))
    alarma_lbl = tk.Label(fila, text="  OK  ", bg="#244032", fg="#9aa0a6",
                          font=("Consolas", 11, "bold"), width=14)
    alarma_lbl.grid(row=0, column=0, padx=6)
    fan_lbl = tk.Label(fila, text="ventilador ○", bg=BG, fg="#9aa0a6", font=("Consolas", 10))
    fan_lbl.grid(row=0, column=1, padx=6)

    fan_btn = tk.Button(root, text="❄  Enfriamiento: OFF", font=("Helvetica", 11, "bold"),
                        bg="#3c4043", fg="white", activebackground="#3c4043", relief="flat",
                        width=22, height=1, command=node.toggle_enfriamiento)
    fan_btn.pack(pady=(0, 8))

    hist = deque(maxlen=GW)   # 1 muestra/píxel de ancho

    def y_of(t):
        return GH - (t - T_LO) / (T_HI - T_LO) * GH

    def dibuja_barra(t):
        barra.delete("all")
        frac = max(0.0, min(1.0, (t - T_LO) / (T_HI - T_LO)))
        barra.create_rectangle(0, 0, BW, BH, fill="#101010", outline="#444")
        barra.create_rectangle(0, 0, int(BW * frac), BH, fill=zona_color(t), outline="")
        for thr, c in ((T_WARN, "#f9ab00"), (T_ALARM, "#ea4335")):
            x = int(BW * (thr - T_LO) / (T_HI - T_LO))
            barra.create_line(x, 0, x, BH, fill=c, width=2)

    def dibuja_grafica():
        grafica.delete("all")
        # franjas de umbral (fondo)
        grafica.create_rectangle(0, 0, GW, y_of(T_ALARM), fill="#241316", outline="")
        grafica.create_rectangle(0, y_of(T_ALARM), GW, y_of(T_WARN), fill="#241f10", outline="")
        # rejilla + etiquetas del eje Y (°C)
        for v in (40, 60, 80, 100):
            yy = y_of(v)
            grafica.create_line(0, yy, GW, yy, fill="#222")
            grafica.create_text(2, yy - 6, anchor="w", text=str(v), fill="#666", font=("Consolas", 7))
        # líneas de umbral WARN / ALARM
        for thr, c, txt in ((T_WARN, "#f9ab00", "WARN 70°"), (T_ALARM, "#ea4335", "ALARM 78°")):
            yy = y_of(thr)
            grafica.create_line(0, yy, GW, yy, fill=c, dash=(4, 3))
            grafica.create_text(GW - 4, yy - 7, anchor="e", text=txt, fill=c, font=("Consolas", 8))
        n = len(hist)
        if n >= 2:
            step = GW / max(1, n - 1)
            pts = []
            cool_any = False
            for i, (t, fan) in enumerate(hist):
                x = i * step
                pts += [x, y_of(t)]
                if fan:                       # banda cian abajo = enfriamiento activo en ese instante
                    cool_any = True
                    grafica.create_line(x, GH - 5, x, GH - 1, fill="#26c6da", width=max(2, int(step) + 1))
            grafica.create_line(*pts, fill=zona_color(hist[-1][0]), width=2)
            if cool_any:
                grafica.create_text(5, GH - 13, anchor="w", text="❄ enfriando",
                                    fill="#26c6da", font=("Consolas", 8))
        grafica.create_text(GW - 4, 9, anchor="e", text="°C vs tiempo →", fill="#777",
                            font=("Consolas", 7))

    blink = {"on": False}

    def refrescar():
        estado_lbl.config(text=f"estado: {node.estado}")
        caja_lbl.config(text="● CAJA ROJA detectada" if node.caja else "○ sin caja a la vista",
                        fg="#ea4335" if node.caja else "#9aa0a6")

        t = node.temp
        hist.append((t, node.fan_on))
        temp_lbl.config(text=f"{t:4.1f} °C", fg=zona_color(t))
        dibuja_barra(t)
        dibuja_grafica()

        if node.alarma:
            blink["on"] = not blink["on"]
            alarma_lbl.config(text="⚠ ALARMA TEMP", bg="#ea4335" if blink["on"] else "#5c1a14",
                              fg="white")
        else:
            blink["on"] = False
            alarma_lbl.config(text="  OK  ", bg="#244032", fg="#9aa0a6")

        if node.fan_on:
            modo = "AUTO" if not node.fan_manual else "MAN"
            fan_lbl.config(text=f"ventilador ● {modo}", fg="#26c6da")
        else:
            fan_lbl.config(text="ventilador ○", fg="#9aa0a6")
        fan_btn.config(text=f"❄  Enfriamiento: {'ON' if node.fan_manual else 'OFF'}",
                       bg="#0b7c8c" if node.fan_manual else "#3c4043")

        root.after(150, refrescar)

    def cerrar():
        if rclpy.ok():
            rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", cerrar)
    refrescar()
    root.mainloop()


if __name__ == "__main__":
    main()
