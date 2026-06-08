#!/usr/bin/env python3
"""Panel de control (Tkinter) del clasificador rojo.

Botones:
  ▶ Iniciar / Reanudar  -> /banda/comando "start"   (arranca la banda y el ciclo)
  ⏸ Pausar              -> /banda/comando "stop"    (detiene la banda / pausa el ciclo)
  ⟳ Reiniciar           -> /banda/comando "reset"   (caja a la banda + robot al dock -> repite)

Muestra en vivo: el estado del orquestador (/estado) y si la cámara detecta la caja (/caja_roja).

NOTA: "Pausar" pausa el CICLO (banda + actividad), no congela Webots — un controlador no puede
re-arrancar la simulación desde ROS. Para congelar la VISTA por completo, usa la barra espaciadora
en la ventana de Webots.

rclpy spinea en un hilo aparte; los callbacks solo guardan valores y un after() de Tkinter (hilo
principal, único seguro para la GUI) refresca las etiquetas. Publicar desde el hilo de Tkinter es
seguro (los publishers de rclpy son thread-safe).
"""
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

import tkinter as tk


class PanelNode(Node):
    def __init__(self):
        super().__init__("control_gui")
        self.pub_cmd = self.create_publisher(String, "/banda/comando", 10)
        self.estado = "(esperando…)"
        self.caja = False
        self.create_subscription(String, "/estado", self._on_estado, 10)
        self.create_subscription(Bool, "/caja_roja", self._on_caja, 10)

    def _on_estado(self, m):
        self.estado = m.data

    def _on_caja(self, m):
        self.caja = bool(m.data)

    def enviar(self, cmd):
        m = String()
        m.data = cmd
        self.pub_cmd.publish(m)
        self.get_logger().info(f"comando -> {cmd}")


def main():
    rclpy.init()
    node = PanelNode()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    root = tk.Tk()
    root.title("Clasificador Rojo — Panel de Control")
    root.configure(bg="#1e1e1e")
    root.geometry("360x340")

    tk.Label(root, text="Clasificador Rojo", bg="#1e1e1e", fg="#ffffff",
             font=("Helvetica", 16, "bold")).pack(pady=(14, 2))
    tk.Label(root, text="banda · cámara · UR5e · TurtleBot/Nav2", bg="#1e1e1e",
             fg="#9aa0a6", font=("Helvetica", 9)).pack(pady=(0, 10))

    def boton(txt, color, cmd):
        b = tk.Button(root, text=txt, font=("Helvetica", 13, "bold"), bg=color, fg="white",
                      activebackground=color, relief="flat", width=22, height=2,
                      command=lambda: node.enviar(cmd))
        b.pack(pady=5)
        return b

    boton("▶  Iniciar / Reanudar", "#188038", "start")
    boton("⏸  Pausar", "#b06000", "stop")
    boton("⟳  Reiniciar ciclo", "#1a73e8", "reset")

    estado_lbl = tk.Label(root, text="", bg="#1e1e1e", fg="#8ab4f8", font=("Consolas", 11))
    estado_lbl.pack(pady=(14, 2))
    caja_lbl = tk.Label(root, text="", bg="#1e1e1e", fg="#9aa0a6", font=("Consolas", 10))
    caja_lbl.pack()

    def refrescar():
        estado_lbl.config(text=f"estado: {node.estado}")
        if node.caja:
            caja_lbl.config(text="● CAJA ROJA detectada", fg="#ea4335")
        else:
            caja_lbl.config(text="○ sin caja a la vista", fg="#9aa0a6")
        root.after(200, refrescar)

    def cerrar():
        rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", cerrar)
    refrescar()
    root.mainloop()


if __name__ == "__main__":
    main()
