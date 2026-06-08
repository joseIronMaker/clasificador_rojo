#!/usr/bin/env python3
"""Genera config/mapa_celda.pgm para Nav2 a partir de la geometría CONOCIDA de las paredes del mundo.

No usa SLAM: como las paredes (DEF WALL_* en worlds/mundo_banda.wbt) las colocamos nosotros, sus
posiciones se conocen exactamente y se dibujan directo como ocupadas (valor 0) sobre un fondo libre
(254). El resultado coincide pixel a pixel con el .wbt, así el planner global de Nav2 conoce las
paredes desde el arranque (no solo cuando el lidar las ve). Convención map_server: origen = esquina
INFERIOR-izquierda, la fila 0 del PGM es la y MÁS ALTA.

Reejecutar tras mover/añadir paredes:  python3 scripts/genera_mapa.py
"""
import os

RES = 0.05            # m/px  (igual que mapa_celda.yaml)
OX, OY = -5.0, -5.0   # origin del yaml (esquina inferior-izquierda)
W = H = 200           # 10 m x 10 m
FREE, OCC = 254, 0

# Paredes como rectángulos en coords del MUNDO [x0,x1]x[y0,y1] (= los Box del .wbt, centro±size/2).
WALLS = [
    (-3.00, 1.70,  0.95, 1.05),   # WALL_N  centro(-0.65, 1.0) size(4.7,0.1)
    (-3.00, 1.70, -2.55, -2.45),  # WALL_S  centro(-0.65,-2.5) size(4.7,0.1)
    ( 1.65, 1.75, -2.50, 1.00),   # WALL_E  centro( 1.7,-0.75) size(0.1,3.5)
    (-3.05, -2.95, -2.50, 1.00),  # WALL_W  centro(-3.0,-0.75) size(0.1,3.5)
]

grid = [[FREE] * W for _ in range(H)]

def fill(x0, x1, y0, y1):
    c0 = int(round((x0 - OX) / RES)); c1 = int(round((x1 - OX) / RES))
    rb0 = int(round((y0 - OY) / RES)); rb1 = int(round((y1 - OY) / RES))
    for rb in range(rb0, rb1 + 1):
        r = (H - 1) - rb                      # fila del PGM (0 = arriba = y máx)
        if 0 <= r < H:
            for c in range(c0, c1 + 1):
                if 0 <= c < W:
                    grid[r][c] = OCC

for w in WALLS:
    fill(*w)

out = os.path.join(os.path.dirname(__file__), "..", "config", "mapa_celda.pgm")
out = os.path.abspath(out)
with open(out, "wb") as f:
    f.write(b"P5\n# celda de trabajo (paredes) para Nav2 -- generado por scripts/genera_mapa.py\n")
    f.write(b"%d %d\n255\n" % (W, H))
    f.write(bytes(v for row in grid for v in row))

occ = sum(row.count(OCC) for row in grid)
print(f"escrito {out}: {W}x{H} px, {occ} px ocupados (paredes), resto libre")
