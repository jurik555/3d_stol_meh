#!/usr/bin/env python3
"""Параметрический генератор 3D-модели П-образного диванного столика.

Конструкция (вид сбоку):

      ┌──────────────── столешница ───────────────┐
      │◎ поворотный узел (ось M16, фиксатор 0°/90°)
     ┌┴┐
     │║│  двойная стойка: доска A + бруски 30×30 + доска B,
     │║│  внутри — мачта с зубчатой рейкой, собачка с рычагом,
     │║│  газовый упор
     └┬┘
      └──────────────── основание (под диван) ────┘

Механизм подъёма: зубчатая рейка на мачте + подпружиненная собачка на стойке
(храповик: вверх — просто тянуть, вниз — нажать рычаг) + газовый упор,
частично компенсирующий вес.

Оси: X — от стойки к человеку (вдоль столешницы), Y — ширина доски, Z — вверх.
Все размеры в мм.

Запуск:  python generate.py [--params params.json] [--out output] [--set key=value ...]
"""

import argparse
import json
import math
import os
import sys
from dataclasses import dataclass

import cadquery as cq

# ----------------------------------------------------------------------------
# Параметры
# ----------------------------------------------------------------------------

DEFAULTS = {
    "width": 300,             # ширина основания, стойки и столешницы
    "board_t": 18,            # толщина фанеры
    "table_height_min": 600,  # высота верха столешницы в нижнем положении
    "travel": 150,            # ход подъёма
    "top_length": 500,        # длина столешницы
    "base_length": 500,       # длина основания (заходит под диван)
    "spacer_w": 30,           # ширина боковых брусков стойки
    "mast_t": 18,             # толщина мачты (фанера)
    "rack_t": 10,             # толщина рейки
    "pawl_t": 8,              # толщина собачки
    "clearance": 1,           # зазор скольжения
    "tooth_pitch": 10,        # шаг зубьев (= шаг регулировки)
    "tooth_depth": 5,         # высота зуба
    "cap_h": 60,              # высота поворотного блока
    "flange_d": 120,          # диаметр фланца под столешницей
    "flange_t": 4,
    "rotation_deg": 90,       # угол отвода столешницы
    "gas_spring_length": 400, # длина газового упора (между ушками, выдвинут)
    "gas_spring_stroke": 150,
    "gas_spring_body_d": 22,
    "gas_spring_rod_d": 8,
    "gusset": 100,            # косынки стойка-основание (0 — без косынок)
}

DENSITY = {  # кг/м³
    "фанера": 650,
    "бук/клеёная фанера": 700,
    "сталь": 7850,
    "алюминий": 2700,
    "PETG/пластик": 1270,
    "PTFE": 2200,
}

COLORS = {
    "wood": (0.85, 0.68, 0.45),
    "wood_dark": (0.70, 0.52, 0.32),
    "wood_top": (0.92, 0.80, 0.60),
    "steel": (0.62, 0.64, 0.68),
    "alu": (0.78, 0.80, 0.84),
    "black": (0.15, 0.15, 0.17),
    "red": (0.80, 0.18, 0.15),
    "white": (0.95, 0.95, 0.95),
    "spring": (0.25, 0.45, 0.80),
}


@dataclass
class Part:
    name: str
    shape: cq.Workplane
    color: str
    material: str
    descr: str


class Geo:
    """Производные размеры."""

    def __init__(self, p):
        self.p = p
        self.W = p["width"]
        self.T = p["board_t"]
        c = p["clearance"]
        self.pitch = p["tooth_pitch"]
        self.td = p["tooth_depth"]
        # ход кратен шагу зуба
        self.travel = int(p["travel"] // self.pitch) * self.pitch
        self.Gap = p["mast_t"] + p["rack_t"] + 2 * c          # полость между досками
        self.D = 2 * self.T + self.Gap                        # глубина стойки по X
        self.ax, self.ay = self.D / 2, self.W / 2             # ось поворота
        self.table_bot = p["table_height_min"] - self.T
        self.hinge_gap = p["flange_t"] + 1                    # фланец + шайба PTFE
        self.cap_top = self.table_bot - self.hinge_gap
        self.stand_top = self.cap_top - p["cap_h"]
        self.stand_h = self.stand_top - self.T
        self.tenon = 20                                       # шип мачты в блоке

        self.Ax0, self.Ax1 = 0, self.T
        self.Bx0, self.Bx1 = self.T + self.Gap, self.D
        self.mast_x0 = self.T + c
        self.mast_x1 = self.mast_x0 + p["mast_t"]
        self.rack_x0 = self.mast_x1
        self.rack_x1 = self.rack_x0 + p["rack_t"]
        self.pawl_x0 = self.rack_x0 + (p["rack_t"] - p["pawl_t"]) / 2
        self.pawl_x1 = self.pawl_x0 + p["pawl_t"]

        sw = p["spacer_w"]
        self.mast_y0, self.mast_y1 = sw + c, self.W - sw - c
        self.mast_z0 = self.T + 2
        self.mast_z1 = self.stand_top + self.tenon

        # рейка/собачка — у правого бруска
        self.rack_edge_y = self.W - sw - 65
        self.rack_y0 = self.rack_edge_y - 20
        self.piv_y = self.rack_edge_y + 35
        self.z_tip = self.stand_top - 50
        self.piv_z = self.z_tip - 30
        self.tail_end_y = self.W + 20

        # газовый упор — в окне мачты
        self.gs_x = self.T + self.Gap / 2
        self.gs_y = self.mast_y0 + 70
        self.win_y0, self.win_y1 = self.gs_y - 25, self.gs_y + 25
        self.gs_z0 = self.T + 15
        L, S = p["gas_spring_length"], p["gas_spring_stroke"]
        self.gs_eye0 = self.gs_z0 + L - self.travel           # верхнее ушко при h=0
        self.win_top = self.gs_eye0 + 20

        self.check()

    def check(self):
        p = self.p
        err = []
        if p["gas_spring_stroke"] < self.travel:
            err.append("ход газового упора меньше хода стола")
        if self.win_y1 + 10 > self.rack_y0:
            err.append("слишком узкая доска: окно газового упора налезает на рейку")
        if self.win_top + 30 > self.stand_top:
            err.append("газовый упор слишком длинный для стойки")
        if self.stand_top - self.mast_z0 - self.travel < 150:
            err.append("малое перекрытие мачты в стойке на верхнем положении (<150 мм)")
        if p["gas_spring_body_d"] > self.Gap - 4:
            err.append("газовый упор толще полости")
        if self.mast_z0 + self.travel + 0 > self.stand_top:
            err.append("ход больше высоты стойки")
        if err:
            sys.exit("Ошибка параметров:\n  - " + "\n  - ".join(err))


# ----------------------------------------------------------------------------
# Геометрические помощники
# ----------------------------------------------------------------------------

def box(x0, x1, y0, y1, z0, z1):
    return (cq.Workplane("XY")
            .box(x1 - x0, y1 - y0, z1 - z0, centered=False)
            .translate((x0, y0, z0)))


def cyl_z(x, y, z0, z1, d):
    return cq.Workplane("XY", origin=(x, y, z0)).circle(d / 2).extrude(z1 - z0)


def cyl_x(y, z, x0, x1, d):
    return cq.Workplane("YZ", origin=(x0, 0, 0)).moveTo(y, z).circle(d / 2).extrude(x1 - x0)


def bar2d(p1, p2, w):
    (x1, y1), (x2, y2) = p1, p2
    L = math.hypot(x2 - x1, y2 - y1)
    nx, ny = -(y2 - y1) / L * w / 2, (x2 - x1) / L * w / 2
    return [(x1 + nx, y1 + ny), (x2 + nx, y2 + ny), (x2 - nx, y2 - ny), (x1 - nx, y1 - ny)]


def yz_frame(x0, y0, z0):
    """Локальная плоскость: u → +Y, v → +Z, толщина → +X."""
    return cq.Location(cq.Plane(origin=(x0, y0, z0), xDir=(0, 1, 0), normal=(1, 0, 0)))


def place(wp_local, loc):
    return cq.Workplane("XY").add(wp_local.val().moved(loc))


# ----------------------------------------------------------------------------
# 2D-профили механизма (используются и для 3D, и для DXF)
# ----------------------------------------------------------------------------

def rack_profile(g):
    """Рейка: u — от рабочей кромки (+u — зубья), v — Z в системе мачты (h=0)."""
    p, d = g.pitch, g.td
    n = g.travel // p
    faces = sorted(g.z_tip - k * p for k in range(-1, n + 2))
    pts = [(-20, faces[0])]
    for z0 in faces:
        pts += [(0, z0), (d, z0)]
    pts += [(0, faces[-1] + p), (-20, faces[-1] + p)]
    return pts


def rack_local(g):
    return cq.Workplane("XY").polyline(rack_profile(g)).close().extrude(g.p["rack_t"])


def pawl_local(g, t):
    """Собачка в локальных координатах (0,0) — ось вращения."""
    p, d = g.pitch, g.td
    e = g.rack_edge_y - g.piv_y          # кромка рейки (отрицательно)
    vt = g.z_tip - g.piv_z               # верх зуба собачки
    tip = [(e + 0.8, vt - 0.3), (e + d + 7, vt - 0.3),
           (e + d + 7, vt - 0.9 * p), (e + d + 3, vt - 0.9 * p)]
    arm = bar2d((e + d + 5, vt - 0.45 * p), (0, 0), 10)
    tail_len = g.tail_end_y - g.piv_y
    tail = bar2d((0, 0), (tail_len, 0), 12)
    s = cq.Workplane("XY").polyline(tip).close().extrude(t)
    s = s.union(cq.Workplane("XY").polyline(arm).close().extrude(t))
    s = s.union(cq.Workplane("XY").polyline(tail).close().extrude(t))
    s = s.union(cq.Workplane("XY").circle(10).extrude(t))
    s = s.union(cq.Workplane("XY").moveTo(tail_len, 0).circle(6).extrude(t))
    s = s.cut(cq.Workplane("XY").circle(4.2).extrude(t))
    return s


def arc_slot(r, a0, a1, w, t, n=24):
    """Дуговой паз в плоскости XY (углы в градусах)."""
    ang = [math.radians(a0 + (a1 - a0) * i / n) for i in range(n + 1)]
    outer = [((r + w / 2) * math.cos(a), (r + w / 2) * math.sin(a)) for a in ang]
    inner = [((r - w / 2) * math.cos(a), (r - w / 2) * math.sin(a)) for a in reversed(ang)]
    s = cq.Workplane("XY").polyline(outer + inner).close().extrude(t)
    for a in (ang[0], ang[-1]):
        s = s.union(cq.Workplane("XY").moveTo(r * math.cos(a), r * math.sin(a)).circle(w / 2).extrude(t))
    return s


def flange_local(g):
    p = g.p
    rot = p["rotation_deg"]
    t = p["flange_t"]
    f = cq.Workplane("XY").circle(p["flange_d"] / 2).extrude(t)
    f = f.cut(cq.Workplane("XY").circle(8.25).extrude(t))                   # M16
    f = f.cut(arc_slot(40, -90 - rot, -90, 10, t))                          # паз упора
    for a in (90, 90 - rot):                                                # фиксатор
        a = math.radians(a)
        f = f.cut(cq.Workplane("XY").moveTo(30 * math.cos(a), 30 * math.sin(a)).circle(3).extrude(t))
    r = p["flange_d"] / 2 - 8
    for a in (0, 90, 180, 270):                                             # шурупы
        a = math.radians(a + 22.5)
        f = f.cut(cq.Workplane("XY").moveTo(r * math.cos(a), r * math.sin(a)).circle(2.5).extrude(t))
    return f


# ----------------------------------------------------------------------------
# Сборка
# ----------------------------------------------------------------------------

def build(g, h=0, theta=0):
    """h — подъём (кратно шагу), theta — поворот столешницы, градусы."""
    p, T, W, D = g.p, g.T, g.W, g.D
    sw = p["spacer_w"]
    parts = []

    def add(name, shape, color, material, descr):
        parts.append(Part(name, shape, color, material, descr))

    # --- неподвижная часть ---------------------------------------------------
    add("base", box(0, p["base_length"], 0, W, 0, T), "wood", "фанера",
        f"Основание {p['base_length']}×{W}×{T}")

    board_a = box(g.Ax0, g.Ax1, 0, W, T, g.stand_top)
    add("stand_board_A", board_a, "wood", "фанера",
        f"Доска стойки A (со стороны спинки) {g.stand_h:.0f}×{W}×{T}")

    board_b = box(g.Bx0, g.Bx1, 0, W, T, g.stand_top)
    board_b = board_b.cut(cyl_x(g.piv_y, g.piv_z, g.Bx0, g.Bx1, 8.5))
    add("stand_board_B", board_b, "wood", "фанера",
        f"Доска стойки B (со стороны дивана) {g.stand_h:.0f}×{W}×{T}, отв. Ø8.5 под ось собачки")

    add("spacer_left", box(T, T + g.Gap, 0, sw, T, g.stand_top), "wood_dark", "фанера",
        f"Брусок стойки левый {g.stand_h:.0f}×{sw}×{g.Gap}")
    spacer_r = box(T, T + g.Gap, W - sw, W, T, g.stand_top)
    spacer_r = spacer_r.cut(box(g.rack_x0, g.Bx0 + 0.01, W - sw - 1, W + 1,
                                g.piv_z - 22, g.piv_z + 10))
    add("spacer_right", spacer_r, "wood_dark", "фанера",
        f"Брусок стойки правый {g.stand_h:.0f}×{sw}×{g.Gap}, паз под рычаг 32×{g.Bx0 - g.rack_x0:.0f}")

    if p["gusset"] > 0:
        gs = p["gusset"]
        tri = [(D, T), (D + gs, T), (D, T + gs)]
        for nm, y1 in (("gusset_left", T), ("gusset_right", W)):
            s = cq.Workplane("XZ", origin=(0, y1, 0)).polyline(tri).close().extrude(T)
            add(nm, s, "wood_dark", "фанера", f"Косынка {gs}×{gs}×{T}")

    # направляющие (PTFE/UHMW или бук, на доске B)
    add("guide_left", box(g.rack_x0 + 0.5, g.Bx0, g.mast_y0 + 1, g.win_y0 - 5,
                          T + 5, g.stand_top - 5),
        "white", "PTFE", "Направляющая планка левая")
    add("guide_right", box(g.rack_x0 + 0.5, g.Bx0, W - sw - 24, W - sw - 1,
                           T + 5, g.piv_z - 45),
        "white", "PTFE", "Направляющая планка правая")

    # собачка, ось, пружина, ручка
    loc_pawl = yz_frame(g.pawl_x0, g.piv_y, g.piv_z)
    add("pawl", place(pawl_local(g, p["pawl_t"]), loc_pawl), "red", "сталь",
        f"Собачка храповика, лист {p['pawl_t']} мм (DXF)")
    tail_len = g.tail_end_y - g.piv_y
    pc = (g.pawl_x0 + g.pawl_x1) / 2
    add("lever_knob", box(pc - 10, pc + 10, g.piv_y + tail_len - 12, g.piv_y + tail_len + 12,
                          g.piv_z - 10, g.piv_z + 10).edges("|X").fillet(4),
        "black", "PETG/пластик", "Накладка-ручка рычага (печать)")
    pivot = cyl_x(g.piv_y, g.piv_z, g.pawl_x0, g.Bx1, 8)
    pivot = pivot.union(cyl_x(g.piv_y, g.piv_z, g.Bx1, g.Bx1 + 6, 14))
    pivot = pivot.union(cyl_x(g.piv_y, g.piv_z, g.pawl_x1, g.Bx0, 16))  # втулка-шайба
    add("pawl_pivot", pivot, "steel", "сталь", "Ось собачки: болт M8×40 + втулка-проставка")
    blk_y0 = g.piv_y + 10
    add("spring_block", box(g.pawl_x0, g.Bx0, blk_y0, blk_y0 + 16, g.piv_z - 30, g.piv_z - 18),
        "wood_dark", "бук/клеёная фанера", "Упор пружины собачки")
    add("pawl_spring", cyl_z(pc, blk_y0 + 8, g.piv_z - 18, g.piv_z - 6, 7), "spring", "сталь",
        "Пружина сжатия Ø7×15, проволока 0.8–1")

    # газовый упор: нижний кронштейн
    add("gs_bracket_low", box(g.gs_x - 9, g.gs_x + 9, g.gs_y - 10, g.gs_y + 10, T, g.gs_z0 + 4),
        "steel", "сталь", "Кронштейн газового упора нижний")
    L = p["gas_spring_length"] - g.travel + h
    body_len = p["gas_spring_length"] - p["gas_spring_stroke"] - 10
    add("gas_spring_body", cyl_z(g.gs_x, g.gs_y, g.gs_z0, g.gs_z0 + body_len, p["gas_spring_body_d"]),
        "black", "сталь", f"Газовый упор {p['gas_spring_length']} мм, ход {p['gas_spring_stroke']}")
    add("gas_spring_rod", cyl_z(g.gs_x, g.gs_y, g.gs_z0 + body_len - 5, g.gs_z0 + L, p["gas_spring_rod_d"]),
        "steel", "сталь", "Шток газового упора")

    # --- подвижная часть (поднимается на h) ----------------------------------
    up = lambda s: s.translate((0, 0, h))
    mast = box(g.mast_x0, g.mast_x1, g.mast_y0, g.mast_y1, g.mast_z0, g.mast_z1)
    mast = mast.cut(box(g.mast_x0 - 1, g.mast_x1 + 1, g.win_y0, g.win_y1, g.mast_z0 - 1, g.win_top))
    add("mast", up(mast), "wood", "фанера",
        f"Мачта {g.mast_z1 - g.mast_z0:.0f}×{g.mast_y1 - g.mast_y0:.0f}×{p['mast_t']}, "
        f"паз снизу {g.win_y1 - g.win_y0:.0f}×{g.win_top - g.mast_z0:.0f}")
    add("rack", up(place(rack_local(g), yz_frame(g.rack_x0, g.rack_edge_y, 0))), "alu", "алюминий",
        f"Зубчатая рейка, лист {p['rack_t']} мм, шаг {g.pitch} (DXF)")
    add("gs_bracket_top", up(box(g.gs_x - 10, g.gs_x + 10, g.gs_y - 12, g.gs_y + 12,
                                 g.gs_eye0 - 6, g.win_top)),
        "steel", "сталь", "Кронштейн газового упора верхний (на мачте)")

    cb = g.stand_top + h
    cap = box(0, D, 0, W, cb, cb + p["cap_h"])
    cap = cap.cut(box(g.mast_x0, g.mast_x1, g.mast_y0, g.mast_y1, cb - 1, cb + g.tenon))
    cap = cap.cut(box(g.ax - 15, D + 1, g.ay - 18, g.ay + 18, cb + g.tenon, cb + g.tenon + 16))
    cap = cap.cut(cyl_z(g.ax, g.ay, cb + g.tenon, cb + p["cap_h"] + 1, 20))
    cap = cap.cut(cyl_z(g.ax, g.ay + 30, cb + p["cap_h"] - 14, cb + p["cap_h"] + 1, 8))
    cap = cap.cut(cyl_z(g.ax, g.ay - 40, cb + p["cap_h"] - 16, cb + p["cap_h"] + 1, 8))
    add("swivel_block", cap, "wood_dark", "бук/клеёная фанера",
        f"Поворотный блок {D:.0f}×{W}×{p['cap_h']} (бук или 3–4 слоя фанеры)")
    ct = cb + p["cap_h"]
    add("bushing", cyl_z(g.ax, g.ay, cb + g.tenon + 16, ct, 20).cut(cyl_z(g.ax, g.ay, cb, ct + 1, 16)),
        "alu", "сталь", "Втулка бронзовая 16×20×24")
    add("ball_plunger", cyl_z(g.ax, g.ay + 30, ct - 14, ct + 1.5, 8), "steel", "сталь",
        "Шариковый фиксатор M8 (0° и 90°)")
    add("stop_pin", cyl_z(g.ax, g.ay - 40, ct - 16, ct + 1 + p["flange_t"], 8), "steel", "сталь",
        "Штифт-ограничитель Ø8")
    add("ptfe_washer", cyl_z(g.ax, g.ay, ct, ct + 1, 48).cut(cyl_z(g.ax, g.ay, ct - 1, ct + 2, 16)),
        "white", "PTFE", "Шайба PTFE Ø48/16×1")
    nz = cb + g.tenon + 2
    add("nut_m16", cyl_z(g.ax, g.ay, nz, nz + 13, 24), "steel", "сталь",
        "Гайка M16 самоконтрящаяся + шайба")

    # --- вращающаяся часть ---------------------------------------------------
    def turn(s):
        return up(s).rotate((g.ax, g.ay, 0), (g.ax, g.ay, 1), theta)

    fl = flange_local(g).translate((g.ax, g.ay, g.cap_top + 1))
    add("flange", turn(fl), "steel", "сталь",
        f"Фланец Ø{p['flange_d']}×{p['flange_t']} (DXF), M16 приварен/вкручен")
    add("bolt_m16", turn(cyl_z(g.ax, g.ay, g.stand_top + g.tenon + 2, g.cap_top + 1 + p["flange_t"], 16)),
        "steel", "сталь", "Ось поворота: болт/шпилька M16")
    top = box(0, p["top_length"], 0, W, g.table_bot, g.table_bot + T).edges("|Z").fillet(25)
    add("tabletop", turn(top), "wood_top", "фанера", f"Столешница {p['top_length']}×{W}×{T}")

    return parts


# ----------------------------------------------------------------------------
# Экспорт
# ----------------------------------------------------------------------------

def export_assembly(parts, path_base):
    a = cq.Assembly(name="sofa_table")
    for pt in parts:
        a.add(pt.shape, name=pt.name, color=cq.Color(*COLORS[pt.color]))
    a.export(path_base + ".step")
    a.export(path_base + ".glb")


def _subdivide(polys, max_edge):
    """Делит длинные треугольники пополам, чтобы сортировка по глубине не ошибалась."""
    import numpy as np
    done = []
    while len(polys):
        e = np.stack([np.linalg.norm(polys[:, (i + 1) % 3] - polys[:, i], axis=1) for i in range(3)], 1)
        big = e.max(1) > max_edge
        done.append(polys[~big])
        P, k = polys[big], e[big].argmax(1)
        idx = np.arange(len(P))
        a, b, c = P[idx, k], P[idx, (k + 1) % 3], P[idx, (k + 2) % 3]
        m = (a + b) / 2
        polys = np.concatenate([np.stack([a, m, c], 1), np.stack([m, b, c], 1)])
    return np.concatenate(done)


def render_png(parts, path, elev, azim, hide=(), title="", zoom=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    import numpy as np

    light = np.array([0.4, -0.5, 0.8])
    light /= np.linalg.norm(light)
    fig = plt.figure(figsize=(9, 9), dpi=110)
    ax = fig.add_subplot(111, projection="3d")
    allv, all_polys, all_fc = [], [], []
    for pt in parts:
        if pt.name in hide:
            continue
        verts, tris = pt.shape.val().tessellate(0.4, 0.3)
        V = np.array([v.toTuple() for v in verts])
        if zoom is not None:
            lo, hi = np.array(zoom[0]), np.array(zoom[1])
            if ((V.max(0) < lo) | (V.min(0) > hi)).any():
                continue
        polys = _subdivide(V[np.array(tris)], 25.0)
        n = np.cross(polys[:, 1] - polys[:, 0], polys[:, 2] - polys[:, 0])
        n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
        shade = 0.45 + 0.55 * np.abs(n @ light)
        base = np.array(COLORS[pt.color])
        fc = np.clip(base[None, :] * shade[:, None], 0, 1)
        all_polys.append(polys)
        all_fc.append(fc)
        allv.append(V)
    # одна коллекция — matplotlib сортирует по глубине каждый треугольник
    ax.add_collection3d(Poly3DCollection(np.vstack(all_polys), facecolors=np.vstack(all_fc),
                                         edgecolors="none", linewidths=0))
    V = np.vstack(allv)
    lo, hi = (np.array(zoom[0]), np.array(zoom[1])) if zoom else (V.min(0), V.max(0))
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2])
    ax.set_box_aspect(hi - lo)
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    if title:
        ax.set_title(title)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def export_dxf(wp, path):
    cq.exporters.export(wp.faces("<Z"), path, exportType="DXF")


def mass_kg(pt):
    return pt.shape.val().Volume() * 1e-9 * DENSITY[pt.material]


def write_bom(g, parts, path):
    p = g.p
    moving_names = {"mast", "rack", "gs_bracket_top", "swivel_block", "bushing", "ball_plunger",
                    "stop_pin", "ptfe_washer", "nut_m16", "flange", "bolt_m16", "tabletop"}
    m_move = sum(mass_kg(pt) for pt in parts if pt.name in moving_names)
    m_total = sum(mass_kg(pt) for pt in parts)
    w_n = m_move * 9.81
    f_rec = int(round(w_n * 0.8 / 5) * 5)
    L = []
    L.append("# Спецификация столика\n")
    L.append("Сгенерировано `generate.py`. Все размеры в мм.\n")
    L.append("## Основные размеры\n")
    L.append(f"- Ширина всех досок: **{g.W}**, фанера **{g.T}**")
    L.append(f"- Высота столешницы: **{p['table_height_min']} … {p['table_height_min'] + g.travel}**"
             f" (ход {g.travel}, шаг {g.pitch})")
    L.append(f"- Стойка: высота {g.stand_h:.0f}, глубина {g.D:.0f} (доска + полость {g.Gap:.0f} + доска)")
    L.append(f"- Перекрытие мачты в стойке в верхнем положении: "
             f"{g.stand_top - g.mast_z0 - g.travel:.0f}")
    L.append(f"- Поворот столешницы: 0° … {p['rotation_deg']}°, фиксация шариком")
    L.append(f"- Масса всего ≈ **{m_total:.1f} кг**, подвижная часть ≈ **{m_move:.1f} кг** ({w_n:.0f} Н)\n")
    L.append("## Газовый упор\n")
    L.append(f"- Длина (ушко-ушко, выдвинут) **{p['gas_spring_length']}**, ход **{p['gas_spring_stroke']}**")
    L.append(f"- Рекомендуемое усилие ≈ **{f_rec} Н** (≈80 % веса подвижной части).")
    L.append("  Усилие должно быть *меньше* веса подвижной части: храповик свободно")
    L.append("  пропускает вверх, и слишком сильный упор будет сам поднимать столешницу.")
    if f_rec < 50:
        L.append("  Упоров слабее 50 Н почти нет в продаже — можно ставить без упора"
                 " (подвижная часть лёгкая) или взять 50 Н и догрузить столешницу.")
    L.append("")
    L.append("## Детали\n")
    L.append("| Деталь | Материал | Описание | Масса, кг |")
    L.append("|---|---|---|---|")
    for pt in parts:
        L.append(f"| `{pt.name}` | {pt.material} | {pt.descr} | {mass_kg(pt):.2f} |")
    L.append("")
    L.append("## Крепёж (дополнительно)\n")
    L.append("- Конфирматы 7×50: основание↔доски/бруски стойки (≥ 8 шт), клей ПВА")
    L.append("- Шурупы 4×30: рейка → мачта (через 50 мм), направляющие → доска B")
    L.append("- 2 шурупа 5×60 поперёк поворотного блока через шип мачты + клей")
    L.append("- 4 шурупа 5×25: фланец → столешница")
    L.append("- 2 болта M8 с ушками/шарнирными наконечниками для газового упора")
    L.append("")
    L.append("## Файлы для резки\n")
    L.append("- `dxf/rack.dxf` — рейка (алюминий/сталь 8–10 мм, или PETG 100 % заполнения)")
    L.append("- `dxf/pawl.dxf` — собачка (сталь 6–8 мм)")
    L.append("- `dxf/flange.dxf` — фланец (сталь 4–5 мм)")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")
    return m_move, f_rec


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--params", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "params.json"))
    ap.add_argument("--out", default="output")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                    help="переопределить параметр, напр. --set width=320")
    ap.add_argument("--no-png", action="store_true")
    args = ap.parse_args()

    p = dict(DEFAULTS)
    if os.path.exists(args.params):
        with open(args.params, encoding="utf-8") as f:
            p.update({k: v for k, v in json.load(f).items() if not k.startswith("_")})
    for kv in args.set:
        k, v = kv.split("=", 1)
        k = k.strip()
        if k not in DEFAULTS:
            sys.exit(f"Неизвестный параметр: {k}")
        p[k] = float(v) if "." in v else int(v)

    g = Geo(p)
    out = args.out
    for d in ("step", "glb", "stl", "dxf", "png"):
        os.makedirs(os.path.join(out, d), exist_ok=True)

    configs = {
        "low": (0, 0),
        "high": (g.travel, 0),
        "high_rotated": (g.travel, p["rotation_deg"]),
    }
    built = {}
    for name, (h, th) in configs.items():
        print(f"[build] {name}: h={h}, поворот={th}°")
        parts = build(g, h, th)
        built[name] = parts
        export_assembly(parts, os.path.join(out, "step", f"assembly_{name}"))
        os.replace(os.path.join(out, "step", f"assembly_{name}.glb"),
                   os.path.join(out, "glb", f"assembly_{name}.glb"))

    print("[stl] детали")
    for pt in built["low"]:
        cq.exporters.export(pt.shape, os.path.join(out, "stl", f"{pt.name}.stl"), tolerance=0.1)

    print("[dxf] рейка, собачка, фланец")
    export_dxf(rack_local(g), os.path.join(out, "dxf", "rack.dxf"))
    export_dxf(pawl_local(g, 1), os.path.join(out, "dxf", "pawl.dxf"))
    export_dxf(flange_local(g), os.path.join(out, "dxf", "flange.dxf"))

    m_move, f_rec = write_bom(g, built["low"], os.path.join(out, "BOM.md"))
    with open(os.path.join(out, "params_used.json"), "w", encoding="utf-8") as f:
        json.dump({**p, "travel_effective": g.travel}, f, ensure_ascii=False, indent=2)

    if not args.no_png:
        print("[png] превью")
        mech_hide = ("stand_board_B", "gusset_left", "gusset_right", "tabletop", "flange",
                     "guide_left", "guide_right")
        render_png(built["low"], os.path.join(out, "png", "iso_low.png"), 22, -60, title="Нижнее положение")
        render_png(built["high"], os.path.join(out, "png", "iso_high.png"), 22, -60, title="Верхнее положение")
        render_png(built["high_rotated"], os.path.join(out, "png", "iso_rotated.png"), 30, -50,
                   title=f"Столешница отведена на {p['rotation_deg']}°")
        render_png(built["low"], os.path.join(out, "png", "side.png"), 0, -90, title="Вид сбоку")
        render_png(built["low"], os.path.join(out, "png", "mechanism.png"), 10, 10, hide=mech_hide,
                   title="Механизм (без доски B)")
        z0 = g.piv_z - 60
        render_png(built["low"], os.path.join(out, "png", "pawl_closeup.png"), 0, 0, hide=mech_hide,
                   title="Рейка и собачка",
                   zoom=((-5, g.rack_y0 - 20, z0), (g.D + 10, g.W + 40, z0 + 130)))

    print(f"Готово → {out}/  (подвижная часть {m_move:.1f} кг, упор ≈{f_rec} Н)")


if __name__ == "__main__":
    main()
