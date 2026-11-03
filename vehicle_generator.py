#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vehicle_generator.py
====================

Prozeduraler Generator für game-ready 3D-Fahrzeugmodelle auf Basis von Blender (bpy):

  * ``enduro`` – 300er Zweitakt-Enduro (Gitterrohrrahmen, USD-Gabel, Speichenräder
    mit Stollenreifen, Auspuffbirne, Kette, Kühler, Kunststoffteile, Bremsleitungen …)
  * ``heli``   – leichter Turbinen-Hubschrauber (Kabinen-Loft mit echten Fenster-
    ausschnitten, Türen, Innenraum, Hauptrotor mit Profilblättern, Heckrotor, Kufen …)
  * ``plane``  – viersitziger Hochdecker der C172-Klasse (gewölbter Flügel mit Klappen und
    Querrudern, Leitwerk mit Rudern, Propeller, Bugradfahrwerk mit Radverkleidungen …)
  * ``offroad`` – 5-türiger Geländewagen (Leiterrahmen, Starrachsen mit Verschränkung, lenkbare
    Achsschenkel, teleskopierende Federbeine, Motorraum mit Reihen-Sechszylinder-Turbodiesel,
    AT-Reifen 265/75 R16, Seilwinde, Schnorchel, Dachträger, Reserverad an der Hecktür …)
  * ``boat``   – Bowrider-Motorboot (Tiefen-V-Rumpf mit Spritzleisten, Cockpit mit Teakdeck, gebogene
    Windschutzscheibe mit Durchgangstür, Außenborder mit Kipp-, Lenk- und Propeller-Pivot, Badeplattformen …)
  * ``suv``    – Elektro-SUV (zwei E-Motoren, Doppelquerlenker mit Hinterachslenkung, schwarzes Dach, LED-Signatur,
    Unterfahrschutz, Dachplattform, Frunk, Heckklappe, Ladeklappe, AT-Reifen 275/55 R20 …)

Game-Ready-Pipeline:
  reale Maße (Meter) · saubere Pivots für alle beweglichen Teile (inkl. Aim-Pivots und geprüften
  Bewegungsbereichen) · optionales Skelett (Skeletal Mesh) · UV-Atlas · Textur-Bake (BaseColor /
  ORM / Normal / Emissive, AO mit auseinandergezogenen beweglichen Teilen) · LOD-Kette ·
  UCX-Kollision · Export als FBX / GLB / .blend · JSON-Report

Aufruf (Beispiele):
  blender --background --factory-startup --python-exit-code 1 --python vehicle_generator.py -- --out export
  blender -b -P vehicle_generator.py -- --vehicle heli --bake 2048 --rig --target unreal
  python vehicle_generator.py --vehicle enduro --preview          (mit ``pip install bpy``; ohne bpy wird
                                                                   ``blender`` aus dem PATH gestartet)

In der Blender-Oberfläche (Scripting-Tab → Öffnen → Ausführen) werden alle Modelle
ohne Export direkt in der aktuellen Szene erzeugt.

Konventionen beim Modellieren: +X = vorne, +Y = links, +Z = oben, Ursprung am Boden.
Die Ausrichtung für die Ziel-Engine wird erst beim Export angewendet (``--target``).
"""

import argparse
import bisect
import hashlib
import json
import math
import os
import random
import shutil
import subprocess
import sys
import time

try:
    import bpy
except ModuleNotFoundError as _err:
    if __name__ != "__main__" or _err.name != "bpy":
        raise
    # Ohne bpy-Modul: dieselbe Kommandozeile an Blender (aus dem PATH) im Hintergrund weiterreichen.
    # --python-exit-code 1: Python-Fehler → Exit-Code 1 (sonst meldet Blender trotz Absturz Erfolg)
    _blender = shutil.which("blender")
    if not _blender:
        raise SystemExit("Weder das Python-Modul 'bpy' noch 'blender' im PATH gefunden – siehe README (Installation).")
    raise SystemExit(subprocess.call([_blender, "--background", "--factory-startup", "--python-exit-code", "1",
                                      "--python", os.path.abspath(__file__), "--", *sys.argv[1:]]))
import bmesh
from mathutils import Matrix, Quaternion, Vector
from mathutils.bvhtree import BVHTree
from mathutils.geometry import convex_hull_2d
from mathutils.interpolate import poly_3d_calc

V = Vector
TAU = math.tau
X_AXIS, Y_AXIS, Z_AXIS = V((1, 0, 0)), V((0, 1, 0)), V((0, 0, 1))


def _own_sha256():
    """Prüfsumme dieses Scripts beim Laden (für den Report: welche Generator-Version hat exportiert?)."""
    try:
        with open(os.path.abspath(__file__), "rb") as fh:
            return hashlib.sha256(fh.read()).hexdigest()
    except (OSError, NameError):       # z.B. aus dem Blender-Texteditor gestartet
        return None


GENERATOR_SHA256 = _own_sha256()


# =============================================================================
# 1) Mathematik & Kurven
# =============================================================================

def lerp(a, b, t):
    return a + (b - a) * t


def clamp(x, lo=0.0, hi=1.0):
    return max(lo, min(hi, x))


def smoothstep(e0, e1, x):
    t = clamp((x - e0) / (e1 - e0))
    return t * t * (3.0 - 2.0 * t)


def sgnpow(x, p):
    return math.copysign(abs(x) ** p, x)


def basis_from(axis):
    """Zwei Vektoren U, W senkrecht zu ``axis`` mit U x W = axis."""
    a = V(axis).normalized()
    ref = Z_AXIS if abs(a.z) < 0.9 else X_AXIS
    u = (ref - a * ref.dot(a)).normalized()
    return u, a.cross(u)


def frame_matrix(origin, x_axis, y_axis, z_axis):
    """4x4-Matrix aus Ursprung und drei Achsen (Spalten)."""
    m = Matrix.Identity(4)
    for i, ax in enumerate((x_axis, y_axis, z_axis)):
        m[0][i], m[1][i], m[2][i] = ax
    m[0][3], m[1][3], m[2][3] = origin
    return m


def look_frame(origin, forward, up=Z_AXIS):
    """Rechtshändiges Koordinatensystem: lokales +Z = ``forward``, lokales +Y ~ ``up``."""
    z = V(forward).normalized()
    x = V(up).cross(z)
    if x.length < 1e-6:
        x = X_AXIS.cross(z)
    x.normalize()
    return frame_matrix(origin, x, z.cross(x), z)


def axis_frame(origin, axis):
    """Rahmen mit lokalem +Y = ``axis`` (Konvention der Knochen) und lokalem Z möglichst nach oben."""
    y = V(axis).normalized()
    if abs(y.z) < 0.9:
        z = (Z_AXIS - y * Z_AXIS.dot(y)).normalized()
        x = y.cross(z)
    else:
        x = (X_AXIS - y * X_AXIS.dot(y)).normalized()
        z = x.cross(y)
    return frame_matrix(origin, x, y, z)


def forward_frame(origin, forward):
    """Rahmen mit lokalem +X = ``forward`` (für Sockets, z.B. Auspuffrichtung)."""
    x = V(forward).normalized()
    ref = Z_AXIS if abs(x.z) < 0.9 else X_AXIS
    y = ref.cross(x).normalized()
    return frame_matrix(origin, x, y, x.cross(y))


class Pchip:
    """Monotone kubische Interpolation (Fritsch–Carlson) – kein Überschwingen bei Spanten."""

    def __init__(self, xs, ys):
        pairs = sorted(zip(xs, ys))
        self.x = [p[0] for p in pairs]
        self.y = [p[1] for p in pairs]
        n = len(self.x)
        self.h = [self.x[i + 1] - self.x[i] for i in range(n - 1)]
        d = [(self.y[i + 1] - self.y[i]) / self.h[i] for i in range(n - 1)]
        m = [0.0] * n
        if n == 2:
            m = [d[0], d[0]]
        else:
            m[0], m[-1] = d[0], d[-1]
            for i in range(1, n - 1):
                if d[i - 1] * d[i] <= 0.0:
                    m[i] = 0.0
                else:
                    w1 = 2 * self.h[i] + self.h[i - 1]
                    w2 = self.h[i] + 2 * self.h[i - 1]
                    m[i] = (w1 + w2) / (w1 / d[i - 1] + w2 / d[i])
        self.m = m

    def __call__(self, x):
        xs = self.x
        if x <= xs[0]:
            return self.y[0]
        if x >= xs[-1]:
            return self.y[-1]
        i = bisect.bisect_right(xs, x) - 1
        h = self.h[i]
        t = (x - xs[i]) / h
        t2, t3 = t * t, t * t * t
        return ((2 * t3 - 3 * t2 + 1) * self.y[i] + (t3 - 2 * t2 + t) * h * self.m[i]
                + (-2 * t3 + 3 * t2) * self.y[i + 1] + (t3 - t2) * h * self.m[i + 1])


def _cr_point(p0, p1, p2, p3, u, alpha=0.5):
    """Zentripetaler Catmull-Rom (Barry-Goldman) – keine Schlaufen, kein Überschwingen."""
    def knot(ti, a, b):
        return ti + max((b - a).length, 1e-6) ** alpha
    t0 = 0.0
    t1 = knot(t0, p0, p1)
    t2 = knot(t1, p1, p2)
    t3 = knot(t2, p2, p3)
    t = lerp(t1, t2, u)
    a1 = p0 * ((t1 - t) / (t1 - t0)) + p1 * ((t - t0) / (t1 - t0))
    a2 = p1 * ((t2 - t) / (t2 - t1)) + p2 * ((t - t1) / (t2 - t1))
    a3 = p2 * ((t3 - t) / (t3 - t2)) + p3 * ((t - t2) / (t3 - t2))
    b1 = a1 * ((t2 - t) / (t2 - t0)) + a2 * ((t - t0) / (t2 - t0))
    b2 = a2 * ((t3 - t) / (t3 - t1)) + a3 * ((t - t1) / (t3 - t1))
    return b1 * ((t2 - t) / (t2 - t1)) + b2 * ((t - t1) / (t2 - t1))


def spline(points, samples=8, closed=False):
    """Glatte Kurve durch alle Kontrollpunkte (z.B. Schläuche, Kabel, Auspuff)."""
    pts = [V(p) for p in points]
    if closed:
        ext = [pts[-1]] + pts + [pts[0], pts[1]]
        nseg = len(pts)
    else:
        ext = [pts[0] * 2 - pts[1]] + pts + [pts[-1] * 2 - pts[-2]]
        nseg = len(pts) - 1
    out = []
    for i in range(nseg):
        for s in range(samples):
            out.append(_cr_point(ext[i], ext[i + 1], ext[i + 2], ext[i + 3], s / samples))
    if not closed:
        out.append(pts[-1].copy())
    return out


def fillet_path(points, radius, arc_segs=6):
    """Polylinie mit gerundeten Ecken (Biegeradius) – typisch für gebogene Rohre."""
    pts = [V(p) for p in points]
    out = [pts[0]]
    for i in range(1, len(pts) - 1):
        p0, p1, p2 = pts[i - 1], pts[i], pts[i + 1]
        d0, d1 = p1 - p0, p2 - p1
        l0, l1 = d0.length, d1.length
        d0.normalize()
        d1.normalize()
        phi = math.acos(clamp(d0.dot(d1), -1.0, 1.0))
        if phi < 1e-3:
            out.append(p1)
            continue
        r = radius[i - 1] if isinstance(radius, (list, tuple)) else radius
        t = min(r * math.tan(phi / 2), 0.49 * l0, 0.49 * l1)
        r_eff = t / math.tan(phi / 2)
        a = p1 - d0 * t
        axis = d0.cross(d1).normalized()
        c = a + axis.cross(d0) * r_eff
        n = max(2, int(arc_segs * phi / (math.pi / 2)) + 1)
        for s in range(n + 1):
            out.append(c + Quaternion(axis, phi * s / n) @ (a - c))
    out.append(pts[-1])
    return out


def resample(path, step):
    """Polylinie in gleichmäßige Abstände ``step`` unterteilen."""
    out = [path[0].copy()]
    acc = 0.0
    for a, b in zip(path, path[1:]):
        seg = (b - a).length
        while acc + seg >= step:
            t = (step - acc) / seg
            a = a.lerp(b, t)
            seg = (b - a).length
            acc = 0.0
            out.append(a.copy())
        acc += seg
    if (out[-1] - path[-1]).length > step * 0.3:
        out.append(path[-1].copy())
    return out


def path_frames(path, up=None):
    """Rotationsminimierende Rahmen (Parallel Transport) entlang eines Pfades."""
    n = len(path)
    tangents = []
    for i in range(n):
        a = path[max(i - 1, 0)]
        b = path[min(i + 1, n - 1)]
        tangents.append((b - a).normalized())
    t0 = tangents[0]
    ref = V(up) if up is not None else (Z_AXIS if abs(t0.z) < 0.9 else X_AXIS)
    nrm = (ref - t0 * ref.dot(t0)).normalized()
    frames = []
    for i in range(n):
        if i:
            nrm = tangents[i - 1].rotation_difference(tangents[i]) @ nrm
            nrm = (nrm - tangents[i] * nrm.dot(tangents[i])).normalized()
        frames.append((tangents[i], nrm, tangents[i].cross(nrm)))
    return frames


def path_length(path):
    return sum((b - a).length for a, b in zip(path, path[1:]))


def superellipse_2d(hw, top, bot, e_top=2.0, e_bot=2.0, segs=32, dense=96):
    """Superellipsen-Querschnitt (y, z) mit krümmungsadaptiver Abtastung.

    Start unten Mitte, dann +Y, oben, -Y (gegen den Uhrzeigersinn von +X gesehen).
    ``segs`` muss durch 4 teilbar sein – die vier Extrempunkte sind feste Spalten,
    dadurch bleibt die Topologie zwischen Spanten konsistent und symmetrisch.
    """
    def pt(t):
        c, s = math.cos(t), math.sin(t)
        if s >= 0:
            return V((hw * sgnpow(c, 2.0 / e_top), top * sgnpow(s, 2.0 / e_top)))
        return V((hw * sgnpow(c, 2.0 / e_bot), bot * sgnpow(s, 2.0 / e_bot)))

    per = segs // 4
    out = []
    for q in range(4):
        t0 = -math.pi / 2 + q * math.pi / 2
        pts = [pt(t0 + (math.pi / 2) * i / dense) for i in range(dense + 1)]
        seg_len = [(pts[i + 1] - pts[i]).length for i in range(dense)]
        total = sum(seg_len)
        if total < 1e-7:
            out.extend(pts[int(dense * k / per)] for k in range(per))
            continue
        lam = total / (math.pi / 2) * 0.6
        w = []
        for i in range(dense):
            turn = 0.0
            if 0 < i < dense - 1:
                d0, d1 = pts[i + 1] - pts[i], pts[i + 2] - pts[i + 1]
                if d0.length > 1e-9 and d1.length > 1e-9:
                    turn = abs(d0.angle_signed(d1, 0.0))
            w.append(seg_len[i] + lam * turn)
        cum = [0.0]
        for x in w:
            cum.append(cum[-1] + x)
        for k in range(per):
            target = cum[-1] * k / per
            i = min(bisect.bisect_right(cum, target) - 1, dense - 1)
            f = (target - cum[i]) / max(cum[i + 1] - cum[i], 1e-12)
            out.append(pts[i].lerp(pts[i + 1], f))
    return out


def circle_2d(r, n, phase=0.0):
    return [V((r * math.cos(phase + TAU * i / n), r * math.sin(phase + TAU * i / n))) for i in range(n)]


def rounded_rect_2d(w, h, r, corner_segs=4):
    """Rechteck mit Radius (CCW), zentriert."""
    r = min(r, w / 2 - 1e-4, h / 2 - 1e-4)
    pts = []
    for cx, cy, a0 in ((w / 2 - r, h / 2 - r, 0.0), (-w / 2 + r, h / 2 - r, 90.0),
                       (-w / 2 + r, -h / 2 + r, 180.0), (w / 2 - r, -h / 2 + r, 270.0)):
        for s in range(corner_segs + 1):
            a = math.radians(a0 + 90.0 * s / corner_segs)
            pts.append(V((cx + r * math.cos(a), cy + r * math.sin(a))))
    return pts


def rounded_poly_2d(points, radius, segs=4):
    """2D-Polygon mit gerundeten Ecken (CCW)."""
    pts = [V((p[0], p[1], 0.0)) for p in points]
    n = len(pts)
    out = []
    for i in range(n):
        p0, p1, p2 = pts[i - 1], pts[i], pts[(i + 1) % n]
        r = radius[i] if isinstance(radius, (list, tuple)) else radius
        if r <= 0:
            out.append(p1.to_2d())
            continue
        arc = fillet_path([p0, p1, p2], r, segs)[1:-1]
        out.extend(p.to_2d() for p in arc)
    return out


def ensure_ccw(poly):
    area = sum(poly[i - 1].x * poly[i].y - poly[i].x * poly[i - 1].y for i in range(len(poly)))
    return list(poly) if area > 0 else list(reversed(poly))


# =============================================================================
# 2) Mesh-Baukasten
# =============================================================================

class MeshData:
    """Leichtgewichtiger Mesh-Puffer: Vertices, Faces, UVs pro Face-Ecke, Material pro Face."""

    def __init__(self):
        self.verts, self.faces, self.uvs, self.mats = [], [], [], []
        self.recalc = False

    def add(self, verts, faces, uvs=None, mat=None):
        off = len(self.verts)
        self.verts.extend(V(v) for v in verts)
        for i, f in enumerate(faces):
            self.faces.append(tuple(off + k for k in f))
            self.uvs.append(uvs[i] if uvs else None)
            self.mats.append(mat)
        return self

    def merge(self, other, matrix=None):
        off = len(self.verts)
        if matrix is not None:
            self.verts.extend(matrix @ v for v in other.verts)
        else:
            self.verts.extend(v.copy() for v in other.verts)
        self.faces.extend(tuple(off + k for k in f) for f in other.faces)
        self.uvs.extend(other.uvs)
        self.mats.extend(other.mats)
        self.recalc = self.recalc or other.recalc
        return self

    def transform(self, m):
        self.verts = [m @ v for v in self.verts]
        return self

    def compact(self):
        """Nicht referenzierte Vertices entfernen."""
        used = sorted({i for f in self.faces for i in f})
        remap = {old: new for new, old in enumerate(used)}
        self.verts = [self.verts[i] for i in used]
        self.faces = [tuple(remap[i] for i in f) for f in self.faces]
        return self

    def translate(self, offset):
        return self.transform(Matrix.Translation(V(offset)))

    def set_mat(self, mat):
        self.mats = [mat] * len(self.faces)
        return self

    def copy(self):
        return MeshData().merge(self)

    def mirrored(self, axis=1):
        """Spiegelung an einer Ebene (Standard: XZ-Ebene → links/rechts)."""
        md = MeshData()
        for v in self.verts:
            w = v.copy()
            w[axis] = -w[axis]
            md.verts.append(w)
        md.faces = [tuple(reversed(f)) for f in self.faces]
        md.uvs = [tuple(reversed(u)) if u else None for u in self.uvs]
        md.mats = list(self.mats)
        return md

    def with_mirror(self, axis=1):
        return self.copy().merge(self.mirrored(axis))

    def to_object(self, name, smooth=35.0):
        me = bpy.data.meshes.new(name)
        me.from_pydata([tuple(v) for v in self.verts], [], self.faces)
        slots = {}
        for m in self.mats:
            if m not in slots:
                slots[m] = len(slots)
                me.materials.append(m)
        me.polygons.foreach_set("material_index", [slots[m] for m in self.mats])
        has_uv = bool(self.faces) and all(u is not None for u in self.uvs)
        if has_uv:
            layer = me.uv_layers.new(name="UVMap")
            flat = [c for u in self.uvs for uv in u for c in uv]
            layer.data.foreach_set("uv", flat)
        me.validate(clean_customdata=False)
        if self.recalc:
            bm = bmesh.new()
            bm.from_mesh(me)
            bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
            bm.to_mesh(me)
            bm.free()
        me.update()
        obj = bpy.data.objects.new(name, me)
        obj["uv_ready"] = has_uv
        set_smoothing(obj, smooth)
        return obj


def set_smoothing(obj, angle_deg=35.0):
    """Smooth Shading + scharfe Kanten ab ``angle_deg`` (ersetzt das alte Auto-Smooth)."""
    me = obj.data
    bm = bmesh.new()
    bm.from_mesh(me)
    limit = math.radians(angle_deg) if angle_deg is not None else -1
    for f in bm.faces:
        f.smooth = angle_deg is not None
    for e in bm.edges:
        if angle_deg is None:
            continue
        if len(e.link_faces) == 2:
            e.smooth = e.calc_face_angle(0.0) <= limit
    bm.to_mesh(me)
    bm.free()


def grid_rings(rings, closed=True, cap0=None, cap1=None, mat=None, flip=False, wrap=False):
    """Fläche aus Ringen gleicher Punktzahl (Basis für Loft, Lathe und Rohre).

    Ringe CCW von "vorne" (+Laufrichtung) gesehen → Normalen zeigen nach außen.
    cap0/cap1: None, "ngon" oder ein Vector (Fächer zu diesem Mittelpunkt).
    """
    rows, cols = len(rings), len(rings[0])
    verts = [p for ring in rings for p in ring]
    arcs = []
    for ring in rings:
        a = [0.0]
        for j in range(1, cols + (1 if closed else 0)):
            a.append(a[-1] + (ring[j % cols] - ring[j - 1]).length)
        half = a[-1] * 0.5
        arcs.append([x - half for x in a])
    vs = [0.0]
    for k in range(1, rows + (1 if wrap else 0)):
        a, b = rings[k % rows], rings[k - 1]
        vs.append(vs[-1] + sum((a[j] - b[j]).length for j in range(cols)) / cols)
    if wrap:
        arcs.append(arcs[0])
    faces, uvs = [], []
    for k in range(rows if wrap else rows - 1):
        k1 = (k + 1) % rows
        for j in range(cols if closed else cols - 1):
            j1 = (j + 1) % cols
            f = (k * cols + j, k * cols + j1, k1 * cols + j1, k1 * cols + j)
            uv = ((arcs[k][j], vs[k]), (arcs[k][j + 1], vs[k]),
                  (arcs[k + 1][j + 1], vs[k + 1]), (arcs[k + 1][j], vs[k + 1]))
            faces.append(f)
            uvs.append(uv)
    md = MeshData().add(verts, faces, uvs, mat)

    def cap(ring_idx, reverse, mode):
        base = ring_idx * cols
        ring = rings[ring_idx]
        ctr = sum(ring, V()) / cols
        u_, w_ = basis_from((ring[0] - ctr).cross(ring[cols // 4] - ctr) if cols > 2 else Z_AXIS)

        def puv(p):
            d = p - ctr
            return (d.dot(u_), d.dot(w_))
        if mode == "ngon":
            idx = list(range(base, base + cols))
            if reverse:
                idx.reverse()
            md.faces.append(tuple(idx))
            md.uvs.append(tuple(puv(md.verts[i]) for i in idx))
            md.mats.append(mat)
        else:
            c = len(md.verts)
            md.verts.append(V(mode))
            for j in range(cols):
                a, b = base + j, base + (j + 1) % cols
                tri = (c, b, a) if reverse else (c, a, b)
                md.faces.append(tri)
                md.uvs.append(tuple(puv(md.verts[i]) for i in tri))
                md.mats.append(mat)
    if cap0 is not None:
        cap(0, True, cap0)
    if cap1 is not None:
        cap(rows - 1, False, cap1)
    if flip:
        md.faces = [tuple(reversed(f)) for f in md.faces]
        md.uvs = [tuple(reversed(u)) for u in md.uvs]
    return md


def lathe(profile, segs=32, center=(0, 0, 0), axis=Z_AXIS, mat=None, phase=0.0, closed_profile=False):
    """Rotationskörper. Profil: (Radius, Höhe) von unten (an der Achse) nach oben.

    ``closed_profile`` erzeugt einen geschlossenen Ring (z.B. Felge) – Normalen
    werden dann beim Erzeugen des Objekts automatisch nach außen gerichtet.
    """
    a = V(axis).normalized()
    u, w = basis_from(a)
    c = V(center)
    prof = [tuple(p) for p in profile]
    cap0 = cap1 = None
    if closed_profile:
        dirs = [math.cos(phase + TAU * j / segs) * u + math.sin(phase + TAU * j / segs) * w for j in range(segs)]
        md = grid_rings([[c + a * h + d * r for d in dirs] for r, h in prof], True, mat=mat, wrap=True)
        md.recalc = True
        return md
    if prof[0][0] < 1e-6:
        cap0 = c + a * prof[0][1]
        prof = prof[1:]
    if prof[-1][0] < 1e-6:
        cap1 = c + a * prof[-1][1]
        prof = prof[:-1]
    angles = [phase + TAU * j / segs for j in range(segs)]
    dirs = [math.cos(t) * u + math.sin(t) * w for t in angles]
    rings = [[c + a * h + d * r for d in dirs] for r, h in prof]
    return grid_rings(rings, True, cap0, cap1, mat)


def tube(path, radius, segs=12, mat=None, caps=True, up=None, exponent=2.0, roll=0.0):
    """Rohr entlang eines Pfades. ``radius``: Zahl, (rx, ry) oder Funktion f(s∈[0,1])."""
    path = [V(p) for p in path]
    frames = path_frames(path, up)
    cum = [0.0]
    for a, b in zip(path, path[1:]):
        cum.append(cum[-1] + (b - a).length)
    total = max(cum[-1], 1e-9)
    rings = []
    for p, (t, n, b), s in zip(path, frames, cum):
        r = radius(s / total) if callable(radius) else radius
        rx, ry = (r, r) if not isinstance(r, (tuple, list)) else r
        ring = []
        for j in range(segs):
            ang = roll + TAU * j / segs
            cx, sy = math.cos(ang), math.sin(ang)
            ring.append(p + n * (rx * sgnpow(cx, 2.0 / exponent)) + b * (ry * sgnpow(sy, 2.0 / exponent)))
        rings.append(ring)
    return grid_rings(rings, True, "ngon" if caps else None, "ngon" if caps else None, mat)


def cylinder(p0, p1, r, segs=16, mat=None, caps=True):
    return tube([V(p0), V(p1)], r, segs, mat, caps)


def extrude(poly2d, depth, matrix=None, mat=None, centered=True):
    """2D-Polygon (lokal XY) entlang lokal Z extrudieren und mit ``matrix`` platzieren."""
    poly = ensure_ccw([V(p) for p in poly2d])
    n = len(poly)
    z0, z1 = (-depth / 2, depth / 2) if centered else (0.0, depth)
    verts = [V((p.x, p.y, z0)) for p in poly] + [V((p.x, p.y, z1)) for p in poly]
    faces = [tuple(range(n, 2 * n)), tuple(reversed(range(n)))]
    uvs = [tuple((p.x, p.y) for p in poly), tuple((poly[i].x, poly[i].y) for i in reversed(range(n)))]
    per = [0.0]
    for i in range(n):
        per.append(per[-1] + (poly[(i + 1) % n] - poly[i]).length)
    for i in range(n):
        i1 = (i + 1) % n
        faces.append((i, i1, n + i1, n + i))
        uvs.append(((per[i], z0), (per[i + 1], z0), (per[i + 1], z1), (per[i], z1)))
    md = MeshData().add(verts, faces, uvs, mat)
    if matrix is not None:
        md.transform(matrix)
    return md


def box(center, size, mat=None, matrix=None):
    sx, sy, sz = (s / 2 for s in size)
    md = extrude([(-sx, -sy), (sx, -sy), (sx, sy), (-sx, sy)], 2 * sz, None, mat)
    md.translate(center)
    if matrix is not None:
        md.transform(matrix)
    return md


def surface_grid(fn, nu, nv, mat=None, flip=False, keep=None):
    """Offene Fläche aus Funktion fn(u, v) -> Vector, u,v ∈ [0,1] (für Verkleidungen).

    ``keep(u, v)`` (optional) entfernt Faces, deren Mittelpunkt False liefert (Ausschnitte).
    """
    rings = [[V(fn(i / nu, j / nv)) for j in range(nv + 1)] for i in range(nu + 1)]
    md = grid_rings(rings, closed=False, mat=mat, flip=flip)
    if keep is not None:
        sel = [i for i in range(len(md.faces)) if keep((i // nv + 0.5) / nu, (i % nv + 0.5) / nv)]
        md.faces = [md.faces[i] for i in sel]
        md.uvs = [md.uvs[i] for i in sel]
        md.mats = [md.mats[i] for i in sel]
        md.compact()
    return md


def loft_stations(stations, xs, segs=48, cap0=None, cap1=None, mat=None, y_center=0.0):
    """Loft entlang X aus Spanten-Tabelle.

    stations: Liste von Dicts mit x, hw (halbe Breite), zt (Oberkante), zb (Unterkante),
              optional zm (Höhe der breitesten Stelle), et/eb (Superellipsen-Exponenten).
    xs:       aufsteigende X-Positionen der erzeugten Ringe.
    """
    keys = ("hw", "zt", "zb", "zm", "et", "eb")
    data = {}
    for k in keys:
        pts = []
        for s in stations:
            if k == "zm":
                val = s.get("zm", (s["zt"] + s["zb"]) / 2)
            elif k in ("et", "eb"):
                val = s.get(k, 2.0)
            else:
                val = s[k]
            pts.append((s["x"], val))
        data[k] = Pchip([p[0] for p in pts], [p[1] for p in pts])
    rings = []
    for x in xs:
        hw, zt, zb, zm = data["hw"](x), data["zt"](x), data["zb"](x), data["zm"](x)
        sec = superellipse_2d(max(hw, 1e-4), max(zt - zm, 1e-4), max(zm - zb, 1e-4),
                              data["et"](x), data["eb"](x), segs)
        rings.append([V((x, y_center + p.x, zm + p.y)) for p in sec])
    return grid_rings(rings, True, cap0, cap1, mat)


def cosine_spacing(x0, x1, n, bias0=0.0, bias1=0.0):
    """n+1 Werte zwischen x0 und x1, an den Enden optional verdichtet."""
    out = []
    for i in range(n + 1):
        t = i / n
        tc = 0.5 - 0.5 * math.cos(math.pi * t)
        w = t + (tc - t) * max(bias0 if t < 0.5 else bias1, 0.0)
        out.append(lerp(x0, x1, w))
    return out


def helix(center, axis, radius, length, turns, samples_per_turn=24, phase=0.0):
    a = V(axis).normalized()
    u, w = basis_from(a)
    n = int(turns * samples_per_turn)
    return [V(center) + a * (length * i / n) + radius * (math.cos(phase + TAU * turns * i / n) * u
                                                       + math.sin(phase + TAU * turns * i / n) * w)
            for i in range(n + 1)]


def airfoil_2d(n=24, thickness=0.12, camber=0.0, camber_pos=0.4):
    """NACA-4-Ziffern-Profil (geschlossene Hinterkante), Sehnenlänge 1, Vorderkante bei x=0."""
    pts_u, pts_l = [], []
    for i in range(n + 1):
        beta = math.pi * i / n
        x = 0.5 * (1 - math.cos(beta))
        yt = 5 * thickness * (0.2969 * math.sqrt(x) - 0.1260 * x - 0.3516 * x ** 2
                              + 0.2843 * x ** 3 - 0.1036 * x ** 4)
        if camber > 0:
            p = camber_pos
            yc = camber / p ** 2 * (2 * p * x - x * x) if x < p else camber / (1 - p) ** 2 * ((1 - 2 * p) + 2 * p * x - x * x)
        else:
            yc = 0.0
        pts_u.append(V((x, yc + yt)))
        pts_l.append(V((x, yc - yt)))
    # Ring: Hinterkante → Oberseite → Vorderkante → Unterseite (CCW)
    ring = list(reversed(pts_u)) + pts_l[1:-1]
    return ring


# =============================================================================
# 3) Materialbibliothek (prozedurales PBR, vollständig bake-fähig)
# =============================================================================

class Nodes:
    """Kleiner Helfer zum Aufbau von Shader-Node-Bäumen."""

    def __init__(self, mat):
        if mat.node_tree is None:
            mat.use_nodes = True
        self.nt = mat.node_tree
        self.nt.nodes.clear()
        self.x = -1400

    def new(self, kind, **props):
        n = self.nt.nodes.new(kind)
        n.location = (self.x, 300 - 180 * (len(self.nt.nodes) % 6))
        self.x += 120
        for k, v in props.items():
            if hasattr(n, k):
                setattr(n, k, v)
            elif k in n.inputs:
                n.inputs[k].default_value = v
        return n

    def link(self, a, b):
        self.nt.links.new(a, b)

    def value(self, v):
        n = self.new("ShaderNodeValue")
        n.outputs[0].default_value = v
        return n.outputs[0]

    def math(self, op, a, b=None, clamp_=False):
        n = self.new("ShaderNodeMath", operation=op, use_clamp=clamp_)
        for i, s in enumerate((a, b)):
            if s is None:
                continue
            if isinstance(s, (int, float)):
                n.inputs[i].default_value = s
            else:
                self.link(s, n.inputs[i])
        return n.outputs[0]

    def map_range(self, s, f0, f1, t0=0.0, t1=1.0):
        n = self.new("ShaderNodeMapRange", clamp=True)
        self.link(s, n.inputs["Value"])
        n.inputs["From Min"].default_value = f0
        n.inputs["From Max"].default_value = f1
        n.inputs["To Min"].default_value = t0
        n.inputs["To Max"].default_value = t1
        return n.outputs["Result"]

    def mix_color(self, fac, a, b):
        n = self.new("ShaderNodeMix", data_type="RGBA", blend_type="MIX")
        for sock, val in ((n.inputs[0], fac), (n.inputs[6], a), (n.inputs[7], b)):
            if isinstance(val, (tuple, list)):
                sock.default_value = tuple(val) + ((1.0,) if len(val) == 3 else ())
            elif isinstance(val, (int, float)):
                sock.default_value = val
            else:
                self.link(val, sock)
        return n.outputs[2]

    def coords(self, space="Object", scale=(1, 1, 1)):
        tc = self.new("ShaderNodeTexCoord")
        if tuple(scale) == (1, 1, 1):
            return tc.outputs[space]
        mp = self.new("ShaderNodeMapping")
        mp.inputs["Scale"].default_value = scale
        self.link(tc.outputs[space], mp.inputs["Vector"])
        return mp.outputs["Vector"]

    def noise(self, vec, scale=10.0, detail=4.0, rough=0.55, out="Fac"):
        n = self.new("ShaderNodeTexNoise")
        n.inputs["Scale"].default_value = scale
        n.inputs["Detail"].default_value = detail
        n.inputs["Roughness"].default_value = rough
        self.link(vec, n.inputs["Vector"])
        return n.outputs[out]

    def world_z(self):
        g = self.new("ShaderNodeNewGeometry")
        sep = self.new("ShaderNodeSeparateXYZ")
        self.link(g.outputs["Position"], sep.inputs[0])
        return sep.outputs["Z"]


def _principled(nd):
    out = nd.new("ShaderNodeOutputMaterial")
    bsdf = nd.new("ShaderNodeBsdfPrincipled")
    nd.link(bsdf.outputs[0], out.inputs["Surface"])
    return bsdf


def _seam_mask(nd, seams, width=0.0032):
    """Gravierte Panel-Fugen: seams = [(achse 0/1/2, position, z_min, z_max)] in Weltkoordinaten."""
    g = nd.new("ShaderNodeNewGeometry")
    sep = nd.new("ShaderNodeSeparateXYZ")
    nd.link(g.outputs["Position"], sep.inputs[0])
    total = None
    for axis, pos, z0, z1 in seams:
        d = nd.math("ABSOLUTE", nd.math("SUBTRACT", sep.outputs["XYZ"[axis]], pos))
        line = nd.map_range(d, width * 0.35, width * 0.5, 1.0, 0.0)
        zm = nd.math("MULTIPLY", nd.map_range(sep.outputs["Z"], z0 - 0.003, z0 + 0.003),
                     nd.map_range(sep.outputs["Z"], z1 - 0.003, z1 + 0.003, 1.0, 0.0))
        m = nd.math("MULTIPLY", line, zm)
        total = m if total is None else nd.math("MAXIMUM", total, m)
    return total


def _apply_surface(nd, bsdf, color, rough, metallic=0.0, rough_var=0.08, bump=0.02, bump_scale=60.0,
                   dirt=0.0, dirt_z=(0.15, 0.7), dirt_color=(0.10, 0.075, 0.05), extra_color=None,
                   rough_src=None, seams=None):
    """Gemeinsame Oberflächenlogik: Rauheitsvariation, Mikro-Bump, Panel-Fugen, Schmutz von unten."""
    co = nd.coords("Object")
    col = extra_color if extra_color is not None else color
    rough_s = rough_src or nd.map_range(nd.noise(co, 8.0, 5.0, 0.6), 0.3, 0.7, rough - rough_var, rough + rough_var)
    groove = _seam_mask(nd, seams) if seams else None
    if groove is not None:
        col = nd.mix_color(nd.math("MULTIPLY", groove, 0.6), col, (0.02, 0.02, 0.022))
    if dirt > 0:
        zf = nd.map_range(nd.world_z(), dirt_z[0], dirt_z[1], 1.0, 0.0)
        splat = nd.map_range(nd.noise(co, 3.5, 6.0, 0.7), 0.42, 0.62, 0.0, 1.0)
        mask = nd.math("MULTIPLY", nd.math("MULTIPLY", zf, splat), dirt, clamp_=True)
        col = nd.mix_color(mask, col, dirt_color)
        rough_s = nd.math("ADD", nd.math("MULTIPLY", mask, 0.9 - rough), rough_s, clamp_=True)
        metallic_s = nd.math("MULTIPLY", nd.math("SUBTRACT", 1.0, mask), metallic) if metallic else None
    else:
        metallic_s = None
    if isinstance(col, (tuple, list)):
        bsdf.inputs["Base Color"].default_value = tuple(col) + (1.0,)
    else:
        nd.link(col, bsdf.inputs["Base Color"])
    nd.link(rough_s, bsdf.inputs["Roughness"])
    if metallic_s is not None:
        nd.link(metallic_s, bsdf.inputs["Metallic"])
    else:
        bsdf.inputs["Metallic"].default_value = metallic
    normal = None
    if groove is not None:
        gb = nd.new("ShaderNodeBump")
        gb.inputs["Strength"].default_value = 1.0
        gb.inputs["Distance"].default_value = 0.0015
        nd.link(nd.math("MULTIPLY", groove, -1.0), gb.inputs["Height"])
        normal = gb.outputs["Normal"]
    if bump > 0:
        b = nd.new("ShaderNodeBump")
        b.inputs["Strength"].default_value = bump
        b.inputs["Distance"].default_value = 0.002
        nd.link(nd.noise(co, bump_scale, 3.0, 0.5), b.inputs["Height"])
        if normal is not None:
            nd.link(normal, b.inputs["Normal"])
        normal = b.outputs["Normal"]
    if normal is not None:
        nd.link(normal, bsdf.inputs["Normal"])


def uv_weight(mat, factor):
    """Texeldichte-Gewicht fürs UV-Packen (siehe scale_uv_islands)."""
    mat["uv_scale"] = factor
    return mat


def new_mat(name):
    m = bpy.data.materials.new(name)
    m["generator"] = "vehicle_generator"
    return m, Nodes(m)


def mat_paint(name, color, rough=0.35, coat=0.0, metallic=0.0, dirt=0.0, dirt_z=(0.15, 0.7), livery=None,
              seams=None):
    """Lack/glänzender Kunststoff. ``livery``: [(z_von, z_bis, farbe[, steigung[, x_ende]])] Farbstreifen
    über Welt-Z (mit Steigung: Streifen verläuft schräg, z_eff = z - steigung * x; x_ende begrenzt nach hinten),
    ``seams``: gravierte Panel-Fugen (siehe _seam_mask)."""
    m, nd = new_mat(name)
    bsdf = _principled(nd)
    col = None
    if livery:
        col = tuple(color)
        geo = nd.new("ShaderNodeNewGeometry")
        pos = nd.new("ShaderNodeSeparateXYZ")
        nd.link(geo.outputs["Position"], pos.inputs[0])
        for band in livery:
            z0, z1, c = band[:3]
            z = pos.outputs["Z"]
            if len(band) > 3 and band[3]:
                z = nd.math("SUBTRACT", z, nd.math("MULTIPLY", pos.outputs["X"], band[3]))
            mask = nd.math("MULTIPLY", nd.map_range(z, z0 - 0.004, z0 + 0.004),
                           nd.map_range(z, z1 - 0.004, z1 + 0.004, 1.0, 0.0))
            if len(band) > 4 and band[4] is not None:      # Streifen endet bei x_end (z.B. vor dem Heckkonus)
                mask = nd.math("MULTIPLY", mask, nd.map_range(pos.outputs["X"], band[4] - 0.01, band[4] + 0.01))
            col = nd.mix_color(mask, col, c)
    _apply_surface(nd, bsdf, color, rough, metallic, 0.05, 0.015, 90.0, dirt, dirt_z, extra_color=col, seams=seams)
    if coat:
        bsdf.inputs["Coat Weight"].default_value = coat
        bsdf.inputs["Coat Roughness"].default_value = 0.04
    m.diffuse_color = tuple(color) + (1.0,)
    return m


def mat_plastic(name, color, rough=0.6, dirt=0.0, dirt_z=(0.15, 0.7), grain=0.05):
    m, nd = new_mat(name)
    bsdf = _principled(nd)
    _apply_surface(nd, bsdf, color, rough, 0.0, 0.1, grain, 260.0, dirt, dirt_z)
    m.diffuse_color = tuple(color) + (1.0,)
    return m


def mat_metal(name, color, rough=0.3, brushed=False, dirt=0.0, dirt_z=(0.15, 0.6)):
    m, nd = new_mat(name)
    bsdf = _principled(nd)
    if brushed:
        co = nd.coords("Object", (1.0, 1.0, 1.0))
        mp = nd.new("ShaderNodeMapping")
        mp.inputs["Scale"].default_value = (400.0, 4.0, 4.0)
        nd.link(co, mp.inputs["Vector"])
        streak = nd.noise(mp.outputs["Vector"], 2.0, 8.0, 0.7)
        r = nd.map_range(streak, 0.3, 0.7, rough - 0.06, rough + 0.06)
        _apply_surface(nd, bsdf, color, rough, 1.0, 0.04, 0.0, 1.0, dirt, dirt_z, rough_src=r)
    else:
        _apply_surface(nd, bsdf, color, rough, 1.0, 0.05, 0.008, 120.0, dirt, dirt_z)
    m.diffuse_color = tuple(color) + (1.0,)
    m.metallic = 1.0
    return m


def mat_rubber(name, color=(0.025, 0.025, 0.025), rough=0.85, dirt=0.0, dirt_z=(0.0, 0.5)):
    m, nd = new_mat(name)
    bsdf = _principled(nd)
    _apply_surface(nd, bsdf, color, rough, 0.0, 0.06, 0.06, 400.0, dirt, dirt_z, dirt_color=(0.13, 0.10, 0.07))
    bsdf.inputs["Specular IOR Level"].default_value = 0.35
    m.diffuse_color = tuple(color) + (1.0,)
    return m


def mat_glass(name, tint=(0.75, 0.82, 0.85), rough=0.02):
    m, nd = new_mat(name)
    bsdf = _principled(nd)
    bsdf.inputs["Base Color"].default_value = tuple(tint) + (1.0,)
    bsdf.inputs["Roughness"].default_value = rough
    bsdf.inputs["Transmission Weight"].default_value = 1.0
    bsdf.inputs["IOR"].default_value = 1.49
    m.diffuse_color = tuple(tint) + (0.3,)
    m["is_glass"] = True
    return m


def mat_emit(name, color, strength=8.0, base=None):
    m, nd = new_mat(name)
    bsdf = _principled(nd)
    bsdf.inputs["Base Color"].default_value = tuple(base or color) + (1.0,)
    bsdf.inputs["Roughness"].default_value = 0.15
    bsdf.inputs["Emission Color"].default_value = tuple(color) + (1.0,)
    bsdf.inputs["Emission Strength"].default_value = strength
    m.diffuse_color = tuple(color) + (1.0,)
    return m


def mat_carbon(name, rough=0.25):
    m, nd = new_mat(name)
    bsdf = _principled(nd)
    co = nd.coords("Object")
    wave_a = nd.new("ShaderNodeTexWave", wave_type="BANDS", bands_direction="X")
    wave_a.inputs["Scale"].default_value = 180.0
    nd.link(co, wave_a.inputs["Vector"])
    chk = nd.new("ShaderNodeTexChecker")
    chk.inputs["Scale"].default_value = 220.0
    nd.link(co, chk.inputs["Vector"])
    fac = nd.math("MULTIPLY", wave_a.outputs["Fac"], chk.outputs["Fac"])
    col = nd.mix_color(fac, (0.012, 0.012, 0.014), (0.06, 0.06, 0.065))
    nd.link(col, bsdf.inputs["Base Color"])
    bsdf.inputs["Roughness"].default_value = rough
    bsdf.inputs["Coat Weight"].default_value = 1.0
    bsdf.inputs["Coat Roughness"].default_value = 0.03
    m.diffuse_color = (0.03, 0.03, 0.035, 1.0)
    return m


def mat_rotor_blade(name, color, bands, rough=0.4, axis="Z"):
    """Rotorblatt-/Propeller-Lack mit Warnstreifen ``bands`` = [(r_von, r_bis, farbe)];
    Maske über den radialen Abstand zur Drehachse ``axis`` durch den Objekt-Ursprung (= Nabe)."""
    m, nd = new_mat(name)
    bsdf = _principled(nd)
    co = nd.coords("Object")
    sep = nd.new("ShaderNodeSeparateXYZ")
    nd.link(co, sep.inputs[0])
    comb = nd.new("ShaderNodeCombineXYZ")
    for c in "XYZ":
        if c != axis:
            nd.link(sep.outputs[c], comb.inputs[c])
    ln = nd.new("ShaderNodeVectorMath", operation="LENGTH")
    nd.link(comb.outputs[0], ln.inputs[0])
    r = ln.outputs["Value"]
    col = tuple(color)
    for r0, r1, c in bands:
        band = nd.math("MULTIPLY", nd.map_range(r, r0 - 0.003, r0 + 0.003),
                       nd.map_range(r, r1 - 0.003, r1 + 0.003, 1.0, 0.0))
        col = nd.mix_color(band, col, c)
    _apply_surface(nd, bsdf, color, rough, 0.0, 0.06, 0.01, 120.0, extra_color=col)
    m.diffuse_color = tuple(color) + (1.0,)
    return m


# =============================================================================
# 4) Baugruppe (Hierarchie, Pivots, Kollisionsgruppen)
# =============================================================================

class Assembly:
    """Verwaltet die Objekte eines Fahrzeugs.

    Jedes bewegliche Teil hängt an einem Pivot-Empty (wird bei ``--rig`` zum Knochen).
    Geometrie wird in Fahrzeugkoordinaten erzeugt; ``add`` verschiebt den Objekt-
    ursprung auf den jeweiligen Pivot, damit sich Teile in der Engine korrekt drehen.
    """

    def __init__(self, name):
        self.name = name
        self.coll = bpy.data.collections.new(name)
        bpy.context.scene.collection.children.link(self.coll)
        self.root = self._empty(name, V((0, 0, 0)), 0.5)
        self.pivots = {"root": self.root}
        self.axes = {"root": Z_AXIS.copy()}
        self.parents = {"root": None}
        self.collision = []   # (name, [objects], keep(point) or None, max_verts)
        self.hull_exempt = []           # ([objects], Grund): ragen bewusst aus den Hüllen (dünne Antennen …)
        self.hull_exempt_pivots = {}    # Pivot → Grund: Teilbaum ohne Hülle (drehende Rotor-/Propellerblätter)
        self.ao_distance = 0.3
        self.sockets = {}
        self.socket_fwd = {}
        self.notes = {"root": ""}
        self.motion = {"root": "none"}
        self.targets = {}               # Pivot → Ziel-Pivot (motion "aim")
        self.limits = {}                # Pivot → (min, max): geprüfter Bewegungsbereich (Grad bzw. Meter)

    def _empty(self, name, loc, size=0.15, kind="PLAIN_AXES"):
        e = bpy.data.objects.new(name, None)
        e.empty_display_type = kind
        e.empty_display_size = size
        e.location = loc
        self.coll.objects.link(e)
        return e

    def pivot(self, key, loc, axis=Z_AXIS, parent="root", orient=False, positive="", motion="hinge", target=None,
              limits=None):
        """Drehpunkt. ``orient``: Empty so drehen, dass sein lokales +Y auf der Drehachse liegt
        (für schräge Scharniere wie Türen, Ruder, Lenkkopf) – dann reicht in der Engine eine
        Rotation um die lokale Y-Achse. ``motion``: "spin" (dreht endlos: Räder, Rotoren, Propeller),
        "hinge" (begrenzter Ausschlag), "slide" (Verschiebung entlang der Achse, z.B. Federung) oder
        "aim" (lokales +Y zeigt per Look-At-/Aim-Constraint der Engine stets auf den Pivot ``target`` –
        zwei Hälften eines Federbeins visieren sich gegenseitig an und teleskopieren). ``limits``: kollisionsfrei
        geprüfter Bereich (min, max) in Grad ("hinge") bzw. Metern ("slide") für Constraints in der Engine."""
        e = self._empty(f"{self.name}_{key}_Pivot", V(loc), 0.12, "ARROWS")
        par = self.pivots[parent]
        e.parent = par
        e.matrix_world = axis_frame(V(loc), axis) if orient else Matrix.Translation(V(loc))
        e["oriented"] = bool(orient)
        e["motion"] = motion                 # landet als Custom Property (FBX) bzw. extras (glTF) in der Engine
        e["positive"] = positive
        if target:
            e["target"] = target
            self.targets[key] = target
        if limits:
            e["limit_min"], e["limit_max"] = limits
            self.limits[key] = tuple(limits)
        self.notes[key] = positive
        self.motion[key] = motion
        self.pivots[key] = e
        self.axes[key] = V(axis).normalized()
        self.parents[key] = parent
        return e

    def socket(self, key, loc, forward=None, parent="root"):
        """Andockpunkte für die Engine (Fahrerposition, Kamera, Licht …); lokale +X-Achse = ``forward``
        (Standard: Fahrzeugfront). ``parent``: Pivot, dem der Socket folgt (z.B. Scheinwerfer an der Lenkung).
        Der Name wird beim Export zu ``SOCKET_<key>`` (hier mit Asset-Präfix, damit mehrere
        Fahrzeuge in einer Szene keine Namenskonflikte haben)."""
        fwd = V(forward).normalized() if forward is not None else X_AXIS.copy()
        e = self._empty(f"{self.name}_SOCKET_{key}", V(loc), 0.08, "SPHERE")
        e.parent = self.pivots[parent]
        e.matrix_world = forward_frame(V(loc), fwd)
        e["oriented"] = True
        self.sockets[key] = e
        self.socket_fwd[key] = fwd
        return e

    def add(self, md_or_obj, name, pivot="root", smooth=35.0, mods=(), detail=0):
        obj = md_or_obj.to_object(f"{self.name}_{name}", smooth) if isinstance(md_or_obj, MeshData) else md_or_obj
        if obj.name not in self.coll.objects:
            self.coll.objects.link(obj)
        p = self.pivots[pivot]
        obj.data.transform(p.matrix_world.inverted())
        obj.parent = p
        obj.matrix_parent_inverse = Matrix.Identity(4)
        obj.location = (0, 0, 0)
        obj["pivot"] = pivot
        obj["detail"] = detail
        for kind, kw in mods:
            add_modifier(obj, kind, **kw)
        return obj

    def collide(self, name, objects, keep=None, max_verts=48):
        """Konvexe Kollisionshülle (UCX_) aus den Vertices von ``objects`` (optional gefiltert)."""
        self.collision.append((name, list(objects), keep, max_verts))

    def meshes(self):
        return [o for o in self.coll.all_objects if o.type == "MESH" and not o.name.startswith("UCX_")]


def add_modifier(obj, kind, **kw):
    """Modifier-Shortcuts: bevel, solidify, weighted, subsurf, mirror."""
    if kind == "bevel":
        m = obj.modifiers.new("Bevel", "BEVEL")
        m.width = kw.get("width", 0.002)
        m.segments = kw.get("segments", 2)
        m.limit_method = kw.get("limit", "ANGLE")
        m.angle_limit = math.radians(kw.get("angle", 40))
        m.harden_normals = kw.get("harden", True)
        m.miter_outer = "MITER_ARC"
        m.use_clamp_overlap = True
    elif kind == "solidify":
        m = obj.modifiers.new("Solidify", "SOLIDIFY")
        m.thickness = kw.get("thickness", 0.003)
        m.offset = kw.get("offset", -1.0)
        m.use_even_offset = True
        m.use_rim = True
        m.use_quality_normals = True
        m.material_offset = kw.get("material_offset", 0)
        m.material_offset_rim = kw.get("material_offset_rim", 0)
    elif kind == "weighted":
        m = obj.modifiers.new("WeightedNormal", "WEIGHTED_NORMAL")
        m.keep_sharp = True
        m.weight = 50
    elif kind == "subsurf":
        m = obj.modifiers.new("Subsurf", "SUBSURF")
        m.levels = m.render_levels = kw.get("levels", 1)
    else:
        raise ValueError(kind)
    return m


# =============================================================================
# 5) Vorschau-Rendering (Studio)
# =============================================================================

def render_preview(objects, path, resolution=(1600, 900), samples=96, view=(-35.0, 18.0), lens=50.0,
                   ground=True, dist_scale=1.0):
    """Studio-Render mit Area-Lights, Bodenschatten und automatisch gerahmter Kamera."""
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.samples = samples
    sc.cycles.use_denoising = True
    sc.render.resolution_x, sc.render.resolution_y = resolution
    sc.render.film_transparent = True
    sc.view_settings.view_transform = "AgX"
    sc.view_settings.look = "AgX - Medium High Contrast"
    sc.view_settings.exposure = -0.5
    temp = []
    bpy.context.view_layer.update()

    pts = []
    dg = bpy.context.evaluated_depsgraph_get()
    for o in objects:
        if o.type == "MESH":
            ev = o.evaluated_get(dg)
            me = ev.to_mesh()
            pts.extend(o.matrix_world @ v.co for v in me.vertices)
            ev.to_mesh_clear()
    lo = V((min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)))
    hi = V((max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)))
    ctr = (lo + hi) / 2
    radius = (hi - lo).length / 2

    world = bpy.data.worlds.new("PreviewWorld")
    sc.world = world
    nd = Nodes(world)
    bg = nd.new("ShaderNodeBackground")
    out = nd.new("ShaderNodeOutputWorld")
    sep = nd.new("ShaderNodeSeparateXYZ")
    tc = nd.new("ShaderNodeTexCoord")
    nd.link(tc.outputs["Generated"], sep.inputs[0])
    ramp_in = nd.map_range(sep.outputs["Z"], 0.45, 0.75)
    col = nd.mix_color(ramp_in, (0.42, 0.44, 0.47), (0.78, 0.84, 0.92))
    nd.link(col, bg.inputs["Color"])
    bg.inputs["Strength"].default_value = 0.55
    nd.link(bg.outputs[0], out.inputs["Surface"])

    if ground:
        gm = mat_plastic("PreviewGround", (0.30, 0.30, 0.31), 0.7)
        size = radius * 80
        g = MeshData().add([(-size, -size, 0), (size, -size, 0), (size, size, 0), (-size, size, 0)], [(0, 1, 2, 3)],
                           mat=gm).to_object("PreviewGround")
        g.is_shadow_catcher = True
        sc.collection.objects.link(g)
        temp.append(g)

    def area(name, loc, energy, size, color=(1, 1, 1), shadow=True):
        ld = bpy.data.lights.new(name, "AREA")
        ld.energy = energy
        ld.size = size
        ld.color = color
        ld.use_shadow = shadow
        lo_ = bpy.data.objects.new(name, ld)
        lo_.location = loc
        lo_.rotation_euler = (ctr - V(loc)).to_track_quat("-Z", "Y").to_euler()
        sc.collection.objects.link(lo_)
        temp.append(lo_)

    r = radius
    area("Key", ctr + V((-1.2 * r, -1.6 * r, 2.0 * r)), 320 * r * r, 1.6 * r, (1.0, 0.97, 0.92))
    area("Fill", ctr + V((1.8 * r, -1.0 * r, 0.8 * r)), 110 * r * r, 2.2 * r, (0.85, 0.9, 1.0), shadow=False)
    area("Rim", ctr + V((0.6 * r, 2.0 * r, 1.6 * r)), 260 * r * r, 1.2 * r, shadow=False)

    cam_d = bpy.data.cameras.new("PreviewCam")
    cam_d.lens = lens
    cam_d.sensor_fit = "HORIZONTAL"
    cam = bpy.data.objects.new("PreviewCam", cam_d)
    sc.collection.objects.link(cam)
    temp.append(cam)
    az, el = math.radians(view[0]), math.radians(view[1])
    dirv = V((math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)))
    # Kamera auf die tatsächliche Silhouette einpassen (nicht auf die Bounding-Kugel): Abstand so wählen,
    # dass alle Punkte ~88 % des Bildes füllen, und per Objektiv-Shift mittig ausrichten.
    fwd = -dirv
    right = fwd.cross(Z_AXIS) if abs(fwd.z) < 0.999 else X_AXIS.copy()
    right.normalize()
    up = right.cross(fwd)
    tan_h = cam_d.sensor_width / 2 / lens
    tan_v = tan_h * resolution[1] / resolution[0]
    rel = [p - ctr for p in pts[::max(1, len(pts) // 20000)]]
    px_, py_, pz_ = ([q.dot(a) for q in rel] for a in (right, up, fwd))
    near = max(-z for z in pz_)
    dist = radius / math.sin(math.atan(tan_v))
    for _ in range(6):
        xs = [x / (dist + z) for x, z in zip(px_, pz_)]
        ys = [y / (dist + z) for y, z in zip(py_, pz_)]
        need = max((max(xs) - min(xs)) / (2 * tan_h * 0.88), (max(ys) - min(ys)) / (2 * tan_v * 0.88))
        dist = max(dist * need, near * 1.1 + 0.05)
    cam_d.shift_x = (max(xs) + min(xs)) / 2 / (2 * tan_h)
    cam_d.shift_y = (max(ys) + min(ys)) / 2 / (2 * tan_h)
    cam.location = ctr + dirv * dist * dist_scale
    cam.rotation_euler = fwd.to_track_quat("-Z", "Y").to_euler()
    sc.camera = cam
    tmp = os.path.abspath(path) + ".rgba.png"
    sc.render.filepath = tmp
    sc.render.image_settings.color_mode = "RGBA"
    bpy.ops.render.render(write_still=True)
    for o in temp:
        bpy.data.objects.remove(o, do_unlink=True)
    # Studio-Hintergrund (vertikaler Verlauf + Vignette) per Alpha-Compositing
    import numpy as np
    img = bpy.data.images.load(tmp)
    fg = _pixels(img)
    h, w = fg.shape[:2]
    yy = np.linspace(0.0, 1.0, h)[:, None]
    xx = np.linspace(-1.0, 1.0, w)[None, :]
    # Blaugrauer Mittelton: helle Lackierungen (weißer Rumpf) heben sich ab, Schatten bleiben sichtbar
    top, bottom = np.array([0.19, 0.225, 0.28]), np.array([0.40, 0.43, 0.48])
    bg = bottom + (top - bottom) * (yy[..., None] ** 1.3)
    bg = bg * (1.0 - 0.16 * (xx[..., None] ** 2))
    a = fg[..., 3:4]
    out = np.concatenate([fg[..., :3] * a + bg * (1 - a), np.ones((h, w, 1), np.float32)], -1)
    final = _save_image(os.path.basename(path), out.astype(np.float32), os.path.abspath(path), "sRGB")
    bpy.data.images.remove(img)
    bpy.data.images.remove(final)
    os.remove(tmp)
    return path


def extrude_ring(outer, inner, depth, matrix=None, mat=None):
    """Ringförmige Extrusion (Außen- und Innenkontur mit gleicher Punktzahl, CCW)."""
    n = len(outer)
    z0, z1 = -depth / 2, depth / 2
    verts = ([V((p[0], p[1], z0)) for p in outer] + [V((p[0], p[1], z1)) for p in outer]
             + [V((p[0], p[1], z0)) for p in inner] + [V((p[0], p[1], z1)) for p in inner])
    o0, o1, i0, i1 = 0, n, 2 * n, 3 * n
    faces = []
    for i in range(n):
        j = (i + 1) % n
        faces.append((o1 + i, o1 + j, i1 + j, i1 + i))      # oben
        faces.append((o0 + j, o0 + i, i0 + i, i0 + j))      # unten
        faces.append((o0 + i, o0 + j, o1 + j, o1 + i))      # außen
        faces.append((i0 + j, i0 + i, i1 + i, i1 + j))      # innen
    md = MeshData().add(verts, faces, None, mat)
    if matrix is not None:
        md.transform(matrix)
    return md


def star_2d(r_in, r_out, arms, samples=180, sharp=0.7, width=0.25, phase=0.0):
    """Sternförmige Kontur (Bremsscheiben-/Kettenrad-Träger mit Armen)."""
    pts = []
    for i in range(samples):
        t = TAU * i / samples
        c = (math.cos(arms * (t - phase)) - (1 - 2 * width)) / (2 * width)
        f = clamp(c) ** sharp
        r = lerp(r_in, r_out, f)
        pts.append(V((r * math.cos(t), r * math.sin(t))))
    return pts


def teeth_2d(n_teeth, r_root, r_tip, samples_per_tooth=8, phase=0.0):
    pts = []
    for k in range(n_teeth * samples_per_tooth):
        t = k / samples_per_tooth
        f = t - math.floor(t)
        h = smoothstep(0.12, 0.38, f) * (1 - smoothstep(0.62, 0.88, f))
        r = lerp(r_root, r_tip, h)
        a = phase + TAU * t / n_teeth
        pts.append(V((r * math.cos(a), r * math.sin(a))))
    return pts


def chain_path(c1, r1, c2, r2, step):
    """Geschlossener Kettenverlauf um zwei Kettenräder (Ebene XZ). Gibt Punkte im Abstand ``step``."""
    d = V((c2.x - c1.x, c2.z - c1.z))
    D = d.length
    u = d / D
    p = V((-u.y, u.x))
    k = (r1 - r2) / D
    s = math.sqrt(max(0.0, 1 - k * k))
    pts = []

    def arc(c, r, a0, a1, n=48):
        if a1 < a0:
            a1 += TAU
        for i in range(n + 1):
            a = lerp(a0, a1, i / n)
            pts.append(V((c.x + r * math.cos(a), c1.y, c.z + r * math.sin(a))))
    n_top = k * u + s * p
    n_bot = k * u - s * p
    a_top = math.atan2(n_top.y, n_top.x)
    a_bot = math.atan2(n_bot.y, n_bot.x)
    # gegen den Uhrzeigersinn (in XZ): unten vorne → oben vorne, dann oben hinten → unten hinten
    arc(c1, r1, a_bot, a_top)
    arc(c2, r2, a_top, a_bot)
    pts.append(pts[0].copy())
    return resample(pts, step)


# =============================================================================
# 6) Enduro (300er Zweitakt, Maße an realen Hardenduros orientiert)
# =============================================================================

ENDURO_PALETTES = {
    "orange": dict(frame=(0.80, 0.14, 0.0), plastic=(0.86, 0.16, 0.0), panel=(0.80, 0.80, 0.78),
                   hub=(0.85, 0.20, 0.01), seat_stripe=(0.86, 0.16, 0.0)),
    "blue": dict(frame=(0.02, 0.06, 0.25), plastic=(0.01, 0.10, 0.55), panel=(0.80, 0.80, 0.78),
                 hub=(0.05, 0.15, 0.6), seat_stripe=(0.01, 0.1, 0.55)),
    "red": dict(frame=(0.55, 0.0, 0.0), plastic=(0.70, 0.01, 0.01), panel=(0.80, 0.80, 0.78),
                hub=(0.6, 0.02, 0.02), seat_stripe=(0.7, 0.01, 0.01)),
    "green": dict(frame=(0.02, 0.02, 0.02), plastic=(0.10, 0.55, 0.02), panel=(0.03, 0.03, 0.03),
                  hub=(0.1, 0.5, 0.02), seat_stripe=(0.1, 0.55, 0.02)),
    "yellow": dict(frame=(0.02, 0.05, 0.3), plastic=(0.90, 0.60, 0.0), panel=(0.04, 0.10, 0.45),
                   hub=(0.02, 0.06, 0.3), seat_stripe=(0.02, 0.06, 0.4)),
}


def enduro_materials(prefix, pal, dirt):
    P = f"M_{prefix}_"
    low = (0.15, 0.75)
    return {
        "frame": mat_paint(P + "Frame", pal["frame"], 0.3, coat=0.4, dirt=dirt * 0.8, dirt_z=(0.3, 0.8)),
        "plastic": mat_plastic(P + "Plastic", pal["plastic"], 0.42, dirt * 0.6, (0.25, 0.85), grain=0.02),
        "panel": mat_plastic(P + "Panel", pal["panel"], 0.42, dirt * 0.6, (0.3, 0.85), grain=0.02),
        "black": mat_plastic(P + "PlasticBlack", (0.018, 0.018, 0.02), 0.55, dirt * 0.7, low),
        "seat": mat_plastic(P + "Seat", (0.012, 0.012, 0.014), 0.75, 0.0, grain=0.25),
        "tire": mat_rubber(P + "Tire", (0.022, 0.022, 0.022), 0.88, dirt, (0.0, 0.55)),
        "rubber": mat_rubber(P + "Rubber", (0.03, 0.03, 0.03), 0.7, dirt * 0.4, (0.2, 0.7)),
        "rim": mat_metal(P + "Rim", (0.04, 0.04, 0.045), 0.32, dirt=dirt * 0.7, dirt_z=(0.0, 0.5)),
        "spoke": mat_metal(P + "Spoke", (0.62, 0.62, 0.64), 0.22, dirt=dirt * 0.5, dirt_z=(0.0, 0.45)),
        "hub": mat_metal(P + "Anodized", pal["hub"], 0.28, dirt=dirt * 0.5, dirt_z=(0.0, 0.5)),
        "alu": mat_metal(P + "Aluminium", (0.80, 0.80, 0.82), 0.28, brushed=True),
        "cast": mat_metal(P + "CastAlu", (0.62, 0.62, 0.64), 0.45, dirt=dirt * 0.6, dirt_z=(0.2, 0.6)),
        "chrome": mat_metal(P + "Chrome", (0.92, 0.92, 0.94), 0.04),
        "gold": mat_metal(P + "Stanchion", (0.86, 0.62, 0.28), 0.18),
        "steel": mat_metal(P + "SteelDark", (0.22, 0.22, 0.23), 0.42, dirt=dirt * 0.5, dirt_z=(0.2, 0.6)),
        "engine": mat_paint(P + "EngineCase", (0.05, 0.05, 0.055), 0.45, dirt=dirt * 0.7, dirt_z=(0.3, 0.7)),
        "pipe": mat_metal(P + "ExhaustPipe", (0.32, 0.29, 0.27), 0.38, dirt=dirt * 0.5, dirt_z=(0.3, 0.6)),
        "carbon": mat_carbon(P + "Carbon"),
        "radiator": mat_metal(P + "RadiatorCore", (0.10, 0.10, 0.11), 0.55),
        "lens": mat_glass(P + "Lens", (0.95, 0.95, 0.95)),
        "lamp": mat_emit(P + "HeadLight", (1.0, 0.95, 0.85), 6.0),
        "tail": mat_emit(P + "TailLight", (1.0, 0.02, 0.01), 4.0, base=(0.5, 0.01, 0.01)),
    }


class EnduroGeo:
    """Zentrale Geometriedaten (Achsen, Lenkkopf, Gabel) – alle Teile referenzieren diese."""

    def __init__(self):
        self.wheelbase = 1.48
        self.rake = math.radians(26.5)
        self.fork_offset = 0.022
        self.front_r, self.rear_r = 0.3477, 0.3406
        self.FA = V((self.wheelbase / 2, 0.0, self.front_r))
        self.RA = V((-self.wheelbase / 2, 0.0, self.rear_r))
        self.up = V((-math.sin(self.rake), 0.0, math.cos(self.rake)))     # Gabelachse aufwärts
        self.fwd = V((math.cos(self.rake), 0.0, math.sin(self.rake)))     # senkrecht dazu nach vorne
        self.fork_y = 0.105
        self.pivot = V((-0.16, 0.0, 0.50))                                # Schwingenlager
        self.crank = V((0.07, 0.0, 0.50))
        self.sprocket_f = V((-0.085, 0.082, 0.478))
        self.chain_y = 0.082

    def fork(self, s, y=0.0):
        """Punkt auf der Gabelachse, s = Abstand von der Vorderachse."""
        return self.FA + self.up * s + V((0, y, 0))

    def steer(self, s):
        """Punkt auf der Lenkachse (um den Gabelversatz hinter der Gabel)."""
        return self.FA + self.up * s - self.fwd * self.fork_offset

    def fork_frame(self, s, y=0.0):
        return frame_matrix(self.fork(s, y), self.fwd, Y_AXIS, self.up)


def build_wheel(M, center, r_rim, rim_w, tire_od, tire_w, knob_h, pattern, rows, hub_hw,
                spokes=36, seed=0):
    """Speichenrad mit Stollenreifen. Gibt dict(Name → MeshData) zurück."""
    rng = random.Random(seed)
    c = V(center)
    Rc = tire_od / 2 - knob_h
    r_bead = r_rim + 0.010
    phi0 = math.radians(122.0)
    a = (Rc - r_bead) / (1 - math.cos(phi0))
    r_c = Rc - a
    b = tire_w / 2 - 0.003
    h_bead = rim_w / 2 - 0.002

    def carcass(phi):
        r = r_c + a * math.cos(phi)
        ap = abs(phi)
        if ap <= math.pi / 2:
            h = b * math.sin(phi)
        else:
            t = (ap - math.pi / 2) / (phi0 - math.pi / 2)
            h = math.copysign(b - (b - h_bead) * t ** 1.6, phi)
        return r, h

    def radial(theta):
        return math.cos(theta) * Z_AXIS + math.sin(theta) * X_AXIS

    prof = [carcass(lerp(-phi0, phi0, i / 36)) for i in range(37)]
    tire = lathe(prof, 96, c, Y_AXIS, M["tire"])

    def surf(phi, theta):
        r, h = carcass(phi)
        r2, h2 = carcass(phi + 1e-4)
        dr, dh = (r2 - r) / 1e-4, (h2 - h) / 1e-4
        rad = radial(theta)
        p = c + Y_AXIS * h + rad * r
        n = (rad * dh - Y_AXIS * dr).normalized()
        t = (-math.sin(theta) * Z_AXIS + math.cos(theta) * X_AXIS)
        return p, n, t

    def knob(phi, theta, L, W, H, skew=0.0):
        p, n, t = surf(phi, theta)
        if skew:
            t = Quaternion(n, skew) @ t
        lat = n.cross(t).normalized()
        t = lat.cross(n).normalized()
        corners = ((-1, -1), (1, -1), (1, 1), (-1, 1))
        base = [p - n * 0.004 + t * (sx * L / 2) + lat * (sy * W / 2) for sx, sy in corners]
        top = [p + n * H + t * (sx * L * 0.40) + lat * (sy * W * 0.40) for sx, sy in corners]
        faces = [(4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]   # Unterseite steckt in der Karkasse
        tire.add(base + top, faces, None, M["tire"])

    for k in range(rows):
        th = TAU * k / rows
        for phi, L, W, hs, skew in pattern[k % len(pattern)]:
            jitter = rng.uniform(-0.002, 0.002)
            knob(phi, th + jitter, L, W, knob_h * hs, skew)

    # Felge (geschlossenes Hohlkammerprofil)
    hw = rim_w / 2 + 0.004
    r = r_rim
    rim_prof = [(r + 0.012, -hw), (r + 0.012, -hw + 0.004), (r + 0.002, -hw + 0.005), (r, -hw + 0.010),
                (r - 0.002, -0.008), (r - 0.010, -0.006), (r - 0.010, 0.006), (r - 0.002, 0.008),
                (r, hw - 0.010), (r + 0.002, hw - 0.005), (r + 0.012, hw - 0.004), (r + 0.012, hw),
                (r - 0.010, hw), (r - 0.018, hw - 0.006), (r - 0.020, 0.0), (r - 0.018, -hw + 0.006),
                (r - 0.010, -hw)]
    rim = lathe(rim_prof, 112, c, Y_AXIS, M["rim"], closed_profile=True)

    # Nabe
    hp = [(0.0125, -hub_hw), (0.024, -hub_hw), (0.024, -hub_hw + 0.008), (0.029, -hub_hw + 0.012),
          (0.029, -0.051), (0.050, -0.049), (0.050, -0.043), (0.031, -0.039), (0.026, -0.018),
          (0.026, 0.018), (0.031, 0.039), (0.050, 0.043), (0.050, 0.049), (0.029, 0.051),
          (0.029, hub_hw - 0.012), (0.024, hub_hw - 0.008), (0.024, hub_hw), (0.0125, hub_hw)]
    hub = lathe(hp, 40, c, Y_AXIS, M["hub"], closed_profile=True)
    hub.merge(cylinder(c - Y_AXIS * (hub_hw + 0.02), c + Y_AXIS * (hub_hw + 0.012), 0.0125, 16, M["chrome"]))
    for side in (-1, 1):
        nut = c + Y_AXIS * side * (hub_hw + 0.006)
        hub.merge(cylinder(nut - Y_AXIS * 0.006, nut + Y_AXIS * 0.006, 0.016, 6, M["steel"]))

    # Speichen (tangential gekreuzt) und Nippel
    sp = MeshData()
    half = spokes // 2
    for side in (-1, 1):
        for k in range(half):
            th = TAU * k / half + (TAU / spokes if side > 0 else 0.0)
            th_rim = th + (0.56 if k % 2 == 0 else -0.56)
            ph = c + Y_AXIS * (side * 0.046) + radial(th) * 0.044
            pr = c + Y_AXIS * (side * 0.005) + radial(th_rim) * (r_rim - 0.016)
            d = (pr - ph).normalized()
            sp.merge(cylinder(ph, pr, 0.0019, 6, M["spoke"], caps=False))
            sp.merge(cylinder(pr - d * 0.006, pr + d * 0.008, 0.0032, 6, M["chrome"]))
    return {"tire": tire, "rim": rim, "hub": hub, "spokes": sp}


def disc_matrix(center, y):
    """Ebene XZ bei Höhe y (lokal x→Welt Z, lokal y→Welt X, lokal z→Welt Y)."""
    return frame_matrix(V((center.x, y, center.z)), Z_AXIS, X_AXIS, Y_AXIS)


def slotted_ring(outer, r_in, r_s0, r_s1, period, slot, depth, matrix=None, mat=None):
    """Ring wie ``extrude_ring`` (Innenkontur Kreis), aber mit echten Durchbrüchen: im Band r_s0..r_s1
    bleiben die Segmente mit (Index % period) in ``slot`` offen; alle Randkanten bekommen Wände."""
    n = len(outer)
    rings = [[V((math.cos(TAU * i / n), math.sin(TAU * i / n))) * r for i in range(n)] for r in (r_in, r_s0, r_s1)]
    rings.append([V(p) for p in outer])
    bot = len(rings) * n
    verts = ([V((p[0], p[1], depth / 2)) for ring in rings for p in ring]
             + [V((p[0], p[1], -depth / 2)) for ring in rings for p in ring])
    top = [((r + 1) * n + i, (r + 1) * n + (i + 1) % n, r * n + (i + 1) % n, r * n + i)
           for r in range(len(rings) - 1) for i in range(n) if not (r == 1 and i % period in slot)]
    faces = top + [tuple(bot + k for k in reversed(f)) for f in top]
    edges = {(a, b) for f in top for a, b in zip(f, f[1:] + f[:1])}
    faces += [(b, a, bot + a, bot + b) for a, b in edges if (b, a) not in edges]     # Randkante → Wand
    md = MeshData().add(verts, faces, None, mat)
    if matrix is not None:
        md.transform(matrix)
    return md


def brake_disc(M, center, y, R, waves=18):
    m = disc_matrix(center, y)
    outer = [V((math.cos(t), math.sin(t))) * (R - 0.006 * (0.5 + 0.5 * math.cos(waves * t)))
             for t in (TAU * i / (waves * 6) for i in range(waves * 6))]
    # Lüftungsschlitze (je Welle einer, unter dem Wellenberg) im Reibring
    md = slotted_ring(outer, R - 0.032, R - 0.0245, R - 0.0145, 6, (2, 3), 0.004, m, M["steel"])
    md.merge(extrude(star_2d(0.040, R - 0.028, 6, 192, 0.6, 0.28), 0.005, m, M["hub"]))
    for i in range(6):
        t = TAU * i / 6 + TAU / 12
        p = m @ V((0.048 * math.cos(t), 0.048 * math.sin(t), 0.0))
        md.merge(cylinder(p - Y_AXIS * 0.005, p + Y_AXIS * 0.005, 0.005, 6, M["steel"]))
    return md


def sprocket(M, center, y, n_teeth, pitch=0.015875, carrier=True, mat=None):
    pr = pitch / (2 * math.sin(math.pi / n_teeth))
    m = disc_matrix(center, y)
    t = teeth_2d(n_teeth, pr - 0.0055, pr + 0.0040, 6)
    if carrier and n_teeth > 20:
        md = extrude_ring(t, circle_2d(pr - 0.022, len(t)), 0.005, m, mat or M["steel"])
        md.merge(extrude(star_2d(0.045, pr - 0.018, 6, 180, 0.5, 0.3, TAU / 12), 0.005, m, mat or M["steel"]))
    else:
        md = extrude(t, 0.006, m, mat or M["steel"])
    return md, pr


def caliper(M, center, y, R, angle, side, length=0.085):
    """Bremssattel, der die Scheibe bei ``angle`` (rad, in XZ von +Z) umgreift."""
    rad = math.cos(angle) * Z_AXIS + math.sin(angle) * X_AXIS
    tan = -math.sin(angle) * Z_AXIS + math.cos(angle) * X_AXIS
    p = V((center.x, y, center.z)) + rad * (R - 0.012)
    m = frame_matrix(p, tan, Y_AXIS, rad)
    body = extrude(rounded_rect_2d(length, 0.036, 0.012, 3), 0.05, None, M["cast"])
    body.transform(Matrix.Rotation(math.pi / 2, 4, "X"))
    md = body.transform(m)
    md.merge(box((0, side * 0.002, 0.02), (length * 0.7, 0.03, 0.02), M["cast"], m))
    for s in (-0.022, 0.022):
        q = m @ V((s, side * 0.021, -0.005))
        md.merge(cylinder(q - Y_AXIS * 0.006, q + Y_AXIS * 0.006, 0.006, 6, M["steel"]))
    return md, p, rad, tan


def build_enduro(cfg):
    g = EnduroGeo()
    pal = ENDURO_PALETTES.get(cfg.enduro_color, ENDURO_PALETTES["orange"])
    name = cfg.enduro_name
    M = enduro_materials(name, pal, cfg.dirt)
    asm = Assembly(name)
    asm.ao_distance = 0.25
    asm.pivot("Steering", g.steer(0.76), g.up, orient=True, positive="Lenkung nach links")
    asm.pivot("ForkLower", g.FA, g.up, parent="Steering", orient=True,
              positive="Verschiebung entlang +Y = Gabel federt ein", motion="slide")
    asm.pivot("WheelFront", g.FA, Y_AXIS, parent="ForkLower", positive="Rad rollt vorwärts", motion="spin")
    asm.pivot("Swingarm", g.pivot, Y_AXIS, positive="Hinterachse nach oben (einfedern)")
    asm.pivot("WheelRear", g.RA, Y_AXIS, parent="Swingarm", positive="Rad rollt vorwärts", motion="spin")
    bevel = ("bevel", dict(width=0.0025, segments=2, angle=35))
    bevel_s = ("bevel", dict(width=0.0012, segments=1, angle=40))

    # ------------------------------------------------------------- Räder
    fp = [[(0.0, 0.024, 0.030, 1.0, 0.0), (0.80, 0.022, 0.024, 0.95, 0.25), (-0.80, 0.022, 0.024, 0.95, -0.25)],
          [(0.40, 0.022, 0.024, 1.0, 0.1), (-0.40, 0.022, 0.024, 1.0, -0.1),
           (1.15, 0.020, 0.020, 0.85, 0.0), (-1.15, 0.020, 0.020, 0.85, 0.0)]]
    rp = [[(0.0, 0.026, 0.032, 1.0, 0.0), (0.72, 0.024, 0.028, 1.0, 0.2), (-0.72, 0.024, 0.028, 1.0, -0.2),
           (1.25, 0.022, 0.024, 0.85, 0.0), (-1.25, 0.022, 0.024, 0.85, 0.0)],
          [(0.36, 0.024, 0.028, 1.0, 0.1), (-0.36, 0.024, 0.028, 1.0, -0.1),
           (1.0, 0.024, 0.026, 0.95, 0.25), (-1.0, 0.024, 0.026, 0.95, -0.25)]]
    fw = build_wheel(M, g.FA, 0.2667, 0.041, 2 * g.front_r, 0.090, 0.014, fp, 46, 0.058, seed=cfg.seed)
    rw = build_wheel(M, g.RA, 0.2286, 0.055, 2 * g.rear_r, 0.140, 0.017, rp, 42, 0.070, seed=cfg.seed + 1)
    fw["hub"].merge(brake_disc(M, g.FA, 0.060, 0.130))
    rw["hub"].merge(brake_disc(M, g.RA, -0.064, 0.110))
    spr_r, pr_r = sprocket(M, g.RA, g.chain_y, 50, mat=M["hub"])
    rw["hub"].merge(spr_r)
    for pre, wheel, piv in (("WheelFront", fw, "WheelFront"), ("WheelRear", rw, "WheelRear")):
        asm.add(wheel["tire"], pre + "_Tire", piv, smooth=50, mods=[("bevel", dict(width=0.0015, segments=1, angle=55))])
        asm.add(wheel["rim"], pre + "_Rim", piv, smooth=40)
        asm.add(wheel["hub"], pre + "_Hub", piv, smooth=40, mods=[bevel_s])
        asm.add(wheel["spokes"], pre + "_Spokes", piv, smooth=60, detail=2)

    # ------------------------------------------------------------- Gabel
    lower, upper = MeshData(), MeshData()
    for side in (-1, 1):
        y = side * g.fork_y
        fm = g.fork_frame(0.0, y)
        foot = extrude(rounded_rect_2d(0.074, 0.056, 0.024, 4), 0.11, None, M["cast"]).translate((0.006, 0, 0.018))
        lower.merge(foot.transform(fm))
        lower.merge(cylinder(g.fork(0.05, y), g.fork(0.47, y), 0.024, 28, M["gold"]))
        lower.merge(cylinder(g.fork(-0.012, y) - Y_AXIS * side * 0.03, g.fork(-0.012, y) + Y_AXIS * side * 0.03,
                             0.006, 6, M["steel"]))

        def guard(u, v, fm=fm, side=side):
            a = lerp(-1.75, 1.75, v) + side * 0.25
            r = 0.035 + 0.004 * math.sin(math.pi * u)
            return fm @ V((r * math.cos(a), r * math.sin(a), lerp(0.075, 0.355, u)))
        asm.add(surface_grid(guard, 16, 14, M["black"]), f"ForkGuard_{'L' if side > 0 else 'R'}", "ForkLower",
                mods=[("solidify", dict(thickness=0.0025))])
        tube_prof = [(0.0, 0.392), (0.0355, 0.392), (0.0365, 0.402), (0.0355, 0.43), (0.0315, 0.44), (0.0315, 0.892),
                     (0.0, 0.892)]
        upper.merge(lathe(tube_prof, 32, g.fork(0, y), g.up, M["rim"]))
        cap = [(0.0, 0.89), (0.0262, 0.89), (0.0262, 0.914), (0.016, 0.918), (0.016, 0.93), (0.0, 0.93)]
        upper.merge(lathe(cap, 24, g.fork(0, y), g.up, M["hub"]))
    asm.add(lower, "ForkLower", "ForkLower", mods=[bevel_s])

    def clamp_outline():
        pts = []
        def arc(cx, cy, r, a0, a1, n):
            for i in range(n + 1):
                a = math.radians(lerp(a0, a1, i / n))
                pts.append(V((cx + r * math.cos(a), cy + r * math.sin(a))))
        arc(0.0, g.fork_y, 0.047, -10, 200, 16)
        arc(-0.035, 0.0, 0.055, 125, 235, 8)
        arc(0.0, -g.fork_y, 0.047, 160, 370, 16)
        return pts
    clamps = extrude(clamp_outline(), 0.045, g.fork_frame(0.6675), M["hub"])
    clamps.merge(extrude(clamp_outline(), 0.030, g.fork_frame(0.875), M["hub"]))
    for side in (-1, 1):
        for s, h in ((0.6675, 0.04), (0.875, 0.026)):
            m = g.fork_frame(s, side * (g.fork_y + 0.0))
            p = m @ V((-0.05, side * 0.012, 0))
            clamps.merge(cylinder(p - g.fwd * 0.004, p + g.fwd * 0.03, 0.005, 6, M["steel"]))
        riser = g.fork_frame(0.89) @ V((-0.012, side * 0.038, 0.0))
        clamps.merge(cylinder(riser, riser + g.up * 0.03, 0.015, 16, M["cast"]))
    bar_c = g.fork_frame(0.932) @ V((-0.012, 0.0, 0.0))
    clamps.merge(box(bar_c + V((0, 0, 0.004)), (0.045, 0.09, 0.03), M["cast"]))
    asm.add(clamps, "TripleClamps", "Steering", mods=[bevel])
    asm.add(upper, "ForkUpper", "Steering", mods=[bevel_s])

    # ------------------------------------------------------------- Lenker, Griffe, Hebel, Handprotektoren
    bar_ctrl = [(-0.07, -0.41, 0.078), (-0.05, -0.33, 0.068), (-0.016, -0.215, 0.028), (0.0, -0.12, 0.002),
                (0.0, 0.0, 0.0), (0.0, 0.12, 0.002), (-0.016, 0.215, 0.028), (-0.05, 0.33, 0.068),
                (-0.07, 0.41, 0.078)]
    bar_path = spline([bar_c + V(p) for p in bar_ctrl], 10)

    def bar_at(y):
        pts = sorted(bar_path, key=lambda p: p.y)
        for a, b in zip(pts, pts[1:]):
            if a.y <= y <= b.y:
                return a.lerp(b, (y - a.y) / max(b.y - a.y, 1e-9))
        return pts[0] if y < pts[0].y else pts[-1]

    bar = tube(bar_path, lambda s: lerp(0.0111, 0.0143, smoothstep(0.31, 0.24, abs(s - 0.5))), 16, M["alu"])
    asm.add(bar, "Handlebar", "Steering", smooth=50)
    ctl = MeshData()
    for side in (-1, 1):
        grip_pts = [p for p in bar_path if side * (p.y - bar_c.y) > 0.285]
        if side < 0:
            grip_pts.reverse()
        ctl.merge(tube(grip_pts, 0.0165, 16, M["rubber"]))
        ctl.merge(cylinder(bar_at(bar_c.y + side * 0.283), bar_at(bar_c.y + side * 0.290), 0.021, 16, M["rubber"]))
        perch = bar_at(bar_c.y + side * 0.215)
        ctl.merge(cylinder(perch - Y_AXIS * 0.012, perch + Y_AXIS * 0.012, 0.019, 16, M["black"]))
        body_len = 0.07 if side < 0 else 0.05
        ctl.merge(box(perch + V((0.022, -side * 0.015, 0.004)), (0.03, body_len, 0.028), M["black"]))
        if side < 0:
            res = perch + V((0.016, 0.02, 0.032))
            ctl.merge(box(res, (0.038, 0.042, 0.026), M["black"]))
            ctl.merge(box(res + V((0, 0, 0.014)), (0.04, 0.044, 0.004), M["cast"]))
        lever = spline([perch + V((0.04, side * 0.004, -0.004)), bar_at(bar_c.y + side * 0.30) + V((0.058, 0, -0.018)),
                        bar_at(bar_c.y + side * 0.375) + V((0.052, 0, -0.024))], 8)
        ctl.merge(tube(lever, (0.0045, 0.0085), 10, M["cast"], exponent=3.0))
        ctl.merge(cylinder(lever[-1] - V((0, side * 0.004, 0)), lever[-1] + V((0, side * 0.008, 0)), 0.007, 10, M["cast"]))
        # Handprotektor: Alu-Bügel + Kunststoffschale
        hb = spline([bar_at(bar_c.y + side * 0.20) + V((0.01, 0, -0.012)), bar_at(bar_c.y + side * 0.30) + V((0.085, 0, -0.03)),
                     bar_at(bar_c.y + side * 0.40) + V((0.06, 0, -0.03)), bar_at(bar_c.y + side * 0.43) + V((0.0, 0, 0.0))], 8)
        ctl.merge(tube(hb, 0.0075, 10, M["alu"]))

        def shield(u, v, side=side):
            y = bar_c.y + side * lerp(0.215, 0.425, u)
            base = bar_at(y)
            wrap = smoothstep(0.65, 1.0, u)
            x = base.x + 0.072 - 0.066 * wrap ** 1.4 + 0.012 * math.sin(math.pi * v)
            z = base.z - 0.25 * (base.z - bar_c.z) * u + lerp(-0.03, 0.036, v)
            return V((x, y + side * 0.012 * wrap, z))
        asm.add(surface_grid(shield, 18, 10, M["black"], flip=side < 0), f"HandGuard_{'L' if side > 0 else 'R'}",
                "Steering", mods=[("solidify", dict(thickness=0.003)), bevel_s])
    asm.add(ctl, "Controls", "Steering", mods=[bevel_s])

    # ------------------------------------------------------------- Lampenmaske, Scheinwerfer, Kotflügel vorne
    mfm = g.fork_frame(0.0)

    def mask_uv(u, v):
        uu = lerp(-1, 1, u)
        s = lerp(0.55, 0.945, v)
        w = 0.065 + 0.085 * math.sin(math.pi * clamp(v * 0.95 + 0.05)) ** 0.8 - 0.02 * v
        x = 0.068 + 0.05 * (1 - uu * uu) + 0.015 * v
        return x, uu * w, s

    def mask(u, v):
        return mfm @ V(mask_uv(u, v))

    def mask_keep(u, v):
        _, y, s = mask_uv(u, v)
        return not (abs(y) < 0.057 and abs(s - 0.78) < 0.029)
    mask_md = surface_grid(mask, 24, 26, M["plastic"], flip=True, keep=mask_keep)
    asm.add(mask_md.copy(), "HeadlightMask", "Steering", mods=[("solidify", dict(thickness=0.003)), bevel_s])
    lf = frame_matrix(g.fork(0.78) + g.fwd * 0.115, Y_AXIS, g.up, g.fwd)
    lamp = extrude(rounded_rect_2d(0.13, 0.068, 0.022, 4), 0.04, lf @ Matrix.Translation((0, 0, -0.012)), M["black"])
    lamp.merge(extrude(rounded_rect_2d(0.112, 0.052, 0.016, 4), 0.004, lf @ Matrix.Translation((0, 0, 0.007)), M["lamp"]))
    lamp.merge(extrude(rounded_rect_2d(0.12, 0.06, 0.02, 4), 0.006, lf @ Matrix.Translation((0, 0, 0.011)), M["lens"]))
    asm.add(lamp, "Headlight", "Steering", mods=[bevel_s])
    cfm = g.fork_frame(0.643)

    def fender_f(u, v):
        lx = lerp(-0.22, 0.33, u)
        vv = lerp(-1, 1, v)
        hw = 0.068 - 0.02 * abs(u - 0.45) ** 1.5
        ls = -0.004 - 0.5 * (lx - 0.04) ** 2 - 0.45 * max(0.0, lx - 0.22) ** 2 - 0.036 * vv * vv
        return cfm @ V((lx, vv * hw, ls))
    asm.add(surface_grid(fender_f, 28, 12, M["plastic"], flip=True), "FenderFront", "Steering",
            mods=[("solidify", dict(thickness=0.003))])

    # ------------------------------------------------------------- Rahmen
    fr = MeshData()
    fr.merge(cylinder(g.steer(0.635), g.steer(0.895), 0.028, 24, M["frame"]))
    for side in (-1, 1):
        spar = [g.steer(0.84) + V((-0.03, side * 0.028, 0.0)), (0.18, side * 0.086, 0.975), (-0.05, side * 0.113, 0.865),
                (-0.135, side * 0.113, 0.79), (-0.172, side * 0.108, 0.655)]
        fr.merge(tube(fillet_path(spar, 0.12, 8), (0.021, 0.0135), 16, M["frame"]))
        down = [g.steer(0.665) + V((0.012, side * 0.012, 0.0)), (0.414, side * 0.022, 0.80), (0.376, side * 0.052, 0.58),
                (0.30, side * 0.058, 0.385), (0.10, side * 0.062, 0.362), (-0.12, side * 0.08, 0.388),
                (-0.175, side * 0.098, 0.445)]
        fr.merge(tube(fillet_path(down, 0.09, 6), 0.0145, 14, M["frame"]))
        brace = [(0.2, side * 0.08, 0.955), (0.33, side * 0.04, 0.78)]
        fr.merge(tube(brace, 0.011, 12, M["frame"]))
        plate = rounded_poly_2d([(-0.238, 0.415), (-0.098, 0.428), (-0.102, 0.60), (-0.148, 0.705), (-0.205, 0.705),
                                 (-0.238, 0.55)], 0.025, 4)
        fr.merge(extrude(plate, 0.024, frame_matrix(V((0, side * 0.104, 0)), X_AXIS, Z_AXIS, -Y_AXIS), M["frame"]))
    fr.merge(cylinder(g.pivot - Y_AXIS * 0.125, g.pivot + Y_AXIS * 0.125, 0.012, 12, M["steel"]))
    for side in (-1, 1):
        fr.merge(cylinder(g.pivot + Y_AXIS * side * 0.118, g.pivot + Y_AXIS * side * 0.128, 0.02, 6, M["steel"]))
    asm.add(fr, "Frame", mods=[bevel_s])
    sub = MeshData()
    for side in (-1, 1):
        sub.merge(tube(fillet_path([(-0.08, side * 0.112, 0.845), (-0.30, side * 0.10, 0.865), (-0.64, side * 0.08, 0.885)],
                                   0.2), 0.0125, 12, M["cast"]))
        sub.merge(tube([(-0.20, side * 0.104, 0.68), (-0.56, side * 0.085, 0.87)], 0.0105, 12, M["cast"]))
    sub.merge(tube(fillet_path([(-0.60, -0.082, 0.88), (-0.64, 0.0, 0.89), (-0.60, 0.082, 0.88)], 0.05), 0.01, 12, M["cast"]))
    asm.add(sub, "Subframe")

    # ------------------------------------------------------------- Motor
    eng = MeshData()
    case = rounded_poly_2d([(0.19, 0.47), (0.17, 0.40), (0.09, 0.376), (-0.07, 0.382), (-0.135, 0.42), (-0.14, 0.53),
                            (-0.09, 0.60), (0.0, 0.625), (0.12, 0.615), (0.185, 0.565)], 0.03, 4)
    eng.merge(extrude(case, 0.150, frame_matrix(V((0, -0.009, 0)), X_AXIS, Z_AXIS, -Y_AXIS), M["engine"]))
    dome = lambda R, H: [(R + 0.002, -0.004), (R, 0.3 * H), (R * 0.88, 0.7 * H), (R * 0.55, 0.95 * H), (0.0, H)]
    eng.merge(lathe(dome(0.082, 0.03), 40, V((-0.012, -0.083, 0.505)), -Y_AXIS, M["engine"]))
    eng.merge(lathe(dome(0.07, 0.024), 40, V((0.075, 0.065, 0.50)), Y_AXIS, M["engine"]))
    eng.merge(lathe(dome(0.034, 0.018), 24, V((0.13, -0.083, 0.44)), -Y_AXIS, M["cast"]))
    for i in range(8):
        t = TAU * i / 8
        p = V((-0.012 + 0.074 * math.cos(t), -0.11, 0.505 + 0.074 * math.sin(t)))
        eng.merge(cylinder(p, p + Y_AXIS * 0.012, 0.0045, 6, M["steel"]))
    ax = V((math.sin(math.radians(15)), 0, math.cos(math.radians(15))))
    px = V((math.cos(math.radians(15)), 0, -math.sin(math.radians(15))))
    cyl_f = lambda h: frame_matrix(g.crank + ax * h, px, Y_AXIS, ax)
    eng.merge(extrude(rounded_rect_2d(0.105, 0.10, 0.02, 4), 0.135, cyl_f(0.1675), M["engine"]))
    eng.merge(extrude(rounded_rect_2d(0.115, 0.108, 0.03, 4), 0.05, cyl_f(0.26), M["cast"]))
    for k in range(5):
        eng.merge(extrude(rounded_rect_2d(0.112, 0.104, 0.024, 3), 0.004, cyl_f(0.12 + 0.025 * k), M["engine"]))
    eng.merge(lathe([(0.04, 0.0), (0.036, 0.012), (0.02, 0.022), (0.0, 0.025)], 24, g.crank + ax * 0.285, ax, M["cast"]))
    plug = g.crank + ax * 0.305
    eng.merge(cylinder(plug, plug + ax * 0.03, 0.011, 12, M["rubber"]))
    eng.merge(tube(spline([plug + ax * 0.025, plug + ax * 0.05 - px * 0.03, V((-0.05, 0.05, 0.86))], 6), 0.0045, 8, M["rubber"]))
    eng.merge(box((0.0, -0.065, 0.0), (0.05, 0.03, 0.045), M["cast"], cyl_f(0.17)))
    p_ex = g.crank + ax * 0.175 + px * 0.052
    eng.merge(cylinder(p_ex - px * 0.004, p_ex + px * 0.012, 0.03, 16, M["cast"]))
    p_in = g.crank + ax * 0.16 - px * 0.05
    intake = spline([p_in, p_in - px * 0.05 + V((0, 0, 0.025)), V((-0.105, 0.0, 0.73))], 8)
    eng.merge(tube(intake, 0.028, 16, M["rubber"]))
    eng.merge(cylinder(intake[len(intake) // 2] - (intake[-1] - intake[0]).normalized() * 0.03,
                       intake[len(intake) // 2] + (intake[-1] - intake[0]).normalized() * 0.03, 0.033, 16, M["cast"]))
    eng.merge(cylinder(V((-0.055, -0.07, 0.637)), V((-0.055, 0.05, 0.637)), 0.026, 16, M["engine"]))
    eng.merge(lathe(dome(0.026, 0.012), 16, V((-0.055, -0.07, 0.637)), -Y_AXIS, M["cast"]))
    for sx in (-1, 1):
        for sy in (-1, 1):
            p = cyl_f(0.285) @ V((sx * 0.043, sy * 0.041, 0.0))
            eng.merge(cylinder(p, p + ax * 0.012, 0.0055, 6, M["steel"]))
    for i in range(6):
        t = TAU * i / 6 + 0.3
        p = V((0.075 + 0.062 * math.cos(t), 0.088, 0.50 + 0.062 * math.sin(t)))
        eng.merge(cylinder(p - Y_AXIS * 0.004, p + Y_AXIS * 0.006, 0.0045, 6, M["steel"]))
    eng.merge(lathe([(0.0, 0.0), (0.016, 0.0), (0.016, 0.01), (0.012, 0.014), (0.0, 0.015)], 16,
                    V((-0.045, -0.112, 0.555)), -Y_AXIS, M["hub"]))
    cover = rounded_poly_2d([(-0.135, 0.425), (-0.035, 0.44), (-0.03, 0.535), (-0.125, 0.54)], 0.03, 3)
    eng.merge(extrude(cover, 0.012, frame_matrix(V((0, 0.098, 0)), X_AXIS, Z_AXIS, -Y_AXIS), M["black"]))
    asm.add(eng, "Engine", mods=[bevel])
    asm.add(*sprocket(M, g.sprocket_f, g.chain_y, 13)[:1], "SprocketFront", smooth=40)

    # Auspuffbirne (Expansionskammer) + Endschalldämpfer
    pipe_path = spline([p_ex + px * 0.01, (0.235, -0.02, 0.632), (0.298, -0.075, 0.54), (0.30, -0.115, 0.44),
                        (0.205, -0.14, 0.40), (0.04, -0.148, 0.43), (-0.12, -0.14, 0.535), (-0.26, -0.132, 0.65),
                        (-0.385, -0.13, 0.735)], 12)

    def pipe_r(s):
        if s < 0.10:
            return lerp(0.021, 0.024, s / 0.10)
        if s < 0.42:
            return lerp(0.024, 0.056, smoothstep(0.10, 0.42, s))
        if s < 0.58:
            return 0.056
        if s < 0.84:
            return lerp(0.056, 0.016, smoothstep(0.58, 0.84, s))
        return 0.0145
    asm.add(tube(pipe_path, pipe_r, 24, M["pipe"]), "ExhaustPipe", smooth=60)
    s0, s1 = V((-0.37, -0.13, 0.728)), V((-0.84, -0.148, 0.876))
    sd = (s1 - s0).normalized()
    sil = MeshData()
    sil.merge(tube([s0, s0 + sd * 0.07], lambda s: lerp(0.018, 0.048, smoothstep(0, 1, s)), 24, M["alu"]))
    sil.merge(tube([s0 + sd * 0.07, s1 - sd * 0.06], (0.049, 0.045), 24, M["alu"], caps=False, exponent=2.4))
    sil.merge(tube([s1 - sd * 0.06, s1], lambda s: (lerp(0.0495, 0.034, s ** 2), lerp(0.0455, 0.032, s ** 2)), 24,
                   M["carbon"], exponent=2.4))
    sil.merge(cylinder(s1 - sd * 0.01, s1 + sd * 0.03, 0.013, 16, M["steel"]))
    sil.merge(tube([s1 - sd * 0.03, s1 + sd * 0.03], 0.0105, 16, M["black"]))
    sil.merge(tube([s0 + sd * 0.25 + V((0, 0, 0.04)), V((-0.56, -0.09, 0.86))], (0.006, 0.014), 8, M["cast"], exponent=4))
    asm.add(sil, "Silencer", smooth=50, mods=[bevel_s])

    # Kühler
    rad = MeshData()
    for side in (-1, 1):
        rm = (Matrix.Translation((0.305, side * 0.098, 0.735)) @ Matrix.Rotation(side * math.radians(16), 4, "Z")
              @ Matrix.Rotation(math.radians(-18), 4, "Y"))
        rad.merge(box((0, 0, 0), (0.035, 0.13, 0.20), M["radiator"], rm))
        for z in (-0.108, 0.108):
            rad.merge(box((0, 0, z), (0.04, 0.136, 0.018), M["cast"], rm))
        rad.merge(cylinder(rm @ V((0, 0, -0.118)), rm @ V((0, 0, -0.14)), 0.006, 8, M["cast"]))
    for side in (-1, 1):
        hose = spline([(0.31, side * 0.06, 0.84), (0.25, side * 0.035, 0.82), (0.16, 0.0, 0.81), g.crank + ax * 0.27 + px * 0.04], 8)
        rad.merge(tube(hose, 0.011, 12, M["rubber"]))
        hose2 = spline([(0.29, side * 0.06, 0.63), (0.22, side * 0.05, 0.56), (0.16, -0.06, 0.47), (0.13, -0.09, 0.44)], 8)
        rad.merge(tube(hose2, 0.010, 12, M["rubber"]))
    asm.add(rad, "Radiators", mods=[bevel_s])

    # ------------------------------------------------------------- Verkleidung, Tank, Sitzbank
    def shroud(u, v):
        xf, xr = lerp(0.455, 0.345, v), lerp(0.02, -0.005, v)
        x = lerp(xf, xr, u)
        zb = lerp(0.655, 0.80, smoothstep(0.15, 1.0, u))
        zt = lerp(0.995, 0.945, u) - 0.012 * math.sin(math.pi * u)
        z = lerp(zb, zt, v)
        y = 0.122 + 0.052 * (1 - v) ** 1.3 * (1 - u) ** 0.8 + 0.012 * math.sin(math.pi * v)
        return V((x, y, z))
    sh = surface_grid(shroud, 30, 20, M["plastic"], flip=True)
    asm.add(sh.with_mirror(), "Shrouds", mods=[("solidify", dict(thickness=0.003)), bevel_s])
    lv = MeshData()
    for side in (-1, 1):
        for i in range(4):
            u0 = 0.18 + 0.08 * i
            p0, p1 = shroud(u0, 0.12), shroud(u0 + 0.05, 0.55)
            p0.y *= side
            p1.y *= side
            lv.merge(tube([p0 + V((0, side * 0.004, 0)), p1 + V((0, side * 0.004, 0))], (0.004, 0.008), 6, M["black"],
                          exponent=4))
    asm.add(lv, "ShroudVents", detail=3)
    tank = loft_stations([dict(x=-0.02, hw=0.086, zt=0.955, zb=0.80, et=2.6, eb=3.0),
                          dict(x=0.12, hw=0.10, zt=1.0, zb=0.77, et=2.6, eb=3.0),
                          dict(x=0.28, hw=0.094, zt=1.012, zb=0.80, et=2.4, eb=3.0),
                          dict(x=0.37, hw=0.05, zt=0.99, zb=0.90, et=2.2, eb=2.2)],
                         cosine_spacing(-0.02, 0.37, 24, 0.6, 0.8), 32, "ngon", V((0.375, 0, 0.95)), M["panel"])
    tank.merge(lathe([(0.03, 0.0), (0.03, 0.012), (0.026, 0.018), (0.0, 0.02)], 24, V((0.25, 0, 1.0)), Z_AXIS, M["hub"]))
    asm.add(tank, "FuelTank", smooth=40)
    seat = loft_stations([dict(x=-0.665, hw=0.075, zt=0.95, zb=0.90, et=3.0, eb=4.0),
                          dict(x=-0.60, hw=0.102, zt=0.962, zb=0.875, et=3.5, eb=4.0),
                          dict(x=-0.30, hw=0.116, zt=0.966, zb=0.868, et=3.5, eb=4.0),
                          dict(x=0.0, hw=0.10, zt=0.978, zb=0.875, et=3.2, eb=4.0),
                          dict(x=0.18, hw=0.068, zt=1.002, zb=0.935, et=2.6, eb=3.0),
                          dict(x=0.25, hw=0.03, zt=1.012, zb=0.98, et=2.2, eb=2.2)],
                         cosine_spacing(-0.665, 0.25, 40, 0.6, 0.6), 32, V((-0.672, 0, 0.93)), V((0.256, 0, 1.0)), M["seat"])
    asm.add(seat, "Seat", smooth=50)

    def side_panel(u, v):
        x = lerp(lerp(-0.07, -0.02, v), -0.545, u)
        zb = lerp(0.665, 0.80, u)
        z = lerp(zb, 0.878, v)
        y = 0.112 + 0.016 * math.sin(math.pi * v) * (1 - 0.5 * u) - 0.02 * u
        return V((x, y, z))
    sp_md = surface_grid(side_panel, 22, 12, M["panel"], flip=True)
    asm.add(sp_md.with_mirror(), "SidePanels", mods=[("solidify", dict(thickness=0.003)), bevel_s])

    # Grafik-Kit: Streifen auf den Kühlerspoilern, Startnummern auf Seitenteilen und Lampenmaske
    deco = MeshData()
    deco.merge(surface_decal(shroud, 0.04, 0.92, lambda u: 0.70 - 0.10 * u, lambda u: 0.84 - 0.07 * u, 30, 4,
                             M["black"], side=1).with_mirror())
    deco.merge(surface_decal(shroud, 0.10, 0.88, lambda u: 0.64 - 0.10 * u, lambda u: 0.675 - 0.10 * u, 26, 2,
                             M["panel"], side=1).with_mirror())
    number = str(cfg.seed % 1000 or 1)
    for side in (1, -1):
        fm = frame_matrix(V((-0.335, side * 0.30, 0.795)), -X_AXIS * side, Z_AXIS, Y_AXIS * side)
        txt = text_md(number, 0.15, fm, M["black"], bold=0.06)
        deco.merge(project_onto(txt, sp_md if side > 0 else sp_md.mirrored(), -Y_AXIS * side, 0.0018))
    asm.add(deco, "Decals", detail=1, smooth=60)
    txt = text_md(number, 0.08, frame_matrix(g.fork(0.882) + g.fwd * 0.3, Y_AXIS, g.up, g.fwd), M["black"], bold=0.06)
    asm.add(project_onto(txt, mask_md, -g.fwd, 0.0018), "DecalsFront", "Steering", detail=1, smooth=60)

    def fender_r(u, v):
        x = lerp(-0.40, -1.02, u)
        vv = lerp(-1, 1, v)
        hw = lerp(0.118, 0.07, u ** 1.3)
        zc = 0.868 + 0.07 * u ** 1.8
        return V((x, vv * hw, zc - 0.045 * vv * vv * (1 - 0.3 * u)))
    asm.add(surface_grid(fender_r, 26, 12, M["plastic"], flip=True), "FenderRear",
            mods=[("solidify", dict(thickness=0.003))])
    tl = frame_matrix(V((-0.93, 0, 0.885)), Y_AXIS, Z_AXIS, -X_AXIS)
    tlm = extrude(rounded_rect_2d(0.075, 0.03, 0.01, 3), 0.02, tl, M["black"])
    tlm.merge(extrude(rounded_rect_2d(0.065, 0.022, 0.008, 3), 0.004, tl @ Matrix.Translation((0, 0, 0.011)), M["tail"]))
    asm.add(tlm, "TailLight", mods=[bevel_s])

    # Skid-Plate, Fußrasten, Hebel
    def skid(u, v):
        x = lerp(-0.13, 0.25, u)
        vv = lerp(-1, 1, v)
        z = 0.348 + 0.11 * smoothstep(0.75, 1.0, u) ** 1.5 + 0.03 * vv * vv
        return V((x + 0.02 * smoothstep(0.85, 1.0, u), vv * (0.075 - 0.01 * u), z))
    asm.add(surface_grid(skid, 20, 10, M["black"]), "SkidPlate", mods=[("solidify", dict(thickness=0.004))])
    ped = MeshData()
    for side in (-1, 1):
        peg_m = frame_matrix(V((-0.075, side * 0.165, 0.426)), X_AXIS, Z_AXIS, -Y_AXIS)
        teeth = [(-0.024, -0.008), (0.024, -0.008)]
        for i in range(5):
            x0 = 0.024 - i * 0.0096
            teeth += [(x0, 0.010), (x0 - 0.0048, 0.006)]
        ped.merge(extrude(teeth, 0.085, peg_m, M["steel"]))
        ped.merge(box((-0.075, side * 0.118, 0.43), (0.05, 0.03, 0.04), M["cast"]))
    brk = spline([(-0.11, -0.13, 0.455), (-0.02, -0.142, 0.43), (0.08, -0.155, 0.415), (0.105, -0.16, 0.425)], 8)
    ped.merge(tube(brk, (0.010, 0.006), 10, M["cast"], exponent=4))
    ped.merge(box(brk[-1] + V((0.0, 0, 0.004)), (0.03, 0.035, 0.008), M["steel"]))
    shf = spline([(-0.03, 0.098, 0.505), (0.02, 0.135, 0.495), (0.075, 0.16, 0.50)], 8)
    ped.merge(tube(shf, (0.009, 0.006), 10, M["cast"], exponent=4))
    ped.merge(cylinder(shf[-1] - Y_AXIS * 0.005, shf[-1] + Y_AXIS * 0.035, 0.008, 10, M["steel"]))
    ks = fillet_path([(-0.10, 0.126, 0.418), (-0.30, 0.136, 0.400), (-0.43, 0.142, 0.432)], 0.06)
    ped.merge(tube(ks, 0.0105, 10, M["steel"]))
    ped.merge(box((-0.44, 0.146, 0.433), (0.05, 0.03, 0.012), M["steel"]))
    ped.merge(cylinder(V((-0.10, 0.11, 0.418)), V((-0.10, 0.145, 0.418)), 0.014, 12, M["steel"]))
    asm.add(ped, "Pegs", mods=[bevel_s])

    # ------------------------------------------------------------- Federbein (PDS)
    top, bot = V((-0.205, 0.0, 0.865)), V((-0.33, 0.0, 0.505))
    d = (bot - top).normalized()
    sh_up, sh_lo = MeshData(), MeshData()
    sh_up.merge(cylinder(top - Y_AXIS * 0.03, top + Y_AXIS * 0.03, 0.017, 16, M["cast"]))
    sh_up.merge(cylinder(top + d * 0.01, top + d * 0.20, 0.025, 24, M["cast"]))
    sh_up.merge(cylinder(top + d * 0.03 + V((0.04, 0, 0.0)), top + d * 0.16 + V((0.04, 0, 0)), 0.021, 20, M["hub"]))
    sh_up.merge(cylinder(top + d * 0.05, top + d * 0.062, 0.038, 24, M["hub"]))
    sh_lo.merge(cylinder(top + d * 0.18, bot - d * 0.03, 0.009, 16, M["chrome"]))
    sh_lo.merge(cylinder(bot - d * 0.07, bot - d * 0.055, 0.04, 24, M["hub"]))
    sh_lo.merge(cylinder(bot - d * 0.05, bot + d * 0.01, 0.018, 16, M["cast"]))
    sh_lo.merge(cylinder(bot - Y_AXIS * 0.025, bot + Y_AXIS * 0.025, 0.015, 16, M["cast"]))
    spring = helix(top + d * 0.065, d, 0.033, 0.245, 6.5, 20)
    sh_up.merge(tube(spring, 0.0055, 8, M["frame"]))
    asm.add(sh_up, "ShockUpper", mods=[bevel_s])
    asm.add(sh_lo, "ShockLower", "Swingarm", mods=[bevel_s])

    # ------------------------------------------------------------- Schwinge, Kette, Bremse hinten
    sw = MeshData()
    for side in (-1, 1):
        arm = spline([(g.pivot.x, side * 0.09, g.pivot.z), (-0.30, side * 0.098, 0.468), (-0.56, side * 0.104, 0.385),
                      (g.RA.x - 0.02, side * 0.106, g.RA.z)], 8)
        sw.merge(tube(arm, lambda s: (lerp(0.034, 0.022, s), lerp(0.018, 0.014, s)), 16, M["cast"], exponent=3.2))
        sw.merge(box((g.RA.x, side * 0.106, g.RA.z), (0.06, 0.03, 0.045), M["cast"]))
        sw.merge(cylinder(V((g.RA.x - 0.04, side * 0.106, g.RA.z)), V((g.RA.x - 0.065, side * 0.106, g.RA.z)),
                          0.005, 6, M["steel"]))
    sw.merge(cylinder(g.pivot - Y_AXIS * 0.1, g.pivot + Y_AXIS * 0.1, 0.028, 24, M["cast"]))
    sw.merge(tube(fillet_path([(-0.33, -0.096, 0.46), (-0.33, -0.03, 0.485), (-0.33, 0.03, 0.485), (-0.33, 0.096, 0.46)],
                              0.03), (0.016, 0.028), 12, M["cast"], exponent=3))
    sw.merge(box((-0.33, 0.0, 0.495), (0.05, 0.05, 0.03), M["cast"]))
    slider = spline([(-0.12, 0.09, 0.50), (-0.20, 0.091, 0.515), (-0.36, 0.096, 0.475)], 8)
    sw.merge(tube(slider, (0.012, 0.022), 12, M["black"], exponent=4))
    asm.add(sw, "Swingarm", "Swingarm", mods=[bevel])
    ch = MeshData()
    pr_f = 0.015875 / (2 * math.sin(math.pi / 13))
    cpts = chain_path(V((g.sprocket_f.x, g.chain_y, g.sprocket_f.z)), pr_f, V((g.RA.x, g.chain_y, g.RA.z)), pr_r, 0.015875)
    for i, (p0, p1) in enumerate(zip(cpts, cpts[1:])):
        dirv = (p1 - p0).normalized()
        mid = (p0 + p1) / 2
        m = frame_matrix(mid, dirv, Y_AXIS, dirv.cross(Y_AXIS))
        off = 0.0095 if i % 2 else 0.0068
        for s in (-1, 1):
            ch.merge(extrude(rounded_rect_2d(0.022, 0.012, 0.0055, 1), 0.0016, None, M["steel"])
                     .transform(Matrix.Rotation(math.pi / 2, 4, "X")).translate((0, s * off, 0)).transform(m))
        ch.merge(cylinder(p0 - Y_AXIS * 0.0085, p0 + Y_AXIS * 0.0085, 0.0052, 6, M["gold"]))
    asm.add(ch, "Chain", "Swingarm", smooth=50, detail=2)
    cal_r, cp, cr, ct = caliper(M, g.RA, -0.064, 0.110, math.radians(200), -1)
    cal_r.merge(tube([cp + cr * 0.02, V((g.RA.x, -0.10, g.RA.z))], (0.012, 0.006), 8, M["cast"], exponent=4))
    asm.add(cal_r, "CaliperRear", "Swingarm", mods=[bevel_s])
    cal_f, cpf, crf, ctf = caliper(M, g.FA, 0.060, 0.130, math.radians(-145), 1)
    cal_f.merge(tube([cpf + crf * 0.015, g.fork(0.05, g.fork_y) - g.fwd * 0.02], (0.012, 0.008), 8, M["cast"], exponent=4))
    asm.add(cal_f, "CaliperFront", "ForkLower", mods=[bevel_s])

    # ------------------------------------------------------------- Leitungen
    lines = MeshData()
    mc = bar_at(bar_c.y - 0.215) + V((0.03, 0.03, 0.0))
    lines.merge(tube(spline([mc, mc + V((0.05, 0.06, -0.05)), g.fork(0.70) + g.fwd * 0.085, g.fork(0.56, 0.135) + g.fwd * 0.012,
                             g.fork(0.30, 0.142) - g.fwd * 0.01, cpf + crf * 0.03 + Y_AXIS * 0.01], 10), 0.0042, 8, M["black"]))
    cl = bar_at(bar_c.y + 0.215) + V((0.03, -0.03, 0.0))
    lines.merge(tube(spline([cl, cl + V((0.06, -0.04, -0.06)), V((0.36, 0.06, 0.95)), V((0.25, 0.12, 0.78)),
                             V((-0.03, 0.085, 0.585))], 10), 0.0042, 8, M["black"]))
    th = bar_at(bar_c.y - 0.29)
    lines.merge(tube(spline([th + V((0.01, 0.03, 0.01)), th + V((0.05, 0.10, -0.03)), V((0.30, -0.05, 0.98)),
                             V((0.12, -0.04, 0.86)), intake[len(intake) // 2] + V((0, -0.03, 0.02))], 10), 0.0035, 8,
                     M["black"]))
    asm.add(lines, "Lines", "Steering", smooth=60, detail=3)

    # ------------------------------------------------------------- Sockets & Kollision
    asm.socket("Rider_Seat", (-0.18, 0, 0.98))
    asm.socket("Peg_L", (-0.075, 0.165, 0.44))
    asm.socket("Peg_R", (-0.075, -0.165, 0.44))
    asm.socket("Grip_L", bar_at(bar_c.y + 0.35))
    asm.socket("Grip_R", bar_at(bar_c.y - 0.35))
    asm.socket("Exhaust", s1 + sd * 0.03, forward=sd)
    asm.socket("Headlight", g.fork(0.80) + g.fwd * 0.13, forward=V((1.0, 0.0, -0.03)), parent="Steering")
    wheels = [o for o in asm.meshes() if o["pivot"] in ("WheelFront", "WheelRear")]
    asm.collide("Body", [o for o in asm.meshes() if o not in wheels])
    # ganzes Rad (Reifen, Nabe, Bremsscheibe, Kettenrad) – die Scheibe steht seitlich über den Reifen hinaus
    asm.collide("WheelFront", [o for o in wheels if o["pivot"] == "WheelFront"], max_verts=48)
    asm.collide("WheelRear", [o for o in wheels if o["pivot"] == "WheelRear"], max_verts=48)
    return asm


# =============================================================================
# 7) Hilfen für Booleans, Normalen und Profile
# =============================================================================

def apply_modifiers(obj):
    """Alle Modifier über den Depsgraph anwenden (ohne bpy.ops, funktioniert headless)."""
    if not obj.modifiers:
        return obj
    dg = bpy.context.evaluated_depsgraph_get()
    ev = obj.evaluated_get(dg)
    me = bpy.data.meshes.new_from_object(ev, preserve_all_data_layers=True, depsgraph=dg)
    old = obj.data
    obj.modifiers.clear()
    obj.data = me
    me.name = old.name
    if old.users == 0:
        bpy.data.meshes.remove(old)
    return obj


def boolean(obj, cutters, operation="DIFFERENCE"):
    """Exakter Boolean gegen eine Liste von Cutter-Objekten (Materialien werden übernommen)."""
    coll = bpy.data.collections.new("_cutters")
    bpy.context.scene.collection.children.link(coll)
    for c in cutters:
        coll.objects.link(c)
    if obj.name not in bpy.context.scene.collection.all_objects:
        bpy.context.scene.collection.objects.link(obj)
        temp_link = True
    else:
        temp_link = False
    mod = obj.modifiers.new("Bool", "BOOLEAN")
    mod.operation = operation
    mod.solver = "EXACT"
    mod.operand_type = "COLLECTION" if len(cutters) > 1 else "OBJECT"
    if len(cutters) > 1:
        mod.collection = coll
    else:
        mod.object = cutters[0]
    mod.material_mode = "TRANSFER"
    apply_modifiers(obj)
    for c in cutters:
        me = c.data
        bpy.data.objects.remove(c, do_unlink=True)
        bpy.data.meshes.remove(me)
    bpy.data.collections.remove(coll)
    if temp_link:
        bpy.context.scene.collection.objects.unlink(obj)
    return obj


def transfer_normals(obj, src_me, tol=0.004):
    """Glatte Normalen der ungeschnittenen Fläche auf ein Boolean-Ergebnis übertragen.

    Verhindert die typischen Shading-Artefakte an Ausschnitten (Fenster, Türen):
    Faces auf der Originalfläche erhalten interpolierte Original-Normalen,
    neue Schnittkanten-Faces bleiben flach.
    """
    src_me.calc_loop_triangles()
    sv = [v.co.copy() for v in src_me.vertices]
    tris = [tuple(t.vertices) for t in src_me.loop_triangles]
    tri_loops = [tuple(t.loops) for t in src_me.loop_triangles]
    cn = [c.vector.copy() for c in src_me.corner_normals]
    bvh = BVHTree.FromPolygons(sv, tris)
    me = obj.data
    out = []
    for poly in me.polygons:
        fn = poly.normal.copy()
        _, n, idx, dist = bvh.find_nearest(poly.center)
        on_surface = idx is not None and dist < tol and n.dot(fn) > 0.5
        for li in poly.loop_indices:
            nrm = fn
            if on_surface:
                co = me.vertices[me.loops[li].vertex_index].co
                loc, n2, idx2, _ = bvh.find_nearest(co)
                if idx2 is not None and n2.dot(fn) > 0.3:
                    a, b, c = tris[idx2]
                    w = poly_3d_calc([sv[a], sv[b], sv[c]], loc)
                    la, lb, lc = tri_loops[idx2]
                    cand = cn[la] * w[0] + cn[lb] * w[1] + cn[lc] * w[2]
                    if cand.length > 1e-6:
                        nrm = cand.normalized()
            out.append(nrm)
    me.polygons.foreach_set("use_smooth", [True] * len(me.polygons))
    me.normals_split_custom_set(out)


def prism_y(outline_xz, y0, y1, mat=None):
    """Prisma aus einer Seitenansichts-Kontur (x, z), extrudiert zwischen y0 und y1."""
    m = frame_matrix(V((0, (y0 + y1) / 2, 0)), X_AXIS, Z_AXIS, -Y_AXIS)
    return extrude(outline_xz, abs(y1 - y0), m, mat)


def prism_x(outline_yz, x0, x1, mat=None):
    """Prisma aus einer Front-/Heckansichts-Kontur (y, z), extrudiert zwischen x0 und x1."""
    m = frame_matrix(V(((x0 + x1) / 2, 0, 0)), Y_AXIS, Z_AXIS, X_AXIS)
    return extrude(outline_yz, abs(x1 - x0), m, mat)


def prism_z(outline_xy, z0, z1, mat=None):
    """Prisma aus einer Draufsichts-Kontur (x, y), extrudiert zwischen z0 und z1."""
    return extrude(outline_xy, abs(z1 - z0), Matrix.Translation(V((0, 0, (z0 + z1) / 2))), mat)


def offset_poly(pts, d):
    """2D-Kontur (CCW) um d nach außen (d>0) bzw. innen (d<0) versetzen."""
    pts = ensure_ccw([V(p) for p in pts])
    n = len(pts)
    out = []
    for i in range(n):
        p0, p1, p2 = pts[i - 1], pts[i], pts[(i + 1) % n]
        e0 = (p1 - p0).normalized()
        e1 = (p2 - p1).normalized()
        n0 = V((e0.y, -e0.x))
        n1 = V((e1.y, -e1.x))
        nm = (n0 + n1)
        if nm.length < 1e-6:
            nm = n0
        nm.normalize()
        out.append(p1 + nm * (d / max(nm.dot(n0), 0.35)))
    return out


def point_in_poly(p, poly):
    x, y = p
    inside = False
    n = len(poly)
    for i in range(n):
        x0, y0 = poly[i - 1]
        x1, y1 = poly[i]
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            inside = not inside
    return inside


def vertex_normals(md):
    acc = [V() for _ in md.verts]
    for f in md.faces:
        p = [md.verts[i] for i in f]
        n = V()
        for i in range(len(p)):
            n += p[i - 1].cross(p[i])
        for i in f:
            acc[i] += n
    return [a.normalized() if a.length > 1e-12 else Z_AXIS.copy() for a in acc]


def glass_patch(md, normals, region, inset, mat, y_test=None):
    """Glasfläche aus einem Loft: alle Faces mit Vertex in ``region`` (Seitenansicht),
    nach innen versetzt – die Ränder verschwinden in der Wandstärke der Hülle."""
    out = MeshData()
    sel = []
    for i, f in enumerate(md.faces):
        if any(point_in_poly((md.verts[k].x, md.verts[k].z), region) and (y_test is None or y_test(md.verts[k].y))
               for k in f):
            sel.append(i)
    used = sorted({k for i in sel for k in md.faces[i]})
    remap = {k: n for n, k in enumerate(used)}
    out.verts = [md.verts[k] - normals[k] * inset for k in used]
    out.faces = [tuple(remap[k] for k in md.faces[i]) for i in sel]
    out.uvs = [md.uvs[i] for i in sel]
    out.mats = [mat] * len(sel)
    return out


def text_md(text, size, matrix, mat, resolution=3, bold=0.0):
    """Schriftzug als flaches Mesh (z.B. Startnummer, Kennzeichen) – lokal XY, zentriert.
    ``bold`` verdickt die Kontur (Anteil der Schriftgröße)."""
    cu = bpy.data.curves.new("_txt", "FONT")
    cu.body = text
    cu.size = size
    cu.offset = size * bold
    cu.align_x = "CENTER"
    cu.align_y = "CENTER"
    cu.resolution_u = resolution
    ob = bpy.data.objects.new("_txt", cu)
    bpy.context.scene.collection.objects.link(ob)
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(ob.evaluated_get(dg))
    md = MeshData()
    md.verts = [matrix @ v.co for v in me.vertices]
    md.faces = [tuple(p.vertices) for p in me.polygons]
    md.uvs = [None] * len(md.faces)
    md.mats = [mat] * len(md.faces)
    bpy.data.objects.remove(ob, do_unlink=True)
    bpy.data.curves.remove(cu)
    bpy.data.meshes.remove(me)
    return md


def subdivide_tris(md, max_edge=0.025):
    """Dreiecke so lange halbieren, bis keine Kante länger als ``max_edge`` ist (für Decals)."""
    faces = []
    for f in md.faces:
        for i in range(1, len(f) - 1):
            faces.append((f[0], f[i], f[i + 1]))
    verts = [v.copy() for v in md.verts]
    mat = md.mats[0] if md.mats else None

    def ek(a, b):
        return (min(a, b), max(a, b))
    for _ in range(10):
        # konform unterteilen: geteilte Kanten werden in beiden angrenzenden Dreiecken geteilt (keine T-Risse)
        split = set()
        for a, b, c in faces:
            if max((verts[a] - verts[b]).length, (verts[b] - verts[c]).length, (verts[c] - verts[a]).length) > max_edge:
                split.update((ek(a, b), ek(b, c), ek(c, a)))
        if not split:
            break
        mids = {}
        for k in split:
            mids[k] = len(verts)
            verts.append((verts[k[0]] + verts[k[1]]) / 2)
        out = []
        for a, b, c in faces:
            m = [mids.get(ek(a, b)), mids.get(ek(b, c)), mids.get(ek(c, a))]
            n = sum(x is not None for x in m)
            if n == 0:
                out.append((a, b, c))
                continue
            # rotieren, sodass die Ausgangsform einheitlich behandelt werden kann
            tri, mm = [a, b, c], m
            for r in range(3):
                if n == 1 and mm[0] is not None or n == 2 and mm[2] is None or n == 3:
                    break
                tri = tri[1:] + tri[:1]
                mm = mm[1:] + mm[:1]
            a_, b_, c_ = tri
            ab, bc, ca = mm
            if n == 3:
                out += [(a_, ab, ca), (ab, b_, bc), (ca, bc, c_), (ab, bc, ca)]
            elif n == 2:            # Kanten ab und bc geteilt
                out += [(ab, b_, bc), (a_, ab, bc), (a_, bc, c_)]
            else:                   # nur ab geteilt
                out += [(a_, ab, c_), (ab, b_, c_)]
        faces = out
    res = MeshData()
    res.verts, res.faces = verts, faces
    res.uvs, res.mats = [None] * len(faces), [mat] * len(faces)
    return res.compact()


def project_onto(md, target, direction, offset=0.0015, max_edge=0.025):
    """Mesh-Decal: alle Vertices entlang ``direction`` auf die Zielfläche (MeshData) projizieren.
    ``max_edge`` an die Krümmung anpassen (stark gewölbt → klein, flach → groß spart Dreiecke)."""
    md = subdivide_tris(md, max_edge)
    tris = []
    for f in target.faces:
        for i in range(1, len(f) - 1):
            tris.append((f[0], f[i], f[i + 1]))
    bvh = BVHTree.FromPolygons(target.verts, tris)
    d = V(direction).normalized()
    keep = []
    for i, v in enumerate(md.verts):
        hit = bvh.ray_cast(v - d * 1.0, d, 3.0)
        if hit[0] is not None:
            md.verts[i] = hit[0] - d * offset
            keep.append(i)
    return md


def surface_decal(fn, u0, u1, v_lo, v_hi, nu, nv, mat, offset=0.0016, side=1.0):
    """Decal exakt auf einer analytischen Fläche fn(u, v): Bereich u0..u1, v_lo(u)..v_hi(u).

    ``side`` = +1 legt das Decal auf die Seite von dP/du x dP/dv, -1 auf die Gegenseite.
    """
    def g(a, b):
        u = lerp(u0, u1, a)
        v = lerp(v_lo(u), v_hi(u), b)
        p = V(fn(u, v))
        n = (V(fn(u + 1e-3, v)) - p).cross(V(fn(u, v + 1e-3)) - p).normalized() * side
        return p + n * offset
    return surface_grid(g, nu, nv, mat, flip=side > 0)


def naca4(x, t=0.12, camber=0.0, camber_pos=0.4):
    """NACA-4-Ziffern: (Wölbungslinie yc, halbe Dicke yt) an Sehnenposition x ∈ [0, 1]."""
    x = clamp(x)
    yt = 5 * t * (0.2969 * math.sqrt(x) - 0.1260 * x - 0.3516 * x ** 2 + 0.2843 * x ** 3 - 0.1036 * x ** 4)
    p = camber_pos
    if camber <= 0:
        yc = 0.0
    elif x < p:
        yc = camber / p ** 2 * (2 * p * x - x * x)
    else:
        yc = camber / (1 - p) ** 2 * ((1 - 2 * p) + 2 * p * x - x * x)
    return yc, yt


def airfoil_ring(n=24, t=0.12, camber=0.0, x0=0.0, x1=1.0, nose=8, stations=None):
    """Profil-Ring (CCW: Hinterkante → Oberseite → Nase → Unterseite) für den Sehnenbereich x0..x1.

    x1 < 1: stumpfer Schnitt hinten (Flügelkasten vor Klappe/Querruder),
    x0 > 0: abgerundete Nase am Scharnier (das Ruder selbst).
    ``stations``: gemeinsame Sehnenpositionen, damit Teilringe exakt auf dem vollen Ring liegen
    (aneinandergrenzende Flügelsegmente ohne Versatz).
    """
    xs = [x0 + (x1 - x0) * 0.5 * (1 - math.cos(math.pi * i / n)) for i in range(n + 1)]
    if stations is not None:
        xs = [x for x in stations if x0 - 1e-9 <= x <= x1 + 1e-9]
    up, lo = [], []
    for x in xs:
        yc, yt = naca4(x, t, camber)
        up.append(V((x, yc + yt)))
        lo.append(V((x, yc - yt)))
    if x0 <= 1e-9:
        ring = list(reversed(up)) + lo[1:]
        return ring[:-1] if x1 >= 1.0 else ring
    yc0, yt0 = naca4(x0, t, camber)
    arc = [V((x0 - yt0 * math.sin(math.pi * i / nose), yc0 + yt0 * math.cos(math.pi * i / nose)))
           for i in range(1, nose)]
    return list(reversed(up)) + arc + lo[:-1]


def airfoil_loft(sections, n_af=20, mat=None, cap=True, ring=None):
    """Tragflächen-/Rotorblatt-Loft. Jede Sektion: dict(le, chord_dir, thick_dir, chord, t).

    ``ring``: eigener 2D-Profilring (z.B. aus airfoil_ring), sonst symmetrisches NACA 0012.
    Der Ring wird mit t/0.12 skaliert; eine Sektion kann mit dict(..., ring=...) einen eigenen,
    unskalierten Ring mitbringen (gleiche Punktzahl), z.B. um die Wölbung unabhängig von der Dicke zu halten.
    """
    base = ring or airfoil_2d(n_af, 0.12)
    rings = []
    for s in sections:
        cd = V(s["chord_dir"]).normalized()
        td = V(s["thick_dir"]).normalized()
        pts = s.get("ring", base)                    # eigener Ring je Sektion: Dicke schon enthalten
        k = 1.0 if "ring" in s else s["t"] / 0.12
        rings.append([V(s["le"]) + cd * (p.x * s["chord"]) + td * (p.y * s["chord"] * k) for p in pts])
    c0 = sum(rings[0], V()) / len(rings[0])
    c1 = sum(rings[-1], V()) / len(rings[-1])
    md = grid_rings(rings, True, c0 if cap else None, c1 if cap else None, mat)
    md.recalc = True
    return md


# =============================================================================
# 8) Hubschrauber (leichter einmotoriger Turbinenhubschrauber, ~10,7 m Rotor)
# =============================================================================

HELI_LIVERIES = {
    "civil": dict(base=(0.80, 0.81, 0.80), bands=[(-1.0, 0.97, (0.01, 0.035, 0.13)), (1.03, 1.10, (0.0, 0.22, 0.55))],
                  rough=0.26, coat=0.7, text=(0.01, 0.035, 0.13), label=None),
    "police": dict(base=(0.78, 0.80, 0.82), bands=[(-1.0, 1.02, (0.0, 0.04, 0.24)), (1.07, 1.12, (0.62, 0.64, 0.66))],
                   rough=0.26, coat=0.7, text=(0.0, 0.04, 0.24), label="POLIZEI"),
    "rescue": dict(base=(0.85, 0.52, 0.0), bands=[(-1.0, 0.92, (0.72, 0.72, 0.72)), (0.97, 1.02, (0.02, 0.02, 0.02))],
                   rough=0.26, coat=0.7, text=(0.02, 0.02, 0.02), label="RESCUE"),
    "military": dict(base=(0.055, 0.07, 0.04), bands=[(-1.0, 0.9, (0.04, 0.05, 0.03))], rough=0.7, coat=0.0,
                     text=(0.02, 0.022, 0.02), label=None),
}


def heli_materials(prefix, liv, dirt):
    P = f"M_{prefix}_"
    return {
        "paint": mat_paint(P + "Paint", liv["base"], liv["rough"], coat=liv["coat"], livery=liv["bands"],
                           dirt=dirt * 0.25, dirt_z=(0.4, 0.9), seams=HeliGeo.SEAMS),
        "interior": uv_weight(mat_plastic(P + "Interior", (0.05, 0.05, 0.055), 0.75, grain=0.15), 0.5),
        "fabric": uv_weight(mat_plastic(P + "SeatFabric", (0.025, 0.027, 0.032), 0.9, grain=0.3), 0.5),
        "floor": uv_weight(mat_rubber(P + "Floor", (0.03, 0.03, 0.03), 0.85), 0.5),
        "seal": mat_rubber(P + "Seal", (0.012, 0.012, 0.012), 0.6),
        "glass": mat_glass(P + "Glass", (0.62, 0.70, 0.74)),
        "metal": mat_metal(P + "Metal", (0.55, 0.56, 0.58), 0.32),
        "dark": mat_metal(P + "DarkMetal", (0.06, 0.06, 0.065), 0.45),
        "steel": mat_metal(P + "Steel", (0.35, 0.35, 0.36), 0.3),
        "skid": mat_metal(P + "Skid", (0.62, 0.62, 0.64), 0.3, brushed=True),
        "soot": mat_metal(P + "Exhaust", (0.08, 0.07, 0.065), 0.65),
        "blade": mat_rotor_blade(P + "Blade", (0.035, 0.035, 0.04), [(5.02, 5.40, (0.9, 0.62, 0.0))], 0.45),
        "tblade": mat_rotor_blade(P + "TailBlade", (0.035, 0.035, 0.04),
                                  [(0.60, 0.70, (0.8, 0.8, 0.8)), (0.70, 0.80, (0.75, 0.03, 0.02)),
                                   (0.80, 0.95, (0.8, 0.8, 0.8))], 0.45, axis="Y"),
        "decal": uv_weight(mat_paint(P + "Decal", liv["text"], 0.3, coat=liv["coat"]), 2.0),
        "dark_paint": mat_paint(P + "DarkPaint", (0.02, 0.02, 0.022), 0.35, coat=0.3),
        "panel": uv_weight(mat_plastic(P + "Panel", (0.018, 0.018, 0.02), 0.55), 0.5),
        "screen": uv_weight(mat_emit(P + "Screen", (0.10, 0.22, 0.32), 1.5, base=(0.01, 0.01, 0.012)), 0.5),
        "grille": mat_metal(P + "Grille", (0.03, 0.03, 0.03), 0.6),
        "nav_red": mat_emit(P + "NavRed", (1.0, 0.02, 0.01), 12.0),
        "nav_green": mat_emit(P + "NavGreen", (0.02, 1.0, 0.1), 12.0),
        "nav_white": mat_emit(P + "NavWhite", (1.0, 1.0, 1.0), 10.0),
        "landing": mat_emit(P + "LandingLight", (1.0, 0.96, 0.9), 15.0),
    }


class HeliGeo:
    """Rumpf-Spanten (Seitenansicht/Querschnitt) und Hauptmaße."""

    # Panel-Fugen (Wartungsklappen der Triebwerksverkleidung, Heckausleger-Segmente)
    SEAMS = [(0, 0.32, 1.93, 2.6), (0, -0.62, 1.93, 2.6), (0, -1.55, 1.93, 2.6), (2, 2.012, 1.95, 2.03),
             (0, -2.95, 1.2, 1.95), (0, -4.35, 1.2, 1.9), (0, -5.75, 1.2, 1.9)]

    STATIONS = [
        dict(x=-6.66, hw=0.098, zt=1.675, zb=1.465),
        dict(x=-5.0, hw=0.128, zt=1.665, zb=1.405),
        dict(x=-3.3, hw=0.18, zt=1.68, zb=1.31),
        dict(x=-2.62, hw=0.30, zt=1.80, zb=1.17, et=2.2, eb=2.2),
        dict(x=-2.12, hw=0.56, zt=1.925, zb=0.93, et=2.6, eb=2.6),
        dict(x=-1.55, hw=0.80, zt=1.975, zb=0.62, zm=1.24, et=3.0, eb=3.2),
        dict(x=-0.6, hw=0.88, zt=2.0, zb=0.52, zm=1.22, et=3.2, eb=3.4),
        dict(x=0.6, hw=0.89, zt=2.0, zb=0.50, zm=1.20, et=3.2, eb=3.4),
        dict(x=1.4, hw=0.86, zt=1.965, zb=0.50, zm=1.18, et=3.0, eb=3.2),
        dict(x=2.0, hw=0.76, zt=1.86, zb=0.52, zm=1.12, et=2.6, eb=3.0),
        dict(x=2.45, hw=0.58, zt=1.64, zb=0.58, zm=1.05, et=2.3, eb=2.6),
        dict(x=2.72, hw=0.34, zt=1.36, zb=0.68, zm=0.98, et=2.1, eb=2.3),
        dict(x=2.84, hw=0.12, zt=1.14, zb=0.80, zm=0.96),
    ]

    def __init__(self, blades=4):
        self.blades = blades
        self.R = 5.35
        self.hub = V((0.0, 0.0, 2.93))
        self.tr = V((-6.72, 0.30, 2.02))
        self.tr_R = 0.93
        keys = ("hw", "zt", "zb", "zm", "et", "eb")
        self.ip = {}
        for k in keys:
            xs, ys = [], []
            for s in self.STATIONS:
                xs.append(s["x"])
                ys.append(s.get(k, (s["zt"] + s["zb"]) / 2) if k == "zm" else s.get(k, 2.0) if k in ("et", "eb") else s[k])
            self.ip[k] = Pchip(xs, ys)

    def section(self, x):
        return tuple(self.ip[k](x) for k in ("hw", "zt", "zb", "zm", "et", "eb"))

    def half_width(self, x, z):
        hw, zt, zb, zm, et, eb = self.section(x)
        h, e = (zt - zm, et) if z >= zm else (zm - zb, eb)
        t = clamp(abs(z - zm) / max(h, 1e-6))
        return hw * max(0.0, 1 - t ** e) ** (1 / e)

    def xs(self):
        return (cosine_spacing(-6.66, -2.8, 26, 0.3, 0.0)[:-1] + cosine_spacing(-2.8, -1.3, 22)[:-1]
                + cosine_spacing(-1.3, 1.9, 34)[:-1] + cosine_spacing(1.9, 2.84, 30, 0.0, 0.9))


def rotor_blade_sections(r0, R, chord, root_chord, twist0, twist1, droop, tip_len=0.32, tip_chord=0.17,
                         sweep=0.12, le_dir=1.0, n=36, t_root=0.17, t=0.12):
    secs = []
    for r in cosine_spacing(r0, R, n, 0.5, 0.9):
        f = (r - r0) / (R - r0)
        c = lerp(root_chord, chord, smoothstep(r0, r0 + 0.45, r))
        tip = smoothstep(R - tip_len, R, r)
        c = lerp(c, tip_chord, tip ** 1.4)
        sw = sweep * tip ** 2
        th = math.radians(lerp(twist0, twist1, f))
        thick = lerp(t_root, t, smoothstep(r0, r0 + 0.45, r))
        le_y = le_dir * (0.25 * c) - le_dir * sw
        cd = V((0.0, -le_dir * math.cos(th), -math.sin(th)))       # Profilsehne (Nase → Hinterkante)
        td = V((0.0, -le_dir * math.sin(th), math.cos(th)))        # Profildicke, senkrecht zur Sehne
        le = V((r, le_y * math.cos(th), le_y * math.sin(th) * le_dir - droop * f * f))
        secs.append(dict(le=le, chord_dir=cd, thick_dir=td, chord=c, t=thick))
    return secs


def build_heli(cfg):
    g = HeliGeo(cfg.blades)
    liv = HELI_LIVERIES.get(cfg.heli_color, HELI_LIVERIES["civil"])
    name = cfg.heli_name
    M = heli_materials(name, liv, cfg.dirt)
    asm = Assembly(name)
    asm.ao_distance = 0.6
    bevel = ("bevel", dict(width=0.004, segments=2, angle=35))
    bevel_s = ("bevel", dict(width=0.0015, segments=1, angle=40))
    shell_t = 0.025

    # ------------------------------------------------------------- Rumpf-Loft + Hülle
    fus_md = loft_stations(g.STATIONS, g.xs(), 64, V((-6.70, 0.0, 1.57)), V((2.875, 0.0, 0.965)), M["paint"])
    normals = vertex_normals(fus_md)
    shell = fus_md.to_object(f"{name}_Fuselage", smooth=60)
    shell.data.materials.append(M["interior"])
    add_modifier(shell, "solidify", thickness=shell_t, material_offset=1)
    asm.coll.objects.link(shell)
    apply_modifiers(shell)
    src = shell.data.copy()

    # Konturen (Seitenansicht x/z)
    windshield = rounded_poly_2d([(1.62, 1.33), (2.0, 1.175), (2.4, 1.065), (2.72, 1.02), (3.2, 1.0), (3.2, 2.4),
                                  (1.40, 2.4), (1.465, 1.97)], [0.10, 0.3, 0.3, 0.2, 0, 0, 0, 0.06], 5)
    chin = rounded_rect_2d(0.52, 0.30, 0.1, 5)
    chin = [V((p.x + 2.33, p.y + 0.80)) for p in chin]
    door_f = rounded_poly_2d([(0.64, 0.68), (1.73, 0.68), (1.69, 1.28), (1.535, 1.93), (0.64, 1.93)],
                             [0.05, 0.05, 0.12, 0.05, 0.05], 4)
    win_f = rounded_poly_2d([(0.74, 1.10), (1.585, 1.10), (1.60, 1.30), (1.47, 1.86), (0.74, 1.86)],
                            [0.07, 0.07, 0.1, 0.07, 0.07], 5)
    door_r = rounded_poly_2d([(-0.95, 0.68), (0.53, 0.68), (0.53, 1.935), (-0.84, 1.935), (-0.95, 1.80)],
                             [0.05, 0.05, 0.05, 0.08, 0.05], 4)
    win_r = rounded_poly_2d([(-0.83, 1.10), (0.42, 1.10), (0.42, 1.85), (-0.74, 1.85), (-0.83, 1.74)],
                            [0.07, 0.07, 0.07, 0.07, 0.07], 5)
    gap = 0.004

    # ------------------------------------------------------------- Türen
    doors = {}
    for key, outline, win, hinge_x in (("FL", door_f, win_f, (1.71, 1.62)), ("RL", door_r, win_r, (0.51, 0.51))):
        for side in (1, -1):
            dk = key[0] + ("L" if side > 0 else "R")
            y0, y1 = (0.30, 1.6) if side > 0 else (-1.6, -0.30)
            d = bpy.data.objects.new(f"{name}_Door_{dk}", src.copy())
            asm.coll.objects.link(d)
            boolean(d, [prism_y(offset_poly(outline, -gap), y0, y1, M["seal"]).to_object("_c")], "INTERSECT")
            boolean(d, [prism_y(win, y0, y1, M["seal"]).to_object("_c")], "DIFFERENCE")
            transfer_normals(d, src)
            zl, zh = 0.85, 1.75
            xl, xh = hinge_x
            p_lo = V((xl, side * g.half_width(xl, zl), zl))
            p_hi = V((xh, side * g.half_width(xh, zh), zh))
            hinge = (p_hi - p_lo).normalized() * (-1.0 if side > 0 else 1.0)    # positiv = öffnet nach außen
            asm.pivot(f"Door_{dk}", (p_lo + p_hi) / 2, hinge, orient=True, positive="Tür öffnet nach außen")
            asm.add(d, f"Door_{dk}", f"Door_{dk}")
            gl = glass_patch(fus_md, normals, offset_poly(win, 0.05), shell_t * 0.45, M["glass"],
                             (lambda y, s=side: y * s > 0.2))
            asm.add(gl, f"Glass_Door_{dk}", f"Door_{dk}", smooth=80)
            doors[dk] = d

    # ------------------------------------------------------------- Fensterausschnitte im Rumpf
    cutters = []
    for y0, y1 in ((0.035, 1.6), (-1.6, -0.035)):
        cutters.append(prism_y(windshield, y0, y1, M["seal"]).to_object("_c"))
    for y0, y1 in ((0.22, 1.6), (-1.6, -0.22)):
        cutters.append(prism_y(chin, y0, y1, M["seal"]).to_object("_c"))
    for outline in (door_f, door_r):
        for y0, y1 in ((0.30, 1.6), (-1.6, -0.30)):
            cutters.append(prism_y(offset_poly(outline, gap), y0, y1, M["seal"]).to_object("_c"))
    boolean(shell, cutters, "DIFFERENCE")
    transfer_normals(shell, src)
    asm.add(shell, "Fuselage")
    asm.add(glass_patch(fus_md, normals, offset_poly(windshield, 0.05), shell_t * 0.45, M["glass"]), "Glass_Front",
            smooth=80)
    asm.add(glass_patch(fus_md, normals, offset_poly(chin, 0.05), shell_t * 0.45, M["glass"],
                        lambda y: abs(y) > 0.15), "Glass_Chin", smooth=80)
    bpy.data.meshes.remove(src)

    # ------------------------------------------------------------- Triebwerksverkleidung, Einlässe, Abgas
    cowl = loft_stations([dict(x=-2.42, hw=0.16, zt=2.0, zb=1.76, et=2.5, eb=4.0),
                          dict(x=-2.0, hw=0.42, zt=2.22, zb=1.78, et=3.0, eb=5.0),
                          dict(x=-1.4, hw=0.55, zt=2.385, zb=1.84, et=3.2, eb=5.0),
                          dict(x=-0.4, hw=0.58, zt=2.42, zb=1.88, et=3.2, eb=5.0),
                          dict(x=0.55, hw=0.52, zt=2.30, zb=1.88, et=3.0, eb=5.0),
                          dict(x=0.88, hw=0.24, zt=2.06, zb=1.90, et=2.5, eb=4.0)],
                         cosine_spacing(-2.42, 0.88, 40, 0.5, 0.5), 48, V((-2.45, 0, 1.9)), V((0.9, 0, 1.97)), M["paint"])
    asm.add(cowl, "Cowling", smooth=50)
    det = MeshData()
    for side in (-1, 1):
        x0, z0 = -0.62, 2.16
        gm = frame_matrix(V((x0, side * 0.565, z0)), -X_AXIS * side, Z_AXIS, Y_AXIS * side)
        det.merge(extrude(rounded_rect_2d(0.62, 0.20, 0.05, 4), 0.03, gm, M["grille"]))
        for k in range(7):
            det.merge(box((lerp(-0.27, 0.27, k / 6), 0.0, 0.016), (0.012, 0.17, 0.008), M["paint"], gm))
    ex0, ex1 = V((-1.75, -0.22, 2.12)), V((-2.30, -0.36, 2.34))
    det.merge(tube([ex0, ex1], lambda s: (lerp(0.12, 0.15, s), lerp(0.10, 0.12, s)), 24, M["soot"], caps=False))
    det.merge(tube([ex1 - (ex1 - ex0).normalized() * 0.3, ex1 - (ex1 - ex0).normalized() * 0.29],
                   (0.12, 0.10), 24, M["soot"]))
    det.merge(tube([ex1 - (ex1 - ex0).normalized() * 0.01, ex1 + (ex1 - ex0).normalized() * 0.01],
                   (0.152, 0.122), 24, M["metal"], caps=False))
    asm.add(det, "CowlingDetails", mods=[bevel_s])

    # ------------------------------------------------------------- Heckausleger-Anbauten
    tail = MeshData()
    tail.merge(loft_stations([dict(x=-6.25, hw=0.035, zt=1.70, zb=1.62, et=2.5, eb=2.5),
                              dict(x=-5.9, hw=0.06, zt=1.71, zb=1.60, et=3.0, eb=3.0),
                              dict(x=-2.6, hw=0.07, zt=1.86, zb=1.75, et=3.0, eb=3.0),
                              dict(x=-2.35, hw=0.04, zt=1.90, zb=1.80, et=2.5, eb=2.5)],
                             cosine_spacing(-6.25, -2.35, 30, 0.3, 0.3), 24, "ngon", "ngon", M["paint"]))
    stab = []
    for yy in (-1.12, 0.0, 1.12):
        stab.append(dict(le=V((-5.22 - 0.05 * abs(yy), yy, 1.625)), chord_dir=V((-1, 0, 0.04)), thick_dir=Z_AXIS,
                         chord=0.44 - 0.06 * abs(yy), t=0.13))
    tail.merge(airfoil_loft(stab, 20, M["paint"]))
    for side in (-1, 1):
        ep = [dict(le=V((-5.25 - 0.10 * f, side * 1.15, lerp(1.32, 1.98, f))), chord_dir=X_AXIS * -1, thick_dir=Y_AXIS,
                    chord=lerp(0.40, 0.28, f), t=0.10) for f in (0.0, 0.5, 1.0)]
        tail.merge(airfoil_loft(ep, 16, M["paint"]))
    fin = [dict(le=V((lerp(-6.02, -6.64, f), 0.0, lerp(1.50, 2.55, f))), chord_dir=V((-1, 0, 0)), thick_dir=Y_AXIS,
                chord=lerp(0.82, 0.44, f), t=0.15) for f in (0.0, 0.25, 0.5, 0.75, 1.0)]
    fin_md = airfoil_loft(fin, 20, M["paint"])
    tail.merge(fin_md)
    vent = [dict(le=V((lerp(-6.12, -6.42, f), 0.0, lerp(1.52, 1.02, f))), chord_dir=V((-1, 0, 0)), thick_dir=Y_AXIS,
                 chord=lerp(0.55, 0.36, f), t=0.13) for f in (0.0, 0.5, 1.0)]
    tail.merge(airfoil_loft(vent, 16, M["paint"]))
    asm.add(tail, "TailSurfaces", smooth=50)

    # Kennzeichen (Seitenleitwerk) + Schriftzug (Heckausleger) als projizierte Mesh-Decals
    rng = random.Random(cfg.seed)
    reg = "D-H" + "".join(rng.choice("ABCDEFGHKLMNPRSTUVWXYZ") for _ in range(3))
    label = liv["label"] or (None if cfg.heli_color == "military" else reg)
    txt = MeshData()
    for side in (1, -1):
        fm = lambda o: frame_matrix(o, -X_AXIS * side, Z_AXIS, Y_AXIS * side)
        txt.merge(project_onto(text_md(reg, 0.11, fm(V((-6.52, side * 0.5, 1.72))), M["decal"]), fin_md,
                               -Y_AXIS * side, 0.0015))
        if label:
            size = 0.25 if len(label) <= 7 else 0.19
            txt.merge(project_onto(text_md(label, size, fm(V((-3.95, side * 0.8, 1.505))), M["decal"]), fus_md,
                                   -Y_AXIS * side, 0.0015))
    asm.add(txt, "Decals", smooth=60, detail=1)
    tg = MeshData()
    tg.merge(tube(fillet_path([(-6.30, 0, 1.06), (-6.62, 0, 0.98), (-6.95, 0, 1.08)], 0.15), 0.018, 10, M["steel"]))
    tg.merge(lathe([(0.10, 0.0), (0.105, 0.06), (0.09, 0.14), (0.065, 0.20), (0.0, 0.215)], 24,
                   V((g.tr.x, 0.05, g.tr.z)), Y_AXIS, M["paint"]))
    tg.merge(cylinder(V((g.tr.x, 0.2, g.tr.z)), V((g.tr.x, g.tr.y + 0.06, g.tr.z)), 0.03, 16, M["dark"]))
    asm.add(tg, "TailGearbox", mods=[bevel_s])

    # ------------------------------------------------------------- Hauptrotor
    asm.pivot("MainRotor", g.hub, Z_AXIS, positive="Drehrichtung des Rotors (von oben gegen den Uhrzeigersinn)",
              motion="spin")
    head = MeshData()
    head.merge(cylinder(V((0, 0, 2.30)), V((0, 0, 3.0)), 0.075, 24, M["dark"]))
    head.merge(lathe([(0.0, 0.0), (0.25, 0.0), (0.25, 0.04), (0.20, 0.06), (0.0, 0.06)], 32, V((0, 0, 2.50)), Z_AXIS,
                     M["metal"]))
    head.merge(lathe([(0.0, 0.0), (0.22, 0.0), (0.22, 0.035), (0.0, 0.035)], 32, V((0, 0, 2.575)), Z_AXIS, M["steel"]))
    head.merge(extrude(star_2d(0.13, 0.46, g.blades, 192, 0.5, 0.3), 0.05, Matrix.Translation(g.hub), M["metal"]))
    head.merge(lathe([(0.17, 0.0), (0.16, 0.05), (0.10, 0.11), (0.0, 0.13)], 32, g.hub + V((0, 0, 0.025)), Z_AXIS,
                     M["paint"]))
    blades = []
    secs = rotor_blade_sections(0.62, g.R, 0.30, 0.23, 8.0, -3.0, 0.14)
    blade_proto = airfoil_loft(secs, 20, M["blade"])
    for k in range(g.blades):
        psi = TAU * k / g.blades + math.radians(45)
        rot = Matrix.Rotation(psi, 4, "Z")
        er = rot @ X_AXIS
        et = Z_AXIS.cross(er)
        head.merge(tube([g.hub + er * 0.16, g.hub + er * 0.70], (0.06, 0.075), 20, M["steel"], exponent=3.0))
        head.merge(box(V((0.36, 0.11, -0.02)), (0.08, 0.08, 0.03), M["metal"], Matrix.Translation(g.hub) @ rot))
        sw = V((0, 0, 2.61)) + (rot @ V((0.20, 0.11, 0)))
        horn = g.hub + (rot @ V((0.36, 0.13, -0.03)))
        head.merge(cylinder(sw, horn, 0.012, 8, M["steel"]))
        head.merge(cylinder(g.hub + er * 0.30 - et * 0.10 + V((0, 0, 0.02)), g.hub + er * 0.55 - et * 0.06 + V((0, 0, 0.02)),
                            0.025, 12, M["dark"]))
        key = f"Blade_{k + 1}"
        asm.pivot(key, g.hub + er * 0.62, er, parent="MainRotor", orient=True,
                  positive="Blattanstellwinkel: Vorderkante nach oben")
        bl = blade_proto.copy().transform(Matrix.Translation(g.hub) @ rot)
        blades.append(asm.add(bl, key, key, smooth=40))
    asm.add(head, "RotorHead", "MainRotor", mods=[bevel_s])

    # ------------------------------------------------------------- Heckrotor
    asm.pivot("TailRotor", g.tr, -Y_AXIS, positive="Rotordrehung (Blattvorderkante voraus)", motion="spin")
    trm = MeshData()
    tsecs = rotor_blade_sections(0.12, g.tr_R, 0.17, 0.15, 6.0, 0.0, 0.0, 0.08, 0.13, 0.03, n=16, t_root=0.14)
    tproto = airfoil_loft(tsecs, 16, M["tblade"])
    for k in range(2):
        m = (Matrix.Translation(g.tr) @ Matrix.Rotation(math.pi * k + math.radians(30), 4, "Y")
             @ Matrix.Rotation(math.pi / 2, 4, "X"))
        trm.merge(tproto.copy().transform(m))
    trm.merge(cylinder(g.tr - Y_AXIS * 0.03, g.tr + Y_AXIS * 0.06, 0.06, 20, M["metal"]))
    trm.merge(lathe([(0.06, 0.0), (0.05, 0.03), (0.0, 0.05)], 20, g.tr + Y_AXIS * 0.06, Y_AXIS, M["metal"]))
    asm.add(trm, "TailRotor", "TailRotor", smooth=40)

    # ------------------------------------------------------------- Kufenlandegestell
    sk = MeshData()
    for side in (-1, 1):
        y = side * 1.13
        skid = fillet_path([(-1.95, y, 0.045), (1.72, y, 0.045), (2.06, y, 0.30)], [0.35], 10)
        sk.merge(tube(skid, 0.04, 16, M["skid"]))
        sk.merge(tube([(-0.6, y, 0.0), (0.6, y, 0.0)], (0.008, 0.035), 8, M["steel"], exponent=4))
        for x in (0.95, -0.95):
            step = V((x + 0.0, side * 1.02, 0.36))
            sk.merge(box(step, (0.30, 0.10, 0.02), M["dark"]))
    for x in (0.95, -0.95):
        arch = fillet_path([(x, 1.13, 0.06), (x, 1.06, 0.40), (x, 0.70, 0.53), (x, -0.70, 0.53), (x, -1.06, 0.40),
                            (x, -1.13, 0.06)], [0.25, 0.4, 0.4, 0.25], 10)
        sk.merge(tube(arch, (0.05, 0.035), 16, M["skid"], exponent=2.2))
    asm.add(sk, "Skids", mods=[bevel_s])

    # ------------------------------------------------------------- Innenraum
    it = MeshData()
    it.merge(box((0.45, 0.0, 0.615), (3.6, 1.6, 0.03), M["floor"]))
    ip_m = frame_matrix(V((0, 0, 0)), X_AXIS, Z_AXIS, -Y_AXIS)
    panel = rounded_poly_2d([(1.98, 0.64), (2.18, 0.64), (2.12, 1.18), (2.02, 1.34), (1.76, 1.36), (1.78, 1.26),
                             (1.90, 1.22)], 0.04, 3)
    it.merge(extrude(panel, 1.30, ip_m, M["panel"]))
    for yy in (-0.36, 0.0, 0.36):
        sm = frame_matrix(V((1.895, yy, 1.06)), Y_AXIS * -1, V((0.15, 0, 0.99)).normalized(), V((-0.99, 0, 0.15)))
        it.merge(box((0, 0, 0.0), (0.22, 0.16, 0.012), M["screen"], sm))
    it.merge(box((1.35, 0.0, 0.80), (0.70, 0.18, 0.35), M["panel"]))

    def seat(x, y, w=0.46, back_angle=14):
        s = MeshData()
        s.merge(box((x, y, 0.93), (0.48, w, 0.10), M["fabric"]))
        s.merge(box((x, y, 0.80), (0.40, w - 0.06, 0.16), M["dark"]))
        bm = (Matrix.Translation((x - 0.24, y, 0.98)) @ Matrix.Rotation(math.radians(-back_angle), 4, "Y"))
        s.merge(box((0.0, 0.0, 0.33), (0.10, w, 0.66), M["fabric"], bm))
        s.merge(box((0.0, 0.0, 0.76), (0.09, w * 0.55, 0.18), M["fabric"], bm))
        return s
    for y in (-0.42, 0.42):
        it.merge(seat(1.30, y))
    for y in (-0.52, 0.0, 0.52):
        it.merge(seat(-0.40, y, 0.48, 8))
    it.merge(tube(fillet_path([(1.62, -0.42, 0.63), (1.62, -0.42, 1.10), (1.55, -0.42, 1.24)], 0.08), 0.014, 10,
                  M["dark"]))
    it.merge(box((1.54, -0.42, 1.27), (0.05, 0.04, 0.09), M["panel"]))
    it.merge(tube([(1.10, -0.70, 0.74), (1.55, -0.70, 0.92)], 0.016, 10, M["dark"]))
    for y in (-0.55, -0.30, 0.30, 0.55):       # Pedale
        pm = Matrix.Translation((2.0, y, 0.72)) @ Matrix.Rotation(math.radians(-50), 4, "Y")
        it.merge(box((0, 0, 0), (0.18, 0.08, 0.02), M["dark"], pm))
    it.merge(box((1.35, 0.0, 1.97), (0.6, 0.30, 0.06), M["panel"]))
    asm.add(it, "Interior", mods=[bevel], detail=1)

    # ------------------------------------------------------------- Kleinteile: Lichter, Antennen, Drahtschneider
    sm = MeshData()
    sm.merge(lathe([(0.0, 0.0), (0.05, 0.0), (0.045, 0.03), (0.0, 0.035)], 16, V((-6.66, 0.0, 2.55)), Z_AXIS, M["nav_red"]))
    sm.merge(lathe([(0.0, 0.0), (0.06, 0.0), (0.05, 0.04), (0.0, 0.05)], 16, V((-0.9, 0.0, 2.39)), Z_AXIS, M["nav_red"]))
    for side, mt in ((1, M["nav_red"]), (-1, M["nav_green"])):
        p = V((-5.24, side * 1.165, 1.99))
        sm.merge(lathe([(0.0, 0.0), (0.025, 0.0), (0.02, 0.04), (0.0, 0.05)], 12, p, X_AXIS, mt))
    sm.merge(lathe([(0.0, 0.0), (0.04, 0.0), (0.03, 0.03), (0.0, 0.035)], 12, V((-6.67, 0, 1.57)), -X_AXIS, M["nav_white"]))
    ll = V((2.42, 0.0, 0.61))
    sm.merge(lathe([(0.0, 0.0), (0.085, 0.0), (0.085, 0.03), (0.0, 0.03)], 24, ll, V((0.3, 0, -1)), M["dark"]))
    sm.merge(lathe([(0.0, 0.0), (0.072, 0.0), (0.0, 0.004)], 24, ll + V((0.3, 0, -1)).normalized() * 0.03,
                   V((0.3, 0, -1)), M["landing"]))
    cutter = rounded_poly_2d([(1.55, 1.98), (1.75, 1.97), (1.62, 2.26), (1.58, 2.26)], 0.01, 2)
    sm.merge(extrude(cutter, 0.018, frame_matrix(V((0, 0, 0)), X_AXIS, Z_AXIS, -Y_AXIS), M["metal"]))
    lc = rounded_poly_2d([(2.0, 0.53), (2.25, 0.57), (2.10, 0.30), (2.06, 0.30)], 0.01, 2)
    sm.merge(extrude(lc, 0.018, frame_matrix(V((0, 0, 0)), X_AXIS, Z_AXIS, -Y_AXIS), M["metal"]))
    for x0, z0, h in ((-1.2, 2.37, 0.22), (-3.6, 1.68, 0.20), (0.2, 0.505, -0.22)):
        ant = [(x0, z0), (x0 + 0.12, z0), (x0 + 0.02, z0 + h), (x0 - 0.02, z0 + h)]
        if h < 0:
            ant = list(reversed(ant))
        sm.merge(extrude(ant, 0.012, frame_matrix(V((0, 0, 0)), X_AXIS, Z_AXIS, -Y_AXIS), M["dark"]))
    sm.merge(tube(fillet_path([(2.55, 0.25, 1.55), (2.75, 0.25, 1.60), (3.05, 0.25, 1.60)], 0.05), 0.01, 8, M["metal"]))
    for dk, d in doors.items():
        side = 1 if dk[1] == "L" else -1
        x = 0.78 if dk[0] == "F" else -0.62
        y = side * (g.half_width(x, 1.0) + 0.01)
        sm.merge(box((x, y, 1.0), (0.12, 0.025, 0.025), M["dark"]))
    asm.add(sm, "Details", mods=[bevel_s], detail=2)

    if cfg.heli_color == "police":
        # Wärmebildkamera (drehbar) und Suchscheinwerfer
        asm.pivot("FLIR", V((2.12, 0.0, 0.44)), Z_AXIS, positive="Kamera schwenkt nach links")
        cam = MeshData()
        cam.merge(cylinder(V((2.12, 0, 0.56)), V((2.12, 0, 0.46)), 0.05, 16, M["dark_paint"]))
        cam.merge(lathe([(0.0, -0.13), (0.07, -0.115), (0.12, -0.06), (0.13, 0.0), (0.12, 0.06), (0.09, 0.10),
                         (0.0, 0.11)], 32, V((2.12, 0, 0.34)), Z_AXIS, M["dark_paint"]))
        for yy in (-0.035, 0.035):
            cam.merge(lathe([(0.0, 0.0), (0.03, 0.0), (0.03, 0.008), (0.0, 0.009)], 20,
                            V((2.12 + 0.118, yy, 0.35)), X_AXIS, M["screen"]))
        asm.add(cam, "FLIR", "FLIR", mods=[bevel_s])
        sl = MeshData()
        c0 = V((0.95, -0.92, 0.47))
        sl.merge(cylinder(c0 + V((0, 0, 0.05)), c0 - V((0, 0, 0.06)), 0.02, 12, M["dark"]))
        sl.merge(lathe([(0.0, -0.12), (0.10, -0.12), (0.115, 0.0), (0.105, 0.06), (0.0, 0.06)], 24,
                       c0 + V((0.06, 0, -0.14)), X_AXIS, M["dark_paint"]))
        sl.merge(lathe([(0.0, 0.0), (0.098, 0.0), (0.0, 0.004)], 24, c0 + V((0.12, 0, -0.14)), X_AXIS, M["landing"]))
        asm.add(sl, "Searchlight", mods=[bevel_s], detail=1)
        asm.socket("Searchlight", c0 + V((0.13, 0, -0.14)), forward=X_AXIS)

    # ------------------------------------------------------------- Sockets & Kollision
    asm.socket("Pilot", (1.30, -0.42, 1.05))
    asm.socket("CoPilot", (1.30, 0.42, 1.05))
    for i, y in enumerate((-0.52, 0.0, 0.52)):
        asm.socket(f"Passenger_{i + 1}", (-0.40, y, 1.05))
    asm.socket("RotorHub", g.hub)
    asm.socket("Exhaust", ex1, forward=ex1 - ex0)
    asm.socket("LandingLight", ll, forward=V((0.3, 0, -1)))
    asm.socket("Winch", (0.3, -0.95, 1.95))
    skids = [o for o in asm.meshes() if o.name.endswith("Skids")]
    # Rumpf inkl. Türen und Wärmebildkamera; Rotoren drehen und bekommen bewusst keine Hülle
    body = [o for o in asm.meshes() if o not in skids and not o["pivot"].startswith(("MainRotor", "TailRotor", "Blade_"))]
    asm.hull_exempt_pivots.update(MainRotor="drehende Rotorblätter", TailRotor="drehende Rotorblätter")
    asm.collide("Cabin", body, lambda p: p.x > -2.3)
    asm.collide("TailBoom", body, lambda p: p.x < -2.0)
    asm.collide("Skid_L", skids, lambda p: p.y > 0.0, 24)
    asm.collide("Skid_R", skids, lambda p: p.y < 0.0, 24)
    return asm


# =============================================================================
# 8b) Flugzeug (einmotoriger Hochdecker, 4 Sitze, ~11 m Spannweite, 8,3 m Länge)
# =============================================================================

PLANE_LIVERIES = {
    "classic": dict(base=(0.82, 0.82, 0.80), rough=0.26, coat=0.6,
                    bands=[(0.97, 1.07, (0.015, 0.06, 0.30), -0.06, -5.25), (1.10, 1.13, (0.70, 0.45, 0.03), -0.06, -5.10)],
                    text=(0.015, 0.06, 0.30), accent=(0.015, 0.06, 0.30)),
    "trainer": dict(base=(0.82, 0.82, 0.80), rough=0.26, coat=0.6,
                    bands=[(0.95, 1.06, (0.55, 0.012, 0.01), -0.06, -5.25), (1.09, 1.11, (0.02, 0.02, 0.02), -0.06, -5.10)],
                    text=(0.02, 0.02, 0.02), accent=(0.55, 0.012, 0.01)),
    "bush": dict(base=(0.80, 0.50, 0.0), rough=0.3, coat=0.4,
                 bands=[(-1.0, 0.90, (0.02, 0.02, 0.022), -0.06), (0.94, 0.98, (0.02, 0.02, 0.022), -0.06)],
                 text=(0.02, 0.02, 0.022), accent=(0.02, 0.02, 0.022)),
    "military": dict(base=(0.055, 0.07, 0.04), rough=0.7, coat=0.0, bands=[(-1.0, 0.86, (0.04, 0.05, 0.03), -0.06)],
                     text=(0.02, 0.022, 0.02), accent=(0.03, 0.035, 0.025)),
}


def plane_materials(prefix, liv, dirt):
    P = f"M_{prefix}_"
    return {
        "paint": mat_paint(P + "Paint", liv["base"], liv["rough"], coat=liv["coat"], livery=liv["bands"],
                           dirt=dirt * 0.3, dirt_z=(0.2, 0.8), seams=PlaneGeo.SEAMS),
        # Flügel, Leitwerk, Streben, Radverkleidungen: gleicher Lack, aber ohne Rumpfstreifen
        "plain": mat_paint(P + "PaintPlain", liv["base"], liv["rough"], coat=liv["coat"], dirt=dirt * 0.3,
                           dirt_z=(0.2, 0.8), seams=[sm for sm in PlaneGeo.SEAMS if sm[0] == 1]),
        "accent": mat_paint(P + "Accent", liv["accent"], 0.3, coat=liv["coat"]),
        "decal": uv_weight(mat_paint(P + "Decal", liv["text"], 0.3, coat=liv["coat"]), 2.0),
        "interior": uv_weight(mat_plastic(P + "Interior", (0.06, 0.06, 0.065), 0.75, grain=0.15), 0.5),
        "fabric": uv_weight(mat_plastic(P + "SeatFabric", (0.05, 0.045, 0.04), 0.9, grain=0.3), 0.5),
        "floor": uv_weight(mat_rubber(P + "Floor", (0.03, 0.03, 0.03), 0.85), 0.5),
        "seal": mat_rubber(P + "Seal", (0.012, 0.012, 0.012), 0.6),
        "glass": mat_glass(P + "Glass", (0.70, 0.76, 0.78)),
        "metal": mat_metal(P + "Metal", (0.62, 0.62, 0.64), 0.3),
        "dark": mat_metal(P + "DarkMetal", (0.06, 0.06, 0.065), 0.45),
        "steel": mat_metal(P + "Steel", (0.35, 0.35, 0.36), 0.3, dirt=dirt * 0.5, dirt_z=(0.0, 0.5)),
        "chrome": mat_metal(P + "Chrome", (0.9, 0.9, 0.92), 0.05),
        "tire": mat_rubber(P + "Tire", (0.025, 0.025, 0.025), 0.8, dirt * 0.6, (0.0, 0.3)),
        "soot": mat_metal(P + "Exhaust", (0.08, 0.07, 0.065), 0.65),
        "grille": mat_metal(P + "Intake", (0.02, 0.02, 0.02), 0.6),
        "prop": mat_rotor_blade(P + "Propeller", (0.03, 0.03, 0.035), [(0.86, 0.97, (0.85, 0.62, 0.0))], 0.4,
                                axis="X"),
        "panel": uv_weight(mat_plastic(P + "Panel", (0.02, 0.02, 0.022), 0.55), 0.5),
        "screen": uv_weight(mat_emit(P + "Screen", (0.10, 0.22, 0.32), 1.5, base=(0.01, 0.01, 0.012)), 0.5),
        "nav_red": mat_emit(P + "NavRed", (1.0, 0.02, 0.01), 12.0),
        "nav_green": mat_emit(P + "NavGreen", (0.02, 1.0, 0.1), 12.0),
        "nav_white": mat_emit(P + "NavWhite", (1.0, 1.0, 1.0), 10.0),
        "landing": mat_emit(P + "LandingLight", (1.0, 0.96, 0.9), 15.0),
    }


class PlaneGeo:
    """Rumpf-Spanten, Flügel-Grundriss (Rechteck innen, Trapez außen) und Fahrwerksmaße."""

    # Kabinendach zwischen Flügelnase und -hinterkante wird in __init__ aus der Profilunterseite erzeugt
    STATIONS = [
        dict(x=-5.75, hw=0.045, zt=1.40, zb=1.25, et=2.2, eb=2.2),
        dict(x=-5.00, hw=0.11, zt=1.46, zb=1.14, et=2.6, eb=2.4),
        dict(x=-4.065, hw=0.20, zt=1.54, zb=1.02, et=3.0, eb=2.8),
        dict(x=-2.942, hw=0.33, zt=1.66, zb=0.88, et=3.4, eb=3.2),
        dict(x=-2.10, hw=0.46, zt=1.82, zb=0.76, zm=1.25, et=3.6, eb=3.4),
        dict(x=-1.55, hw=0.54, zt=1.915, zb=0.70, zm=1.25, et=4.0, eb=3.6),
        dict(x=-0.60, hw=0.575, zt=1.935, zb=0.66, zm=1.24, et=4.2, eb=3.8),
        dict(x=0.46, hw=0.575, zt=1.935, zb=0.66, zm=1.22, et=4.2, eb=3.8),
        dict(x=0.76, hw=0.565, zt=1.75, zb=0.69, zm=1.20, et=3.8, eb=3.6),
        dict(x=1.00, hw=0.545, zt=1.60, zb=0.72, zm=1.18, et=3.4, eb=3.4),
        dict(x=1.50, hw=0.49, zt=1.53, zb=0.80, zm=1.18, et=3.0, eb=3.0),
        dict(x=1.82, hw=0.40, zt=1.47, zb=0.90, zm=1.19, et=2.6, eb=2.6),
        dict(x=1.95, hw=0.33, zt=1.43, zb=0.97, zm=1.20, et=2.4, eb=2.4),
    ]
    # Panel-Fugen: Motorhaube, Rumpfsegmente, Flügelbeplankung
    SEAMS = [(0, 1.0, 0.6, 1.62), (0, 1.55, 0.7, 1.6), (0, -2.35, 0.7, 1.85), (0, -3.75, 0.85, 1.75),
             (0, -4.75, 1.0, 1.6), (1, 1.45, 1.85, 2.3), (1, -1.45, 1.85, 2.3), (1, 3.9, 1.9, 2.3),
             (1, -3.9, 1.9, 2.3)]

    def __init__(self):
        self.wing_le_root = 0.45          # Flügelnase innen (Brandschott bei x = 1,0)
        self.wing_z_root = 1.98
        self.semispan = 5.5
        self.taper_start = 2.6
        self.chord_root, self.chord_tip = 1.63, 1.13
        self.sweep_tip = 0.50              # gesamte Zuspitzung an der Vorderkante → gerade Hinterkante
        self.dihedral = math.radians(1.73)
        self.twist_root, self.twist_tip = 1.5, -1.5          # Schränkung (Washout)
        self.camber = 0.02                                    # NACA 2412
        self.prop = V((1.97, 0.0, 1.22))
        self.prop_R = 0.95
        self.main_axle = V((-0.48, 1.27, 0.215))
        self.main_r = 0.215
        self.nose_axle = V((1.22, 0.0, 0.185))
        self.nose_r = 0.185
        self.stations = list(self.STATIONS)
        self._build_ip()
        # Kabinendach exakt unter die Flügelunterseite legen (kein Spalt, kein Durchstoßen)
        le, te = self.wing_le_root, self.wing_le_root - self.chord_root
        roof = []
        for i in range(8):
            x = lerp(le - 0.004, te + 0.02, i / 7)
            hw, _, zb, zm, _, eb = self.section(x)
            zt = self.wing_point(0.0, clamp((le - x) / self.chord_root), -1.0).z - 0.003
            roof.append(dict(x=x, hw=hw, zt=zt, zb=zb, zm=zm, et=8.0, eb=eb))
        self.stations = [s for s in self.STATIONS if not (te - 0.05 < s["x"] < le + 0.05)] + roof
        self._build_ip()

    def _build_ip(self):
        self.ip = {}
        for k in ("hw", "zt", "zb", "zm", "et", "eb"):
            xs, ys = [], []
            for s in self.stations:
                xs.append(s["x"])
                ys.append(s.get(k, (s["zt"] + s["zb"]) / 2) if k == "zm" else s.get(k, 2.0) if k in ("et", "eb") else s[k])
            self.ip[k] = Pchip(xs, ys)

    def section(self, x):
        return tuple(self.ip[k](x) for k in ("hw", "zt", "zb", "zm", "et", "eb"))

    def half_width(self, x, z):
        hw, zt, zb, zm, et, eb = self.section(x)
        h, e = (zt - zm, et) if z >= zm else (zm - zb, eb)
        t = clamp(abs(z - zm) / max(h, 1e-6))
        return hw * max(0.0, 1 - t ** e) ** (1 / e)

    def xs(self):
        return (cosine_spacing(-5.75, -2.2, 26, 0.3, 0.0)[:-1] + cosine_spacing(-2.2, 0.0, 18)[:-1]
                + cosine_spacing(0.0, 1.1, 20)[:-1] + cosine_spacing(1.1, 1.95, 16, 0.0, 0.6))

    # --- Flügel
    def wing_frac(self, a):
        return clamp((a - self.taper_start) / (self.semispan - self.taper_start))

    def wing_sec(self, y, c_scale=1.0, t=0.12, le_back=0.0):
        a = abs(y)
        f = self.wing_frac(a)
        th = math.radians(lerp(self.twist_root, self.twist_tip, a / self.semispan))
        le = V((self.wing_le_root - self.sweep_tip * f - le_back, y, self.wing_z_root + a * math.tan(self.dihedral)))
        return dict(le=le, chord_dir=V((-math.cos(th), 0.0, -math.sin(th))), thick_dir=V((-math.sin(th), 0.0, math.cos(th))),
                    chord=lerp(self.chord_root, self.chord_tip, f) * c_scale, t=t)

    def wing_point(self, y, xc, side=0.0):
        """Punkt im Flügelprofil: side=+1 Oberseite, -1 Unterseite, 0 Wölbungslinie."""
        s = self.wing_sec(y)
        yc, yt = naca4(xc, s["t"], self.camber)
        return s["le"] + s["chord_dir"] * (xc * s["chord"]) + s["thick_dir"] * ((yc + side * yt) * s["chord"])


def plane_wheel(M, center, R, width, rim_r):
    """Flugzeugrad: glatter Reifen mit Rillen, Felge mit Bremsscheibe."""
    c = V(center)
    hw = width / 2
    phi0 = math.radians(112)
    a = (R - rim_r - 0.008) / (1 - math.cos(phi0))
    rc = R - a
    prof = []
    for i in range(29):
        phi = lerp(-phi0, phi0, i / 28)
        if abs(phi) < math.pi / 2:
            h = hw * math.sin(phi)
        else:                                       # Flanke zieht zum Wulst hin ein
            t = (abs(phi) - math.pi / 2) / (phi0 - math.pi / 2)
            h = math.copysign(hw - 0.3 * hw * t ** 1.5, phi)
        r = rc + a * math.cos(phi)
        for g0 in (-0.28, 0.28):                  # Laufflächenrillen
            r -= 0.004 * math.exp(-((h / hw - g0) / 0.06) ** 2)
        prof.append((r, h))
    md = lathe(prof, 48, c, Y_AXIS, M["tire"])
    rim = [(0.016, -hw * 0.75), (rim_r * 0.55, -hw * 0.75), (rim_r * 0.6, -hw * 0.55), (rim_r, -hw * 0.6),
           (rim_r + 0.008, -hw * 0.72), (rim_r + 0.008, hw * 0.72), (rim_r, hw * 0.6), (rim_r * 0.6, hw * 0.55),
           (rim_r * 0.55, hw * 0.75), (0.016, hw * 0.75)]
    md.merge(lathe(rim, 32, c, Y_AXIS, M["metal"], closed_profile=True))
    for i in range(6):
        t = TAU * i / 6
        p = c + (math.cos(t) * Z_AXIS + math.sin(t) * X_AXIS) * rim_r * 0.4
        for sd in (-1, 1):
            q = p + Y_AXIS * sd * hw * 0.75
            md.merge(cylinder(q, q + Y_AXIS * sd * 0.008, 0.006, 6, M["steel"]))
    return md


def build_plane(cfg):
    g = PlaneGeo()
    liv = PLANE_LIVERIES.get(cfg.plane_color, PLANE_LIVERIES["classic"])
    name = cfg.plane_name
    M = plane_materials(name, liv, cfg.dirt)
    asm = Assembly(name)
    asm.ao_distance = 0.5
    bevel_s = ("bevel", dict(width=0.0015, segments=1, angle=40))
    bevel = ("bevel", dict(width=0.003, segments=2, angle=35))
    shell_t = 0.02

    # ------------------------------------------------------------- Rumpf-Loft + Hülle
    fus_md = loft_stations(g.stations, g.xs(), 64, V((-5.78, 0.0, 1.33)), V((1.975, 0.0, 1.20)), M["paint"])
    normals = vertex_normals(fus_md)
    shell = fus_md.to_object(f"{name}_Fuselage", smooth=60)
    shell.data.materials.append(M["interior"])
    add_modifier(shell, "solidify", thickness=shell_t, material_offset=1)
    asm.coll.objects.link(shell)
    apply_modifiers(shell)
    src = shell.data.copy()

    windshield = rounded_poly_2d([(0.99, 1.50), (0.99, 2.4), (0.43, 2.4), (0.46, 1.92), (0.92, 1.52)],
                                 [0.0, 0.0, 0.0, 0.06, 0.08], 4)
    # Türvorderkante parallel zur Scheibe → durchgehende, ~7 cm breite A-Säule
    door = rounded_poly_2d([(-0.20, 0.78), (0.86, 0.78), (0.86, 1.50), (0.45, 1.86), (-0.20, 1.86)],
                           [0.05, 0.05, 0.10, 0.05, 0.05], 4)
    win_door = rounded_poly_2d([(-0.12, 1.24), (0.78, 1.24), (0.78, 1.49), (0.44, 1.79), (-0.12, 1.79)],
                               [0.06, 0.06, 0.08, 0.06, 0.06], 5)
    win_rear = rounded_poly_2d([(-0.32, 1.24), (-0.32, 1.79), (-1.30, 1.77), (-1.62, 1.42), (-1.45, 1.24)],
                               [0.06, 0.06, 0.12, 0.08, 0.08], 5)
    win_top = rounded_poly_2d([(-1.32, 1.86), (-1.32, 2.4), (-2.45, 2.4), (-2.45, 1.66)], [0.0, 0.0, 0.0, 0.05], 4)
    gap = 0.004

    # ------------------------------------------------------------- Türen (vorne links/rechts)
    for side in (1, -1):
        dk = "L" if side > 0 else "R"
        y0, y1 = (0.30, 1.2) if side > 0 else (-1.2, -0.30)
        d = bpy.data.objects.new(f"{name}_Door_{dk}", src.copy())
        asm.coll.objects.link(d)
        boolean(d, [prism_y(offset_poly(door, -gap), y0, y1, M["seal"]).to_object("_c")], "INTERSECT")
        boolean(d, [prism_y(win_door, y0, y1, M["seal"]).to_object("_c")], "DIFFERENCE")
        transfer_normals(d, src)
        p_lo = V((0.84, side * g.half_width(0.84, 0.90), 0.90))
        p_hi = V((0.84, side * g.half_width(0.84, 1.40), 1.40))
        hinge_ax = (p_hi - p_lo).normalized() * (-1.0 if side > 0 else 1.0)    # positiv = öffnet nach außen
        asm.pivot(f"Door_{dk}", (p_lo + p_hi) / 2, hinge_ax, orient=True, positive="Tür öffnet nach außen")
        asm.add(d, f"Door_{dk}", f"Door_{dk}")
        asm.add(glass_patch(fus_md, normals, offset_poly(win_door, 0.04), shell_t * 0.45, M["glass"],
                            (lambda y, s=side: y * s > 0.2)), f"Glass_Door_{dk}", f"Door_{dk}", smooth=80)
        hx = -0.10
        asm.add(box((hx, side * (g.half_width(hx, 1.12) + 0.007), 1.12), (0.10, 0.02, 0.022), M["metal"]),
                f"DoorHandle_{dk}", f"Door_{dk}", mods=[bevel_s], detail=3)

    # ------------------------------------------------------------- Fenster
    cutters = [prism_y(windshield, -0.8, 0.8, M["seal"]).to_object("_c"),
               prism_y(win_top, -0.34, 0.34, M["seal"]).to_object("_c")]
    for y0, y1 in ((0.30, 1.2), (-1.2, -0.30)):
        cutters.append(prism_y(offset_poly(door, gap), y0, y1, M["seal"]).to_object("_c"))
        cutters.append(prism_y(win_rear, y0, y1, M["seal"]).to_object("_c"))
    boolean(shell, cutters, "DIFFERENCE")
    transfer_normals(shell, src)
    asm.add(shell, "Fuselage")
    bpy.data.meshes.remove(src)
    glass = MeshData()
    glass.merge(glass_patch(fus_md, normals, offset_poly(windshield, 0.04), shell_t * 0.45, M["glass"]))
    glass.merge(glass_patch(fus_md, normals, offset_poly(win_top, 0.04), shell_t * 0.45, M["glass"],
                            lambda y: abs(y) < 0.36))
    glass.merge(glass_patch(fus_md, normals, offset_poly(win_rear, 0.04), shell_t * 0.45, M["glass"],
                            lambda y: abs(y) > 0.2))
    asm.add(glass, "Glass", smooth=80)

    # ------------------------------------------------------------- Motorhaube: Lufteinlässe, Auspuff
    cw = MeshData()
    for side in (1, -1):
        fm = frame_matrix(V((1.952, side * 0.215, 1.135)), Y_AXIS, Z_AXIS, X_AXIS)
        cw.merge(extrude(rounded_rect_2d(0.15, 0.085, 0.04, 4), 0.03, fm, M["grille"]))
    cw.merge(extrude(rounded_rect_2d(0.14, 0.05, 0.02, 3), 0.03, frame_matrix(V((1.90, 0.0, 0.985)), Y_AXIS, Z_AXIS,
                                                                                X_AXIS), M["grille"]))
    ex0, ex1 = V((1.40, -0.16, 0.80)), V((1.26, -0.19, 0.69))
    cw.merge(tube([ex0, ex1], 0.026, 16, M["soot"], caps=False))
    cw.merge(tube([ex1 - (ex1 - ex0).normalized() * 0.02, ex1], 0.027, 16, M["dark"], caps=False))
    asm.add(cw, "CowlDetails", mods=[bevel_s])

    # ------------------------------------------------------------- Flügel (Kasten + Klappen + Querruder)
    wst = sorted({round(0.5 * (1 - math.cos(math.pi * i / 26)), 9) for i in range(27)} | {0.70, 0.75})
    full = airfoil_ring(26, 0.12, g.camber, stations=wst)
    box_flap = airfoil_ring(22, 0.12, g.camber, 0.0, 0.70, stations=wst)
    box_ail = airfoil_ring(22, 0.12, g.camber, 0.0, 0.75, stations=wst)
    flap_ring = airfoil_ring(10, 0.12, g.camber, 0.71, 1.0)
    ail_ring = airfoil_ring(10, 0.12, g.camber, 0.755, 1.0)

    def wl(ys, ring, mat=M["plain"]):
        return airfoil_loft([g.wing_sec(y) for y in ys], mat=mat, ring=ring)

    def hinge(y, xc):
        return g.wing_point(y, xc, 0.0)
    wing = MeshData()
    wing.merge(wl([-0.60, 0.0, 0.60], full))
    for s in (1, -1):
        wing.merge(wl([s * 0.60, s * 2.6, s * 2.75], box_flap))
        wing.merge(wl([s * 2.75, s * 2.85], full))
        wing.merge(wl([s * 2.85, s * 5.05], box_ail))
        tip = [g.wing_sec(s * 5.05), g.wing_sec(s * 5.35), g.wing_sec(s * 5.42, 0.97, 0.115, 0.01),
               g.wing_sec(s * 5.47, 0.86, 0.10, 0.045), g.wing_sec(s * 5.50, 0.55, 0.075, 0.16)]
        wing.merge(airfoil_loft(tip, mat=M["plain"], ring=full))
    asm.add(wing.copy(), "Wing", smooth=40)
    wing_bvh = BVHTree.FromPolygons(wing.verts, [(f[0], f[i], f[i + 1]) for f in wing.faces for i in range(1, len(f) - 1)])

    def on_wing(p, direction):
        """Punkt entlang ``direction`` auf die Flügeloberfläche setzen (Lichter bündig einbauen)."""
        hit = wing_bvh.ray_cast(p - direction * 0.3, direction, 0.6)
        return hit[0] if hit[0] is not None else p

    # Flügelwurzel-Verkleidung: schließt den Spalt zwischen Dachkante und Flügelunterseite
    def fairing(s):
        le, te = g.wing_le_root, g.wing_le_root - g.chord_root

        def fn(u, v):
            x = lerp(le - 0.01, te + 0.01, u)
            top = g.wing_point(s * 0.60, clamp((le - x) / g.chord_root), -1.0) + V((0, 0, 0.004))
            zr = g.section(x)[1]
            bot = V((x, s * g.half_width(x, zr - 0.04), zr - 0.04))
            ctrl = V((x, s * 0.585, zr))
            k = smoothstep(0.0, 0.15, u)                       # vorne aus der Flügelnase herauswachsen
            bot, ctrl = top.lerp(bot, k), top.lerp(ctrl, k)
            return bot * (1 - v) ** 2 + ctrl * (2 * v * (1 - v)) + top * (v * v)
        return surface_grid(fn, 26, 6, M["plain"], flip=s > 0)
    asm.add(fairing(1).merge(fairing(-1)), "WingFairing", smooth=50)
    for s, key in ((1, "L"), (-1, "R")):
        for kind, (ya, yb), ring, xc in (("Flap", (0.61, 2.74), flap_ring, 0.71),
                                          ("Aileron", (2.86, 5.04), ail_ring, 0.755)):
            h0, h1 = hinge(s * ya, xc), hinge(s * yb, xc)
            pk = f"{kind}_{key}"
            ax = (h1 - h0).normalized()
            ax = -ax if ax.y > 0 else ax                     # alle Achsen nach −Y: positiv = Hinterkante runter
            asm.pivot(pk, (h0 + h1) / 2, ax, orient=True,
                      positive="Hinterkante nach unten" + (" (L/R gegensinnig ansteuern)" if kind == "Aileron" else ""))
            asm.add(wl([s * ya, s * yb], ring), pk, pk, smooth=40)    # zwei Sektionen: Scharnierlinie exakt gerade

    # Streben, Tankdeckel, Lichter, Pitot
    wd = MeshData()
    for s in (1, -1):
        a = V((-0.32, s * (g.half_width(-0.32, 0.80) - 0.01), 0.80))
        b = g.wing_point(s * 2.80, 0.27, -1.0) + V((0, 0, 0.03))
        wd.merge(tube([a, b], (0.016, 0.048), 16, M["plain"]))
        for p, d in ((a, (b - a).normalized()), (b, (a - b).normalized())):
            wd.merge(tube([p, p + d * 0.12], lambda t: (lerp(0.024, 0.017, t), lerp(0.055, 0.048, t)), 16, M["plain"]))
        cap = g.wing_point(s * 2.3, 0.28, 1.0)
        n = g.wing_sec(s * 2.3)["thick_dir"]
        wd.merge(lathe([(0.0, 0.0), (0.035, 0.0), (0.035, 0.006), (0.028, 0.012), (0.0, 0.013)], 20, cap - n * 0.004, n,
                       M["metal"]))
        tipc = on_wing(g.wing_point(s * 5.47, 0.06, 0.0) + V((0, s * 0.1, 0)), -Y_AXIS * s) - Y_AXIS * s * 0.006
        wd.merge(lathe([(0.0, 0.0), (0.028, 0.0), (0.024, 0.03), (0.0, 0.04)], 12, tipc, Y_AXIS * s,
                       M["nav_red"] if s > 0 else M["nav_green"]))
        strobe = on_wing(g.wing_point(s * 5.40, 0.85, 0.0) + V((0, s * 0.2, 0)), -Y_AXIS * s) - Y_AXIS * s * 0.006
        wd.merge(lathe([(0.0, 0.0), (0.012, 0.0), (0.01, 0.015), (0.0, 0.02)], 10, strobe, Y_AXIS * s, M["nav_white"]))
    for yy in (1.95, 2.18):          # Linse folgt der Flügelnase (Lampenausschnitt in der Vorderkante)
        wd.merge(surface_decal(lambda u, v: g.wing_point(u, 0.03 * (2 * v - 1) ** 2, 1.0 if v >= 0.5 else -1.0),
                               yy - 0.05, yy + 0.05, lambda u: 0.0, lambda u: 1.0, 6, 12, M["landing"], offset=0.0012,
                               side=1.0))
    pit = g.wing_point(3.45, 0.18, -1.0)
    wd.merge(tube(fillet_path([pit + V((0, 0, 0.01)), pit - V((0, 0, 0.09)), pit - V((-0.32, 0, 0.09))], 0.03), 0.007, 8,
                  M["chrome"]))
    wdo = asm.add(wd, "WingDetails", mods=[bevel_s])

    # ------------------------------------------------------------- Leitwerk
    def ht_sec(y):
        f = abs(y) / 1.72
        # Vorderkante so gepfeilt, dass die Höhenruder-Scharnierlinie (59 %) exakt gerade verläuft
        return dict(le=V((-4.80 - 0.59 * (1.18 - 0.82) * f, y, 1.30)), chord_dir=V((-1, 0, 0)), thick_dir=Z_AXIS,
                    chord=lerp(1.18, 0.82, f), t=0.10)

    def vt_sec(z, c_scale=1.0, le_back=0.0):
        f = (z - 1.42) / (2.62 - 1.42)
        return dict(le=V((lerp(-4.45, -5.26, f) - le_back, 0.0, z)), chord_dir=V((-1, 0, 0)), thick_dir=Y_AXIS,
                    chord=lerp(1.38, 0.74, f) * c_scale, t=0.11)
    tail = MeshData()
    tail.merge(airfoil_loft([ht_sec(y) for y in (-1.72, 0.0, 1.72)], mat=M["plain"],
                            ring=airfoil_ring(18, 0.12, 0.0, 0.0, 0.58)))
    for s in (1, -1):
        tail.merge(airfoil_loft([ht_sec(s * 1.72),
                                 dict(ht_sec(s * 1.77), chord=0.70, t=0.08, le=ht_sec(s * 1.72)["le"] + V((-0.08, s * 0.05, 0)))],
                                mat=M["plain"], ring=airfoil_ring(18, 0.12, 0.0)))
    tail.merge(airfoil_loft([vt_sec(1.38), vt_sec(2.62)], mat=M["plain"], ring=airfoil_ring(18, 0.12, 0.0, 0.0, 0.62)))
    tail.merge(airfoil_loft([vt_sec(2.62), vt_sec(2.66, 0.92, 0.03), vt_sec(2.69, 0.70, 0.12)], mat=M["plain"],
                            ring=airfoil_ring(18, 0.12, 0.0)))

    def zt(x):
        return g.section(x)[1]
    tail.merge(loft_stations([dict(x=-3.35, hw=0.012, zt=zt(-3.35) + 0.004, zb=zt(-3.35) - 0.08, et=2.5, eb=2.5),
                              dict(x=-4.10, hw=0.035, zt=zt(-4.10) + 0.13, zb=zt(-4.10) - 0.08, et=2.5, eb=2.5),
                              dict(x=-4.70, hw=0.05, zt=1.74, zb=zt(-4.70) - 0.08, et=2.5, eb=2.5)],
                             cosine_spacing(-4.70, -3.35, 18), 16, "ngon", "ngon", M["plain"]))
    asm.add(tail, "Tail", smooth=45)
    el_ring = airfoil_ring(10, 0.12, 0.0, 0.59, 1.0)
    el_hinge = V((-4.80 - 0.59 * 1.18, 0.0, 1.30))
    asm.pivot("Elevator", el_hinge, -Y_AXIS, orient=True, positive="Hinterkante nach unten")
    el = MeshData()
    for s in (1, -1):
        el.merge(airfoil_loft([ht_sec(s * 0.07), ht_sec(s * 1.715)], mat=M["plain"], ring=el_ring))
    el.merge(cylinder(el_hinge - Y_AXIS * 0.08, el_hinge + Y_AXIS * 0.08, 0.012, 10, M["steel"]))
    asm.add(el, "Elevator", "Elevator", smooth=40)
    r0, r1 = vt_sec(1.42), vt_sec(2.60)
    rh0 = r0["le"] + V((-0.63 * r0["chord"], 0, 0))
    rh1 = r1["le"] + V((-0.63 * r1["chord"], 0, 0))
    asm.pivot("Rudder", (rh0 + rh1) / 2, (rh1 - rh0).normalized(), orient=True, positive="Hinterkante nach rechts")
    asm.add(airfoil_loft([r0, r1], mat=M["plain"], ring=airfoil_ring(10, 0.12, 0.0, 0.63, 1.0)), "Rudder", "Rudder",
            smooth=40)

    # ------------------------------------------------------------- Propeller + Spinner
    asm.pivot("Propeller", g.prop, X_AXIS, positive="Drehrichtung (von hinten gesehen im Uhrzeigersinn)",
              motion="spin")
    blade_x = X_AXIS * 0.06                           # Blattebene im Spinner, vor der Motorhaube
    pr = MeshData()
    secs = []
    et = X_AXIS.cross(Z_AXIS)                         # Drehrichtung im Uhrzeigersinn (von hinten gesehen)
    for r in cosine_spacing(0.15, g.prop_R, 18, 0.3, 0.9):
        f = (r - 0.15) / (g.prop_R - 0.15)
        beta = math.atan(1.55 / (TAU * r))           # Steigung 1,55 m
        c = lerp(0.07, 0.15, smoothstep(0.0, 0.35, f))
        c = lerp(c, 0.105, smoothstep(0.35, 0.9, f))
        c *= math.sqrt(max(0.12, 1.0 - smoothstep(0.90, 1.0, f) ** 2))
        cd = -math.cos(beta) * et - math.sin(beta) * X_AXIS
        td = math.cos(beta) * X_AXIS - math.sin(beta) * et
        t = lerp(0.30, 0.08, smoothstep(0.0, 0.6, f))
        secs.append(dict(le=g.prop + blade_x + Z_AXIS * r - cd * (0.30 * c), chord_dir=cd, thick_dir=td, chord=c,
                         t=t, ring=airfoil_ring(16, t, 0.04)))
    blade = airfoil_loft(secs, mat=M["prop"])
    for k in range(2):
        rot = Matrix.Translation(g.prop) @ Matrix.Rotation(math.radians(60) + math.pi * k, 4, "X") @ Matrix.Translation(-g.prop)
        pr.merge(blade.copy().transform(rot))
        d = rot.to_3x3() @ Z_AXIS
        pr.merge(cylinder(g.prop + blade_x + d * 0.08, g.prop + blade_x + d * 0.17, 0.026, 12, M["dark"]))
    pr.merge(lathe([(0.0, -0.012), (0.17, -0.012), (0.17, 0.02), (0.165, 0.08), (0.14, 0.16), (0.09, 0.24),
                    (0.04, 0.285), (0.0, 0.30)], 40, g.prop, X_AXIS, M["accent"]))
    asm.add(pr, "Propeller", "Propeller", smooth=45)

    # ------------------------------------------------------------- Fahrwerk (Bugrad lenkbar, Federstahl-Hauptfahrwerk)
    gear = MeshData()
    for s, key in ((1, "L"), (-1, "R")):
        ax = V((g.main_axle.x, s * g.main_axle.y, g.main_axle.z))
        leg = spline([(-0.40, s * 0.30, 0.74), (-0.44, s * 0.85, 0.44), (ax.x, s * 1.16, 0.24)], 8)
        gear.merge(tube(leg, lambda t: (lerp(0.016, 0.011, t), lerp(0.042, 0.028, t)), 12, M["steel"], exponent=3.0))
        gear.merge(cylinder(V((ax.x, s * 1.12, ax.z)), V((ax.x, s * 1.36, ax.z)), 0.016, 12, M["steel"]))
        stp = leg[len(leg) // 3]
        gear.merge(box(stp + V((0.0, 0.0, 0.02)), (0.13, 0.09, 0.012), M["dark"]))
        # Radverkleidung als Tropfen: stumpfe Nase, lang auslaufendes Heck
        gear.merge(loft_stations([dict(x=ax.x + 0.40, hw=0.06, zt=0.35, zb=0.18),
                                  dict(x=ax.x + 0.30, hw=0.11, zt=0.43, zb=0.10, et=2.3, eb=2.3),
                                  dict(x=ax.x + 0.05, hw=0.125, zt=0.45, zb=0.075, et=2.3, eb=2.3),
                                  dict(x=ax.x - 0.25, hw=0.10, zt=0.43, zb=0.12),
                                  dict(x=ax.x - 0.62, hw=0.012, zt=0.40, zb=0.31)],
                                 cosine_spacing(ax.x - 0.62, ax.x + 0.40, 24, 0.5, 0.5), 32,
                                 V((ax.x - 0.63, s * 1.27, 0.355)), V((ax.x + 0.42, s * 1.27, 0.265)), M["plain"],
                                 y_center=s * 1.27))
        asm.pivot(f"WheelMain_{key}", ax, Y_AXIS, positive="Rad rollt vorwärts", motion="spin")
        asm.add(plane_wheel(M, ax, g.main_r, 0.15, 0.10), f"WheelMain_{key}", f"WheelMain_{key}", smooth=40)
    asm.add(gear, "MainGear", mods=[bevel_s])
    asm.pivot("NoseGear", V((1.235, 0.0, 0.62)), Z_AXIS, positive="Bugrad lenkt nach links")
    asm.pivot("WheelNose", g.nose_axle, Y_AXIS, parent="NoseGear", positive="Rad rollt vorwärts", motion="spin")
    ng = MeshData()
    n = g.nose_axle
    ng.merge(cylinder(V((1.245, 0, 0.86)), V((1.235, 0, 0.50)), 0.034, 16, M["steel"]))
    ng.merge(cylinder(V((1.235, 0, 0.52)), V((1.228, 0, 0.42)), 0.024, 16, M["chrome"]))
    ng.merge(box(V((1.226, 0, 0.42)), (0.08, 0.21, 0.04), M["steel"]))           # Gabelbrücke über der Verkleidung
    for sd in (1, -1):
        ng.merge(cylinder(V((1.224, sd * 0.088, 0.42)), V((1.22, sd * 0.088, n.z)), 0.013, 10, M["steel"]))
    ng.merge(tube([V((1.27, 0, 0.50)), V((1.30, 0, 0.47)), V((1.255, 0, 0.44))], (0.008, 0.02), 8, M["steel"], exponent=4))
    ng.merge(loft_stations([dict(x=n.x + 0.34, hw=0.05, zt=0.31, zb=0.155),
                            dict(x=n.x + 0.255, hw=0.095, zt=0.37, zb=0.085, et=2.3, eb=2.3),
                            dict(x=n.x + 0.04, hw=0.11, zt=0.39, zb=0.065, et=2.3, eb=2.3),
                            dict(x=n.x - 0.21, hw=0.09, zt=0.37, zb=0.10, et=2.3, eb=2.3),
                            dict(x=n.x - 0.53, hw=0.012, zt=0.345, zb=0.265)],
                           cosine_spacing(n.x - 0.53, n.x + 0.34, 20, 0.5, 0.5), 32,
                           V((n.x - 0.54, 0, 0.305)), V((n.x + 0.355, 0, 0.23)), M["plain"]))
    asm.add(ng, "NoseGear", "NoseGear", mods=[bevel_s])
    asm.add(plane_wheel(M, g.nose_axle, g.nose_r, 0.125, 0.085), "WheelNose", "WheelNose", smooth=40)

    # ------------------------------------------------------------- Innenraum
    it = MeshData()
    it.merge(box((-0.33, 0.0, 0.76), (1.9, 0.80, 0.03), M["floor"]))
    panel = rounded_poly_2d([(0.80, 0.88), (0.98, 0.88), (0.98, 1.42), (0.90, 1.565), (0.68, 1.565), (0.68, 1.45),
                             (0.78, 1.35)], 0.03, 3)
    it.merge(extrude(panel, 0.84, frame_matrix(V((0, 0, 0)), X_AXIS, Z_AXIS, -Y_AXIS), M["panel"]))
    for yy in (0.26, -0.26):
        sm = frame_matrix(V((0.775, yy, 1.16)), Y_AXIS * -1, V((0.1, 0, 0.995)).normalized(), V((-0.995, 0, 0.1)))
        it.merge(box((0, 0, 0), (0.26, 0.17, 0.012), M["screen"], sm))
        col0 = V((0.78, yy, 1.06))
        it.merge(cylinder(col0, col0 - X_AXIS * 0.17, 0.012, 10, M["chrome"]))
        horn = fillet_path([(0.61, yy + 0.13, 1.17), (0.61, yy + 0.13, 1.08), (0.61, yy - 0.13, 1.08),
                            (0.61, yy - 0.13, 1.17)], 0.04)
        it.merge(tube(horn, 0.014, 10, M["panel"]))
    it.merge(box((0.70, 0.0, 0.98), (0.22, 0.16, 0.20), M["panel"]))

    def seat(x, y):
        sm_ = MeshData()
        sm_.merge(box((x, y, 0.98), (0.48, 0.44, 0.10), M["fabric"]))
        sm_.merge(box((x, y, 0.855), (0.40, 0.38, 0.16), M["dark"]))
        bm = Matrix.Translation((x - 0.24, y, 1.02)) @ Matrix.Rotation(math.radians(-12), 4, "Y")
        sm_.merge(box((0.0, 0.0, 0.32), (0.10, 0.44, 0.64), M["fabric"], bm))
        sm_.merge(box((0.0, 0.0, 0.74), (0.09, 0.26, 0.16), M["fabric"], bm))
        return sm_
    for yy in (0.27, -0.27):
        it.merge(seat(0.05, yy))
        it.merge(seat(-0.95, yy))
    asm.add(it, "Interior", mods=[bevel], detail=1)

    # ------------------------------------------------------------- Kleinteile: Antennen, Lichter, Kennzeichen
    sm = MeshData()
    for x0, h in ((-2.95, 0.20), (-3.45, 0.16)):
        z0 = zt(x0)
        sm.merge(extrude([(x0, z0 - 0.01), (x0 + 0.11, z0 - 0.01), (x0 + 0.02, z0 + h), (x0 - 0.02, z0 + h)], 0.01,
                         frame_matrix(V((0, 0, 0)), X_AXIS, Z_AXIS, -Y_AXIS), M["dark"]))
    whip = cylinder(V((-3.9, 0.0, zt(-3.9) - 0.01)), V((-3.98, 0.0, zt(-3.9) + 0.42)), 0.004, 6, M["dark"])
    belly = g.section(-1.0)[2]
    sm.merge(extrude(list(reversed([(-1.0, belly + 0.01), (-0.89, belly + 0.01), (-0.98, belly - 0.16),
                                    (-1.02, belly - 0.16)])), 0.01, frame_matrix(V((0, 0, 0)), X_AXIS, Z_AXIS, -Y_AXIS),
                     M["dark"]))
    details = asm.add(sm, "Details", mods=[bevel_s], detail=2)
    antenna = asm.add(whip, "Antenna", detail=2, smooth=60)        # dünn → ohne Bevel (sonst degenerierte Faces)
    lights = MeshData()          # Leuchten bleiben in allen LODs sichtbar (wie die Positionslichter am Flügel)
    lights.merge(lathe([(0.0, 0.0), (0.04, 0.0), (0.035, 0.03), (0.0, 0.035)], 16, vt_sec(2.69)["le"] + V((-0.18, 0, 0)),
                       Z_AXIS, M["nav_red"]))
    lights.merge(lathe([(0.0, 0.0), (0.03, 0.0), (0.025, 0.025), (0.0, 0.03)], 12, V((-5.785, 0, 1.33)), -X_AXIS,
                       M["nav_white"]))
    asm.add(lights, "TailLights", mods=[bevel_s])
    rng = random.Random(cfg.seed)
    reg = "D-E" + "".join(rng.choice("ABCDEFGHKLMNPRSTUVWXYZ") for _ in range(3))
    txt = MeshData()
    for side in (1, -1):
        # parallel zu den schrägen Zierstreifen, oberhalb davon
        fm = frame_matrix(V((-2.75, side * 0.8, 1.42)), V((-side, 0.0, 0.06 * side)).normalized(),
                          V((0.06, 0.0, 1.0)).normalized(), Y_AXIS * side)
        txt.merge(project_onto(text_md(reg, 0.26, fm, M["decal"], bold=0.035), fus_md, -Y_AXIS * side, 0.0015,
                               max_edge=0.06))
    asm.add(txt, "Decals", smooth=60, detail=1)

    # ------------------------------------------------------------- Sockets & Kollision
    asm.socket("Pilot", (0.05, 0.27, 1.10))
    asm.socket("CoPilot", (0.05, -0.27, 1.10))
    asm.socket("Passenger_1", (-0.95, 0.27, 1.10))
    asm.socket("Passenger_2", (-0.95, -0.27, 1.10))
    asm.socket("PropHub", g.prop, forward=X_AXIS)
    asm.socket("Exhaust", ex1, forward=ex1 - ex0)
    asm.socket("LandingLight", g.wing_sec(2.06)["le"], forward=X_AXIS)
    asm.socket("TowBar", g.nose_axle, forward=X_AXIS)
    for s, key in ((1, "L"), (-1, "R")):
        asm.socket(f"TieDown_{key}", g.wing_point(s * 3.8, 0.3, -1.0), forward=-Z_AXIS)
    allm = asm.meshes()

    def cabin(p):
        blade = p.x > 1.98 and p.y * p.y + (p.z - g.prop.z) ** 2 > 0.17 ** 2   # Propellerblätter (feste Stellung) weglassen
        return p.x > -1.8 and abs(p.y) < 0.6 and 0.6 < p.z < 2.3 and not blade
    asm.collide("Cabin", allm, cabin, 64)
    asm.collide("Tailcone", allm, lambda p: -6.05 < p.x < -1.6 and abs(p.y) < 0.6 and p.z < 1.6)
    body = [o for o in allm if o not in (details, antenna)]                      # Antennen nicht einhüllen
    asm.hull_exempt.append(([details, antenna], "dünne Antennen"))
    asm.hull_exempt_pivots["Propeller"] = "drehender Propeller"
    asm.collide("UpperAft", body, lambda p: -3.4 < p.x < -1.6 and abs(p.y) < 0.6 and p.z >= 1.5, 32)
    asm.collide("DorsalFin", body, lambda p: -4.75 < p.x < -3.3 and p.z >= 1.5 and abs(p.y) < 0.07, 24)
    asm.collide("Wing_L", allm, lambda p: p.y > 0.6 and p.z > 1.85)
    asm.collide("Wing_R", allm, lambda p: p.y < -0.6 and p.z > 1.85)
    asm.collide("Fin", allm, lambda p: p.x < -4.35 and p.z >= 1.45 and abs(p.y) < 0.12, 32)
    asm.collide("Stab_L", allm, lambda p: p.x < -4.6 and p.y > 0.12 and 1.15 < p.z < 1.45, 32)
    asm.collide("Stab_R", allm, lambda p: p.x < -4.6 and p.y < -0.12 and 1.15 < p.z < 1.45, 32)
    for s, key in ((1, "L"), (-1, "R")):
        asm.collide(f"MainGear_{key}", allm, lambda p, s=s: -1.15 < p.x < 0.1 and p.z < 0.66 and s * p.y > 0.3, 32)
        asm.collide(f"Strut_{key}", [wdo], lambda p, s=s: 0.4 < s * p.y < 2.9 and p.z < 2.08 and -0.5 < p.x < 0.15, 24)
    asm.collide("NoseGear", allm, lambda p: 0.65 < p.x < 1.6 and p.z < 0.72 and abs(p.y) < 0.2, 24)
    return asm


# =============================================================================
# 8c) Geländewagen (5-Türer, Leiterrahmen, Starrachsen, Reifen 265/75 R16)
# =============================================================================

OFFROAD_LIVERIES = {
    "sand": dict(base=(0.50, 0.40, 0.25), rough=0.36, coat=0.35, roof=None, rim=(0.03, 0.03, 0.032)),
    "olive": dict(base=(0.11, 0.13, 0.065), rough=0.62, coat=0.0, roof=None, rim=(0.09, 0.10, 0.06)),
    "white": dict(base=(0.80, 0.80, 0.78), rough=0.28, coat=0.5, roof=(0.025, 0.025, 0.027), rim=(0.025, 0.025, 0.027)),
    "red": dict(base=(0.40, 0.02, 0.015), rough=0.26, coat=0.6, roof=(0.80, 0.80, 0.78), rim=(0.78, 0.78, 0.76)),
}


def offroad_materials(prefix, liv, dirt):
    P = f"M_{prefix}_"
    roof = [(OffroadGeo.Z_ROOF - 0.07, 9.0, liv["roof"])] if liv["roof"] else None     # Zweifarb-Dach
    return {
        "paint": mat_paint(P + "Paint", liv["base"], liv["rough"], coat=liv["coat"], livery=roof, dirt=dirt,
                           dirt_z=(0.15, 1.0)),
        "black": mat_plastic(P + "PlasticBlack", (0.02, 0.02, 0.022), 0.62, dirt * 0.9, (0.1, 0.9), grain=0.22),
        "bumper": mat_metal(P + "BumperSteel", (0.03, 0.03, 0.032), 0.55, dirt=dirt * 0.9, dirt_z=(0.1, 0.8)),
        "chassis": mat_metal(P + "Chassis", (0.035, 0.035, 0.037), 0.6, dirt=dirt * 1.2, dirt_z=(0.0, 0.9)),
        "steel": mat_metal(P + "Steel", (0.30, 0.30, 0.31), 0.35, dirt=dirt * 0.8, dirt_z=(0.0, 0.7)),
        "alu": mat_metal(P + "Aluminium", (0.72, 0.72, 0.74), 0.3, brushed=True),
        "chrome": mat_metal(P + "Chrome", (0.9, 0.9, 0.92), 0.05),
        "rim": mat_paint(P + "Rim", liv["rim"], 0.35, coat=0.2, dirt=dirt * 0.9, dirt_z=(0.0, 0.8)),
        "tire": mat_rubber(P + "Tire", (0.024, 0.024, 0.024), 0.86, dirt * 1.1, (0.0, 0.85)),
        "rubber": mat_rubber(P + "Rubber", (0.015, 0.015, 0.015), 0.7),
        "glass": mat_glass(P + "Glass", (0.55, 0.60, 0.62)),
        "lens": mat_glass(P + "Lens", (0.95, 0.95, 0.95)),
        "lamp": mat_emit(P + "HeadLight", (1.0, 0.95, 0.85), 6.0),
        "tail": mat_emit(P + "TailLight", (1.0, 0.02, 0.01), 4.0, base=(0.5, 0.01, 0.01)),
        "amber": mat_emit(P + "Indicator", (1.0, 0.42, 0.02), 3.0, base=(0.6, 0.28, 0.02)),
        "reverse": mat_paint(P + "ReverseLens", (0.85, 0.85, 0.85), 0.1, coat=1.0),
        "interior": uv_weight(mat_plastic(P + "Interior", (0.05, 0.05, 0.055), 0.75, grain=0.15), 0.5),
        "fabric": uv_weight(mat_plastic(P + "SeatFabric", (0.06, 0.055, 0.05), 0.9, grain=0.3), 0.5),
        "floor": uv_weight(mat_rubber(P + "Floor", (0.03, 0.03, 0.03), 0.85), 0.5),
        "panel": uv_weight(mat_plastic(P + "Dash", (0.025, 0.025, 0.028), 0.55), 0.5),
        "screen": uv_weight(mat_emit(P + "Screen", (0.10, 0.22, 0.32), 1.5, base=(0.01, 0.01, 0.012)), 0.5),
        "plate": mat_paint(P + "Plate", (0.82, 0.82, 0.80), 0.35),
        "decal": uv_weight(mat_paint(P + "Decal", (0.02, 0.02, 0.022), 0.3), 2.0),
        "soot": mat_metal(P + "Exhaust", (0.08, 0.07, 0.065), 0.65),
        "mirror": mat_metal(P + "MirrorGlass", (0.85, 0.87, 0.9), 0.02),
        "engine": mat_metal(P + "EngineCast", (0.10, 0.10, 0.105), 0.55, dirt=dirt * 0.5, dirt_z=(0.55, 1.0)),
        "valve": mat_paint(P + "ValveCover", (0.32, 0.02, 0.015), 0.45, coat=0.2),
    }


def rrect_ring(cx, hx, hy, rc, nx, ny, cs, z):
    """Abgerundetes Rechteck auf Höhe z (CCW von oben, Start vorne rechts); gerade Seiten vorn/hinten in
    ``nx``, seitlich in ``ny`` Stücke geteilt, Ecken mit ``cs`` Segmenten – gleiche Punktzahl auf jeder Höhe."""
    rc = min(rc, hx - 1e-4, hy - 1e-4)
    corners = [(cx + hx - rc, hy - rc, 0.0), (cx - hx + rc, hy - rc, 90.0), (cx - hx + rc, -hy + rc, 180.0),
               (cx + hx - rc, -hy + rc, 270.0)]
    sides = [((cx + hx, -hy + rc), (cx + hx, hy - rc), nx), ((cx + hx - rc, hy), (cx - hx + rc, hy), ny),
             ((cx - hx, hy - rc), (cx - hx, -hy + rc), nx), ((cx - hx + rc, -hy), (cx + hx - rc, -hy), ny)]
    pts = []
    for (a, b, n), (ccx, ccy, a0) in zip(sides, corners):
        pts += [V((lerp(a[0], b[0], i / n), lerp(a[1], b[1], i / n), z)) for i in range(n)]
        pts += [V((ccx + rc * math.cos(math.radians(a0 + 90.0 * s / cs)),
                   ccy + rc * math.sin(math.radians(a0 + 90.0 * s / cs)), z)) for s in range(cs)]
    return pts


class OffroadGeo:
    """Maße eines generischen 5-türigen Geländewagens: 4,6 m Karosserie (≈ 5,0 m mit Seilwinde und
    Reserverad), Breite 1,86 m, Höhe 1,98 m, Radstand 2,76 m, Spur 1,60 m, Reifen 265/75 R16 (Ø 0,80 m),
    Bodenfreiheit ≈ 0,23 m. Gürtellinie auf 1,30 m: der Unterbau ist höher als der Glasaufbau."""

    Z_ROOF = 1.98
    # Unterbau (Loft entlang X): kantige Superellipsen-Spanten, Ecken in Drauf- und Seitenansicht gerundet
    LOWER = [
        dict(x=-2.22, hw=0.892, zt=1.29, zb=0.665, et=14.0, eb=5.0),
        dict(x=-2.205, hw=0.919, zt=1.298, zb=0.645, et=14.0, eb=5.0),
        dict(x=-2.17, hw=0.929, zt=1.30, zb=0.62, et=14.0, eb=5.0),
        dict(x=-1.90, hw=0.93, zt=1.30, zb=0.56, et=14.0, eb=5.0),
        dict(x=-0.95, hw=0.93, zt=1.30, zb=0.50, et=14.0, eb=5.0),
        dict(x=0.75, hw=0.93, zt=1.30, zb=0.50, et=14.0, eb=5.0),
        dict(x=1.60, hw=0.93, zt=1.275, zb=0.56, et=14.0, eb=5.0),
        dict(x=2.15, hw=0.929, zt=1.255, zb=0.64, et=14.0, eb=5.0),
        dict(x=2.20, hw=0.918, zt=1.245, zb=0.665, et=14.0, eb=5.0),
        dict(x=2.22, hw=0.89, zt=1.235, zb=0.69, et=14.0, eb=5.0),
    ]

    def __init__(self):
        self.fa, self.ra = V((1.38, 0.0, 0.40)), V((-1.38, 0.0, 0.40))       # Achsmitten (Radmitte auf 0,40 m)
        self.track = 1.60
        self.x_front, self.x_rear = 2.22, -2.22
        self.hw = 0.93                                  # halbe Karosseriebreite unter der Gürtellinie
        self.z_belt, self.z_roof = 1.30, self.Z_ROOF
        self.x_ws0, self.x_ws1 = 0.74, 0.44             # Windschutzscheibe: Unterkante / Dachkante (≈ 26°)
        self.hw_sh, self.hw_roof = 0.895, 0.83          # halbe Kabinenbreite an Gürtellinie / Dach (Tumblehome)
        self.r_roof, self.r_corner = 0.055, 0.06
        self.arch_r = 0.52                              # Radlauf um die Radmitte (Verschränkung ±6° frei)
        self.shell_t = 0.02

    def wheel(self, key):
        """Radmitte für FL/FR/RL/RR."""
        a = self.fa if key[0] == "F" else self.ra
        return V((a.x, (1.0 if key[1] == "L" else -1.0) * self.track / 2, a.z))

    def t_cab(self, z):
        return clamp((z - self.z_belt) / (self.z_roof - self.r_roof - self.z_belt))

    def ws_x(self, z):
        """X der Windschutzscheibe (Außenhaut) auf Höhe z."""
        return lerp(self.x_ws0, self.x_ws1, self.t_cab(z))

    def cab_hw(self, z):
        return lerp(self.hw_sh, self.hw_roof, self.t_cab(z))

    def cab_xr(self, z):
        """Kabinen-Heck (Heckscheibe leicht nach vorn geneigt, unten 2 mm innerhalb des Unterbaus)."""
        return self.x_rear + 0.002 + 0.03 * self.t_cab(z)

    def lower_xs(self):
        ends = [0.0, 0.005, 0.015, 0.03, 0.05, 0.08, 0.12]
        mid = [lerp(-2.0, 2.0, i / 40) for i in range(41)]
        return [self.x_rear + d for d in ends] + mid + [self.x_front - d for d in reversed(ends)]

    def _cab_ring(self, z, d=0.0, z_ref=None):
        zr = z if z_ref is None else z_ref
        hw, xf, xr = self.cab_hw(zr) - d, self.ws_x(zr) - d, self.cab_xr(zr) + d
        return rrect_ring((xf + xr) / 2, (xf - xr) / 2, hw, self.r_corner, 16, 30, 4, z)

    def cabin_md(self, mat):
        """Kabine als Loft waagerechter Schnitte: Scheibenneigung, Tumblehome und gerundete Dachkante."""
        top = self.z_roof - self.r_roof
        zs = [self.z_belt - 0.07, self.z_belt - 0.02] + [lerp(self.z_belt + 0.02, top, i / 6) for i in range(7)]
        rings = [self._cab_ring(z) for z in zs]
        for k in range(1, 6):
            phi = math.pi / 2 * k / 5
            rings.append(self._cab_ring(top + self.r_roof * math.sin(phi), self.r_roof * (1 - math.cos(phi)), top))
        crown = V(((self.ws_x(top) + self.cab_xr(top)) / 2, 0.0, self.z_roof + 0.012))     # leicht gewölbtes Dach
        return grid_rings(rings, True, "ngon", crown, mat)


def offroad_wheel(M):
    """Rad 265/75 R16: Karkasse mit grobstolligem All-Terrain-Profil, Stahlfelge mit acht Lüftungslöchern,
    Bremsscheibe. Achse +Y, Außenseite +Y, Radmitte im Ursprung. Gibt dict(Name → MeshData) zurück."""
    # halbe Kontur (r, h): Laufflächenmitte → Schulter → Flanke → Wulst
    half = [(0.3885, 0.0), (0.3882, 0.03), (0.3874, 0.06), (0.386, 0.088), (0.383, 0.104), (0.377, 0.117),
            (0.367, 0.126), (0.350, 0.1315), (0.32, 0.1335), (0.285, 0.131), (0.252, 0.123), (0.228, 0.113),
            (0.214, 0.106)]
    tire = lathe([(r, -h) for r, h in reversed(half[1:])] + half, 72, V((0, 0, 0)), Y_AXIS, M["tire"])
    cum = [0.0]
    for a, b in zip(half, half[1:]):
        cum.append(cum[-1] + math.dist(a, b))

    def at(s):
        """(r, h, dr/ds, dh/ds) bei Bogenlänge s ab Laufflächenmitte."""
        s = clamp(s, 0.0, cum[-1] - 1e-6)
        i = bisect.bisect_right(cum, s) - 1
        (r0, h0), (r1, h1) = half[i], half[i + 1]
        seg = cum[i + 1] - cum[i]
        t = (s - cum[i]) / seg
        return lerp(r0, r1, t), lerp(h0, h1, t), (r1 - r0) / seg, (h1 - h0) / seg

    tread = MeshData()

    def block(s, side, theta, L, W, H, skew=0.0):
        r, h, dr, dh = at(s)
        rad = V((math.sin(theta), 0.0, math.cos(theta)))
        tan = V((math.cos(theta), 0.0, -math.sin(theta)))
        p = rad * r + Y_AXIS * (side * h)
        n = (rad * dh - Y_AXIS * (side * dr)).normalized()
        if skew:
            tan = Quaternion(n, skew * side) @ tan
        lat = n.cross(tan).normalized()
        tan = lat.cross(n).normalized()
        corners = ((-1, -1), (1, -1), (1, 1), (-1, 1))
        base = [p - n * 0.004 + tan * (sx * L / 2) + lat * (sy * W / 2) for sx, sy in corners]
        top = [p + n * H + tan * (sx * L * 0.42) + lat * (sy * W * 0.42) for sx, sy in corners]
        tread.add(base + top, [(4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)], None, M["tire"])

    pitches = 30
    hp = TAU / pitches / 2
    for k in range(pitches):
        th = TAU * k / pitches
        for side in (1, -1):
            off = 0.0 if side > 0 else hp                          # Seiten um eine halbe Teilung versetzt
            block(0.024, side, th + off, 0.060, 0.044, 0.0135, 0.25)          # Mittelblöcke, schräg
            block(0.068, side, th + off + hp, 0.062, 0.040, 0.0135, -0.15)    # Zwischenblöcke
            block(0.108, side, th + off, 0.055, 0.042, 0.013)                 # Schulterblöcke
            if k % 2 == 0:
                block(0.138, side, th + off + hp, 0.045, 0.030, 0.007)        # Flankenstollen ("Sidebiter")

    rim_prof = [(0.214, -0.112), (0.214, -0.100), (0.205, -0.097), (0.200, -0.085), (0.196, -0.030), (0.190, -0.020),
                (0.190, 0.020), (0.196, 0.030), (0.200, 0.085), (0.205, 0.097), (0.214, 0.100), (0.214, 0.112),
                (0.206, 0.112), (0.196, 0.104), (0.186, 0.090), (0.183, 0.0), (0.186, -0.090), (0.196, -0.104),
                (0.206, -0.112)]
    rim = lathe(rim_prof, 64, V((0, 0, 0)), Y_AXIS, M["rim"], closed_profile=True)
    # Felgenschüssel mit acht Lüftungslöchern, Radkappe, sechs Radmuttern (Lochkreis 139,7 mm)
    rim.merge(slotted_ring(circle_2d(0.185, 64), 0.058, 0.105, 0.145, 8, (2, 3, 4), 0.008,
                           disc_matrix(V((0, 0, 0)), 0.035), M["rim"]))
    rim.merge(lathe([(0.060, 0.039), (0.058, 0.052), (0.046, 0.066), (0.026, 0.074), (0.0, 0.077)], 32,
                    V((0, 0, 0)), Y_AXIS, M["chrome"]))
    for i in range(6):
        p = V((math.sin(TAU * i / 6), 0.0, math.cos(TAU * i / 6))) * 0.0698
        rim.merge(cylinder(p + Y_AXIS * 0.039, p + Y_AXIS * 0.056, 0.0115, 6, M["steel"]))
    brake = lathe([(0.075, -0.048), (0.158, -0.048), (0.158, -0.022), (0.075, -0.022)], 48, V((0, 0, 0)), Y_AXIS,
                  M["steel"], closed_profile=True)
    brake.merge(cylinder(V((0, -0.048, 0)), V((0, 0.032, 0)), 0.075, 24, M["steel"]))
    return {"tire": tire, "tread": tread, "rim": rim, "brake": brake}


def offroad_engine_bay(g, M):
    """Motorraum (sichtbar bei offener Haube): längs eingebauter Reihen-Sechszylinder-Turbodiesel mit
    Riementrieb, Kühler mit Visco-Lüfter, Luftfilterkasten am Schnorchel, Batterie, Bremskraftverstärker,
    Ausgleichsbehälter, Schläuche, Spritzwand und Innenkotflügel. Gibt (Hauptteile, Kleinteile) zurück."""
    eng, small = MeshData(), MeshData()
    mot, mot_small = MeshData(), MeshData()                                    # Motorteile, um dz angehoben
    dz = 0.10                                                                  # füllt den Motorraum unter der Haube
    xs = [1.0 + 0.7 * (k + 0.5) / 6 for k in range(6)]                       # Zylindermitten
    eng.merge(box((0.76, 0.0, 0.905), (0.02, 1.78, 0.73), M["interior"]))      # Spritzwand
    z_shelf = g.fa.z + g.arch_r + 0.005                                        # Innenkotflügel über den Radhäusern
    for side in (1, -1):
        eng.merge(box((1.39, side * 0.6575, z_shelf - 0.0075), (0.98, 0.425, 0.015), M["interior"]))
        eng.merge(box((1.39, side * 0.45, (0.555 + z_shelf) / 2), (0.98, 0.01, z_shelf - 0.555), M["interior"]))
    # Block mit eingezogener Schürze, Zylinderkopf, Ventildeckel, Steuergehäuse, Kupplungsglocke
    mot.merge(prism_x([(-0.12, 0.66), (0.12, 0.66), (0.15, 0.74), (0.15, 0.95), (-0.15, 0.95), (-0.15, 0.74)],
                      1.0, 1.70, M["engine"]))
    mot.merge(box((1.35, 0.0, 0.98), (0.70, 0.31, 0.06), M["engine"]))
    cover = rounded_poly_2d([(-0.125, 1.01), (0.125, 1.01), (0.125, 1.035), (0.10, 1.058), (-0.10, 1.058),
                             (-0.125, 1.035)], [0.0, 0.0, 0.01, 0.012, 0.012, 0.01], 3)
    mot.merge(prism_x(cover, 1.02, 1.68, M["valve"]))
    mot.merge(text_md("TURBO DIESEL", 0.032, frame_matrix(V((1.35, 0.0, 1.0595)), -X_AXIS, -Y_AXIS, Z_AXIS),
                      M["alu"], bold=0.02))
    mot.merge(box((1.7225, 0.0, 0.835), (0.045, 0.26, 0.33), M["engine"]))
    mot.merge(lathe([(0.12, 0.0), (0.155, 0.16), (0.17, 0.22)], 24, V((0.78, 0.0, 0.74)), X_AXIS, M["engine"]))
    for side in (1, -1):                                                        # Motorlager auf dem Rahmen
        eng.merge(box((1.42, side * 0.26, 0.685 + dz / 2), (0.10, 0.22, 0.05 + dz), M["chassis"]))
    # Ansaugseite links: Sammler mit sechs Saugrohren; Abgasseite rechts: Krümmer → Turbolader → Hosenrohr
    mot.merge(cylinder(V((1.03, 0.25, 0.975)), V((1.67, 0.25, 0.975)), 0.045, 16, M["alu"]))
    for x in xs:
        mot.merge(cylinder(V((x, 0.21, 0.975)), V((x, 0.15, 0.98)), 0.022, 10, M["alu"]))
        mot.merge(cylinder(V((x, -0.15, 0.972)), V((x, -0.21, 0.935)), 0.02, 10, M["soot"]))
    mot.merge(cylinder(V((1.10, -0.215, 0.925)), V((1.66, -0.215, 0.925)), 0.032, 14, M["soot"]))
    mot.merge(cylinder(V((1.02, -0.27, 0.905)), V((1.10, -0.27, 0.905)), 0.07, 20, M["soot"]))       # Turbine
    mot.merge(cylinder(V((0.99, -0.27, 0.905)), V((1.02, -0.27, 0.905)), 0.035, 12, M["steel"]))
    mot.merge(cylinder(V((0.92, -0.27, 0.905)), V((0.99, -0.27, 0.905)), 0.075, 20, M["alu"]))       # Verdichter
    eng.merge(tube(spline([(1.07, -0.27, 0.86 + dz), (1.07, -0.27, 0.76 + dz * 0.6), (1.055, -0.262, 0.64),
                           (1.05, -0.26, 0.55)], 4), 0.03, 12, M["soot"]))                         # Hosenrohr
    mot.merge(tube(spline([(0.955, -0.27, 0.95), (0.955, -0.25, 1.03), (0.95, -0.14, 1.062), (0.95, 0.12, 1.062),
                           (0.97, 0.22, 1.03), (1.01, 0.25, 0.985), (1.05, 0.25, 0.975)], 4), 0.03, 12, M["alu"]))
    # Luftfilterkasten über dem rechten Radhaus, Anschluss an den Schnorchel, Ansaugschlauch zum Verdichter
    eng.merge(box((1.12, -0.665, z_shelf + 0.12), (0.40, 0.25, 0.24), M["black"]))
    eng.merge(cylinder(V((1.02, -0.78, 1.13)), V((1.02, -(g.hw + 0.025), 1.14)), 0.04, 14, M["black"]))
    eng.merge(tube(spline([(1.0, -0.57, 0.99), (0.95, -0.50, 0.985 + dz * 0.5), (0.895, -0.40, 0.95 + dz),
                           (0.88, -0.31, 0.915 + dz), (0.93, -0.27, 0.905 + dz)], 4), 0.04, 14, M["rubber"]))
    # Riementrieb vorn: Kurbelwelle, Wasserpumpe, Lichtmaschine, Klimakompressor
    pulleys = [(0.0, 0.73, 0.085), (0.0, 0.885, 0.06), (0.225, 0.835, 0.035), (-0.225, 0.79, 0.045)]
    for y, z, r in pulleys:
        mot.merge(cylinder(V((1.755, y, z)), V((1.785, y, z)), r, 24 if r > 0.05 else 16, M["steel"]))
    mot.merge(cylinder(V((1.62, 0.225, 0.835)), V((1.75, 0.225, 0.835)), 0.065, 18, M["alu"]))      # Lichtmaschine
    mot.merge(cylinder(V((1.60, -0.225, 0.79)), V((1.75, -0.225, 0.79)), 0.06, 18, M["alu"]))       # Klimakompressor
    ring = []
    for y, z, r in pulleys:
        ring += [V((y + (r + 0.006) * math.cos(TAU * i / 32), z + (r + 0.006) * math.sin(TAU * i / 32), 0.0))
                 for i in range(32)]
    belt = [V((1.77, ring[i].x, ring[i].y)) for i in convex_hull_2d([p.to_2d() for p in ring])]
    mot_small.merge(tube(belt + [belt[0]], 0.006, 6, M["rubber"], caps=False))
    # Visco-Lüfter in der Zarge, Kühler mit Seitenkästen
    fc = V((1.975, 0.0, 0.885))
    mot.merge(cylinder(V((1.785, 0.0, 0.885)), V((1.89, 0.0, 0.885)), 0.022, 12, M["steel"]))
    mot.merge(cylinder(V((1.89, 0.0, 0.885)), V((1.955, 0.0, 0.885)), 0.065, 20, M["alu"]))
    mot.merge(cylinder(V((1.955, 0.0, 0.885)), V((1.99, 0.0, 0.885)), 0.045, 16, M["black"]))
    pitch = math.radians(25)
    for k in range(7):
        phi = TAU * k / 7
        d, t = V((0.0, math.cos(phi), math.sin(phi))), V((0.0, -math.sin(phi), math.cos(phi)))
        n, w = X_AXIS * math.cos(pitch) + t * math.sin(pitch), t * math.cos(pitch) - X_AXIS * math.sin(pitch)
        mot.merge(box((0, 0, 0), (0.11, 0.06, 0.005), M["black"], frame_matrix(fc + d * 0.097, d, w, n)))
    mot.merge(lathe([(0.168, 0.0), (0.174, 0.0), (0.174, 0.075), (0.168, 0.075)], 48, V((1.958, 0.0, 0.885)),
                    X_AXIS, M["black"], closed_profile=True))

    def to_rect(phi):
        """Strahl von der Lüftermitte zum Rand der Zarge (y ±0,32, z 0,715–1,02) – gleiche Punktzahl wie der Kreis."""
        c, s = math.cos(phi), math.sin(phi)
        ky = 0.32 / abs(c) if abs(c) > 1e-9 else 9.0
        kz = ((1.02 if s > 0 else 0.715) - 0.885) / s if abs(s) > 1e-9 else 9.0
        k = min(ky, kz)
        return (c * k, 0.885 + s * k)
    phis = [TAU * i / 48 for i in range(48)]
    hole = [(0.168 * math.cos(p), 0.885 + 0.168 * math.sin(p)) for p in phis]
    mot.merge(extrude_ring([to_rect(p) for p in phis], hole, 0.01,
                           frame_matrix(V((2.035, 0.0, 0.0)), Y_AXIS, Z_AXIS, X_AXIS), M["black"]))      # Zarge
    mot.merge(box((2.075, 0.0, 0.865), (0.05, 0.66, 0.31), M["chassis"]))                                  # Netz
    for side in (1, -1):
        mot.merge(box((2.075, side * 0.36, 0.865), (0.06, 0.06, 0.33), M["black"]))
        eng.merge(box((2.075, side * 0.30, 0.675 + dz / 2), (0.04, 0.04, 0.07 + dz), M["chassis"]))
    mot_small.merge(cylinder(V((2.075, 0.36, 1.03)), V((2.075, 0.36, 1.05)), 0.02, 14, M["chrome"]))
    # Kühlwasserschläuche, Thermostatgehäuse
    mot_small.merge(box((1.725, 0.06, 0.99), (0.04, 0.06, 0.05), M["alu"]))
    mot_small.merge(tube(spline([(2.07, 0.36, 0.99), (2.0, 0.36, 1.0), (1.92, 0.30, 1.005), (1.83, 0.14, 1.005),
                             (1.735, 0.06, 0.99)], 4), 0.021, 10, M["rubber"]))
    mot_small.merge(tube(spline([(2.07, -0.36, 0.73), (2.0, -0.36, 0.72), (1.90, -0.31, 0.70), (1.79, -0.27, 0.685),
                             (1.70, -0.17, 0.69), (1.66, -0.14, 0.70)], 4), 0.021, 10, M["rubber"]))
    # Batterie (links vorn), Bremskraftverstärker mit Hauptzylinder (Fahrerseite), Behälter, Sicherungskasten
    eng.merge(box((1.61, 0.57, z_shelf + 0.01), (0.30, 0.22, 0.02), M["chassis"]))
    eng.merge(box((1.61, 0.57, z_shelf + 0.09), (0.26, 0.18, 0.14), M["black"]))
    small.merge(box((1.61, 0.57, z_shelf + 0.1655), (0.02, 0.20, 0.011), M["steel"]))
    for x, m in ((1.53, M["valve"]), (1.69, M["black"])):
        small.merge(cylinder(V((x, 0.52, z_shelf + 0.16)), V((x, 0.52, z_shelf + 0.175)), 0.012, 10, m))
    eng.merge(cylinder(V((0.775, 0.42, 0.97)), V((0.89, 0.42, 0.97)), 0.12, 24, M["black"]))
    eng.merge(cylinder(V((0.89, 0.42, 0.97)), V((1.02, 0.42, 0.97)), 0.035, 12, M["alu"]))
    small.merge(box((0.955, 0.42, 1.03), (0.08, 0.06, 0.05), M["reverse"]))
    eng.merge(box((1.71, -0.59, z_shelf + 0.065), (0.16, 0.12, 0.13), M["reverse"]))        # Ausgleichsbehälter
    eng.merge(box((1.83, 0.54, z_shelf + 0.0725), (0.10, 0.14, 0.145), M["black"]))         # Waschwasser
    eng.merge(box((0.86, -0.62, 1.01), (0.10, 0.16, 0.10), M["black"]))                     # Sicherungen
    for p in ((1.68, -0.59, z_shelf + 0.13), (1.83, 0.56, z_shelf + 0.145), (1.56, 0.05, 1.058 + dz)):  # Deckel
        small.merge(cylinder(V(p), V(p) + Z_AXIS * 0.016, 0.022, 12, M["black"]))
    for x in (1.08, 1.24, 1.46, 1.62):                                                      # Deckelschrauben
        for side in (1, -1):
            mot_small.merge(cylinder(V((x, side * 0.138, 1.008)), V((x, side * 0.138, 1.022)), 0.007, 6, M["chrome"]))
    up = Matrix.Translation((0.0, 0.0, dz))                     # Motor samt Riementrieb, Lüfter und Kühler anheben
    eng.merge(mot, up)
    small.merge(mot_small, up)
    return eng, small


def build_offroad(cfg):
    g = OffroadGeo()
    liv = OFFROAD_LIVERIES.get(cfg.offroad_color, OFFROAD_LIVERIES["sand"])
    name = cfg.offroad_name
    M = offroad_materials(name, liv, cfg.dirt)
    asm = Assembly(name)
    asm.ao_distance = 0.35
    bevel_s = ("bevel", dict(width=0.0015, segments=1, angle=40))
    bevel = ("bevel", dict(width=0.004, segments=2, angle=35))
    st, gap = g.shell_t, 0.004
    rng = random.Random(cfg.seed)
    k_ws = (g.x_ws1 - g.x_ws0) / (g.z_roof - g.r_roof - g.z_belt)        # Scheibenneigung dx/dz
    n_ws = V((1.0, 0.0, -k_ws)).normalized()

    # ------------------------------------------------------------- Karosserie: Unterbau ∪ Kabine → Hülle
    body = loft_stations(g.LOWER, g.lower_xs(), 48, "ngon", "ngon", M["paint"]).to_object(f"{name}_Body", smooth=50)
    asm.coll.objects.link(body)
    boolean(body, [g.cabin_md(M["paint"]).to_object("_cab", smooth=50)], "UNION")
    set_smoothing(body, 50)
    body.data.materials.append(M["interior"])
    add_modifier(body, "solidify", thickness=st, material_offset=1)
    apply_modifiers(body)
    src = body.data.copy()

    def pillar(z, back):
        """X-Position parallel zur A-Säule (``back`` hinter der Scheibe)."""
        return g.ws_x(z) - back

    # Konturen: Seitenansicht (x, z), Front/Heck (y, z), Draufsicht (x, y)
    zw0, zw1, zd1 = g.z_belt + 0.07, g.z_roof - 0.13, g.z_roof - 0.07       # Fensterunter-/-oberkante, Türoberkante
    door_f = rounded_poly_2d([(0.70, 0.555), (0.70, g.z_belt - 0.03), (pillar(g.z_belt + 0.04, 0.08), g.z_belt + 0.04),
                              (pillar(zd1, 0.08), zd1), (-0.33, zd1), (-0.33, 0.555)], [0.03, 0.03, 0.02, 0.03, 0.04, 0.03], 4)
    win_f = rounded_poly_2d([(pillar(zw0, 0.14), zw0), (pillar(zw1, 0.14), zw1), (-0.27, zw1), (-0.27, zw0)],
                            [0.025, 0.03, 0.03, 0.03], 4)
    rf = g.arch_r + 0.085                            # Hintertür: Ausschnitt um Radlauf + Verbreiterung

    def arch_front(z):
        return g.ra.x + math.sqrt(max(rf * rf - (z - g.ra.z) ** 2, 0.0))
    z_meet = g.ra.z + math.sqrt(rf * rf - (g.ra.x + 1.00) ** 2)
    notch = [(arch_front(z), z) for z in (lerp(0.555, z_meet, i / 8) for i in range(9))]
    door_r = rounded_poly_2d([(-0.37, 0.555)] + notch + [(-1.00, zd1), (-0.37, zd1)],
                             [0.03] + [0.0] * 8 + [0.02, 0.04, 0.03], 4)
    win_r = rounded_poly_2d([(-0.43, zw0), (-0.43, zw1), (-0.94, zw1), (-0.94, zw0)], 0.03, 4)
    win_q = rounded_poly_2d([(-1.08, zw0), (-1.08, zw1), (-2.06, zw1), (-2.06, zw0)], [0.03, 0.03, 0.05, 0.03], 4)
    tg = rounded_poly_2d([(-0.77, 0.70), (0.77, 0.70), (0.77, zd1), (-0.77, zd1)], 0.05, 4)
    win_b = rounded_poly_2d([(-0.66, zw0 + 0.24), (0.66, zw0 + 0.24), (0.66, zw1), (-0.66, zw1)], 0.04, 4)
    ws = rounded_poly_2d([(-0.76, g.z_belt + 0.045), (0.76, g.z_belt + 0.045), (0.71, g.z_roof - 0.095),
                          (-0.71, g.z_roof - 0.095)], [0.04, 0.04, 0.05, 0.05], 4)
    hood = rounded_poly_2d([(0.80, -0.815), (2.185, -0.815), (2.185, 0.815), (0.80, 0.815)], [0.02, 0.05, 0.05, 0.02], 4)

    def arch_outline(a):
        return ([(a.x + g.arch_r, 0.10)] + [(a.x + g.arch_r * math.cos(t), a.z + g.arch_r * math.sin(t))
                                             for t in (math.pi * i / 32 for i in range(33))] + [(a.x - g.arch_r, 0.10)])

    def flat(points3d, mat, flip=False):
        vs = list(reversed(points3d)) if flip else list(points3d)
        return MeshData().add(vs, [tuple(range(len(vs)))], None, mat)

    def side_glass(outline, side):
        """Scheibe in der (geneigten) Kabinenwand, mittig in der Wandstärke; Rand verschwindet in der Wand."""
        pts = ensure_ccw(offset_poly(outline, 0.025))
        return flat([V((p.x, side * (g.cab_hw(p.y) - st * 0.5), p.y)) for p in pts], M["glass"], side > 0)

    def side_seal(outline, side):
        pts = ensure_ccw(offset_poly(outline, 0.008))
        path = [V((p.x, side * (g.cab_hw(p.y) + 0.003), p.y)) for p in pts]
        return tube(path + [path[0]], 0.0065, 6, M["rubber"], caps=False)

    def cut_panel(key, inner, holes=()):
        """Anbauteil (Tür, Haube, Hecktür) aus der Hülle schneiden – gleiche Wandstärke wie die Karosserie."""
        d = bpy.data.objects.new(f"{name}_{key}", src.copy())
        asm.coll.objects.link(d)
        boolean(d, [inner.to_object("_c")], "INTERSECT")
        if holes:
            boolean(d, [h.to_object("_c") for h in holes], "DIFFERENCE")
        transfer_normals(d, src)
        return d

    # ------------------------------------------------------------- Türen (4), Haube, Hecktür
    hinges = MeshData()                              # karosserieseitige Scharnierlappen
    for side in (1, -1):
        s = "L" if side > 0 else "R"
        y0, y1 = (0.30, 1.30) if side > 0 else (-1.30, -0.30)
        for row, outline, win, hx, handle_x in (("F", door_f, win_f, 0.705, -0.22), ("R", door_r, win_r, -0.375, -0.88)):
            key = f"Door_{row}{s}"
            d = cut_panel(key, prism_y(offset_poly(outline, -gap), y0, y1), [prism_y(win, y0, y1)])
            p_lo, p_hi = V((hx, side * (g.hw + 0.012), 0.70)), V((hx, side * (g.hw + 0.012), 1.22))
            asm.pivot(key, (p_lo + p_hi) / 2, (p_hi - p_lo).normalized() * (-1.0 if side > 0 else 1.0), orient=True,
                      positive="Tür öffnet nach außen", limits=(0, 75))
            asm.add(d, key, key)
            asm.add(side_glass(win, side), f"Glass_{key}", key, smooth=80)
            dt = MeshData()
            dt.merge(side_seal(win, side))
            dt.merge(box((handle_x, side * (g.hw + 0.011), g.z_belt - 0.10), (0.13, 0.022, 0.032), M["black"]))
            x_lo, x_hi = (-0.30, 0.67) if row == "F" else (-0.97, -0.40)          # Seitenschutzleiste
            dt.merge(box(((x_lo + x_hi) / 2, side * (g.hw + 0.006), 1.03), (x_hi - x_lo, 0.012, 0.055), M["black"]))
            for z in (0.80, 1.15):                  # außenliegende Scharniere: Rolle auf der Achse, Lappen je Seite
                yh = side * (g.hw + 0.012)
                dt.merge(cylinder(V((hx, yh, z - 0.05)), V((hx, yh, z + 0.05)), 0.011, 10, M["black"]))
                dt.merge(box((hx - 0.026, side * (g.hw + 0.004), z), (0.034, 0.008, 0.08), M["black"]))
                hinges.merge(box((hx + 0.024, side * (g.hw + 0.004), z), (0.024, 0.008, 0.08), M["black"]))
            asm.add(dt, f"DoorTrim_{row}{s}", key, mods=[bevel_s], detail=2)
        # Außenspiegel an der Vordertür
        mm = MeshData()
        base = V((0.585, side * (g.cab_hw(g.z_belt + 0.045) + 0.003), g.z_belt + 0.045))
        head = V((0.60, side * (g.hw + 0.185), g.z_belt + 0.17))
        mm.merge(tube([base, base + V((0.0, side * 0.08, 0.035)), head - V((0.0, side * 0.07, 0.045))], 0.014, 8,
                      M["black"]))
        hm = Matrix.Translation(head)
        mm.merge(box((0, 0, 0), (0.07, 0.19, 0.15), M["black"], hm))
        mm.merge(box((-0.037, 0, 0), (0.005, 0.17, 0.13), M["mirror"], hm))
        asm.add(mm, f"Mirror_{s}", f"Door_F{s}", mods=[bevel])

    hz0, hz1 = g.z_belt - 0.18, g.z_belt + 0.15                                 # Höhenbereich des Haubenschnitts
    hood_obj = cut_panel("Hood", prism_z(offset_poly(hood, -gap), hz0, hz1))
    zt = Pchip([s["x"] for s in g.LOWER], [s["zt"] for s in g.LOWER])
    asm.pivot("Hood", V((0.795, 0.0, zt(0.795) + 0.003)), -Y_AXIS, orient=True,
              positive="Haube öffnet (Vorderkante nach oben)", limits=(0, 65))
    asm.add(hood_obj, "Hood", "Hood")
    scoop = MeshData()                                # Lufthutze (Turbodiesel-Ladeluftkühler)
    prof = [(1.12, zt(1.12) - 0.012), (1.64, zt(1.64) - 0.012), (1.64, zt(1.64) + 0.062), (1.52, zt(1.52) + 0.068),
            (1.12, zt(1.12) + 0.004)]
    scoop.merge(prism_y(rounded_poly_2d(prof, [0.0, 0.0, 0.012, 0.03, 0.01], 3), -0.30, 0.30, M["paint"]))
    scoop.merge(box((1.6415, 0.0, zt(1.64) + 0.026), (0.004, 0.54, 0.05), M["black"]))
    asm.add(scoop, "HoodScoop", "Hood", mods=[bevel])

    def under_hood(pts, d=0.012):
        """Pfad (x, y) dicht unter der Haubeninnenseite."""
        return [V((p[0], p[1], zt(p[0]) - st - d)) for p in pts]
    hf = MeshData()                                  # lackierter Versteifungsrahmen mit X-Strebe
    frame = fillet_path([(1.495, -0.76, 0.0), (2.12, -0.76, 0.0), (2.12, 0.76, 0.0), (0.87, 0.76, 0.0),
                         (0.87, -0.76, 0.0), (1.495, -0.76, 0.0)], 0.07, 4)
    frame = resample(frame, 0.05)
    hf.merge(tube(under_hood([(p.x, p.y) for p in frame]), (0.008, 0.022), 8, M["paint"], caps=False, up=Z_AXIS,
                  exponent=4.0))
    for side in (1, -1):
        hf.merge(tube(under_hood([(lerp(0.93, 2.06, i / 12), side * lerp(0.70, -0.70, i / 12)) for i in range(13)]),
                      (0.007, 0.02), 8, M["paint"], up=Z_AXIS, exponent=4.0))
    asm.add(hf, "HoodFrame", "Hood", detail=1)
    tg_obj = cut_panel("Tailgate", prism_x(offset_poly(tg, -gap), g.x_rear - 0.06, g.x_rear + 0.08),
                       [prism_x(win_b, g.x_rear - 0.06, g.x_rear + 0.10)])
    # Scharnierachse folgt der leicht nach vorn geneigten Heckscheibenpartie (Rollen liegen an der Hecktür an)
    tg_axis = V((g.cab_xr(g.z_roof - g.r_roof) - g.cab_xr(g.z_belt), 0.0, g.z_roof - g.r_roof - g.z_belt)).normalized()
    tg_hinge = V((g.cab_xr(1.25) - 0.018, -0.773, 1.25))
    asm.pivot("Tailgate", tg_hinge, tg_axis, orient=True, limits=(0, 100),
              positive="Hecktür öffnet nach außen (Scharnier rechts)")
    asm.add(tg_obj, "Tailgate", "Tailgate")
    pts = ensure_ccw(offset_poly(win_b, 0.025))
    asm.add(flat([V((g.cab_xr(p.y) + st * 0.5, p.x, p.y)) for p in pts], M["glass"], True), "Glass_Tailgate",
            "Tailgate", smooth=80)
    path = [V((g.cab_xr(p.y) - 0.003, p.x, p.y)) for p in ensure_ccw(offset_poly(win_b, 0.008))]
    tt = tube(path + [path[0]], 0.0065, 6, M["rubber"], caps=False)
    for z in (g.z_belt + 0.08, g.z_roof - 0.20):   # außenliegende Scharniere (über den Rückleuchten)
        c = tg_hinge + tg_axis * ((z - tg_hinge.z) / tg_axis.z)
        tt.merge(cylinder(c - tg_axis * 0.06, c + tg_axis * 0.06, 0.012, 10, M["black"]))
        tt.merge(box((g.cab_xr(z) - 0.004, c.y + 0.029, z), (0.008, 0.034, 0.09), M["black"]))
    asm.add(tt, "TailgateTrim", "Tailgate", mods=[bevel_s], detail=2)
    asm.add(hinges, "Hinges", mods=[bevel_s], detail=2)

    # ------------------------------------------------------------- Öffnungen in der Karosserie, Scheiben
    cutters = []
    for side in (1, -1):
        y0, y1 = (0.30, 1.30) if side > 0 else (-1.30, -0.30)
        cutters += [prism_y(offset_poly(door_f, gap), y0, y1), prism_y(offset_poly(door_r, gap), y0, y1),
                    prism_y(win_q, y0, y1)]
        cutters += [prism_y(arch_outline(a), side * 0.445, side * 1.30) for a in (g.fa, g.ra)]
    cutters.append(prism_z(offset_poly(hood, gap), hz0, hz1))
    cutters.append(prism_x(offset_poly(tg, gap), g.x_rear - 0.06, g.x_rear + 0.10))
    cutters.append(prism_x(ws, 0.36, 0.80))
    cutters.append(box((1.44, 0.0, 0.52), (1.28, 0.88, 0.30)))          # Motorraum nach unten offen
    cutters.append(box((g.ra.x, 0.0, 0.50), (0.40, 0.40, 0.24)))        # Tasche über dem Hinterachs-Differenzial
    boolean(body, [c.to_object("_c") for c in cutters], "DIFFERENCE")
    transfer_normals(body, src)
    asm.add(body, "Body")
    bpy.data.meshes.remove(src)

    glass = MeshData()
    glass.merge(flat([V((g.ws_x(p.y), p.x, p.y)) - n_ws * (st * 0.5) for p in ensure_ccw(offset_poly(ws, 0.025))],
                     M["glass"]))
    for side in (1, -1):
        glass.merge(side_glass(win_q, side))
    asm.add(glass, "Glass", smooth=80)
    seals = MeshData()
    path = [V((g.ws_x(p.y), p.x, p.y)) + n_ws * 0.003 for p in ensure_ccw(offset_poly(ws, 0.008))]
    seals.merge(tube(path + [path[0]], 0.0065, 6, M["rubber"], caps=False))
    for side in (1, -1):
        seals.merge(side_seal(win_q, side))
        # Regenrinne über den Türen
        zg = g.z_roof - g.r_roof + 0.004
        seals.merge(tube([V((g.ws_x(zg) - 0.06, side * (g.hw_roof + 0.003), zg)),
                          V((g.cab_xr(zg) + 0.06, side * (g.hw_roof + 0.003), zg))], 0.006, 6, M["rubber"]))
    asm.add(seals, "Seals", detail=2)

    # ------------------------------------------------------------- Radhäuser und Kotflügelverbreiterungen
    trim = MeshData()
    for side in (1, -1):
        for a in (g.fa, g.ra):
            a0 = math.asin((0.52 - a.z) / g.arch_r)

            rl = g.arch_r - 0.004                      # knapp innerhalb der Schnittkante (kein Z-Fighting)

            def liner_fn(u, v, a=a, a0=a0, side=side, rl=rl):
                t = lerp(a0, math.pi - a0, u)
                return V((a.x + rl * math.cos(t), side * lerp(0.445, g.hw + 0.004, v), a.z + rl * math.sin(t)))
            trim.merge(surface_grid(liner_fn, 24, 2, M["black"], flip=side > 0))
            wall = [V((a.x + rl * math.cos(t), side * 0.445, a.z + rl * math.sin(t)))
                    for t in (lerp(a0, math.pi - a0, i / 24) for i in range(25))]
            trim.merge(flat(wall, M["black"], side > 0))
            rfl = g.arch_r + 0.034
            a1 = math.asin((0.52 - a.z) / rfl)
            path = [V((a.x + rfl * math.cos(t), side * (g.hw + 0.022), a.z + rfl * math.sin(t)))
                    for t in (lerp(a1, math.pi - a1, i / 32) for i in range(33))]
            trim.merge(tube(path, (0.038, 0.046), 12, M["black"], up=Y_AXIS, exponent=4.0))
    asm.add(trim, "Flares", mods=[bevel_s])

    # ------------------------------------------------------------- Front: Stoßfänger, Seilwinde, Grill, Scheinwerfer
    fr = MeshData()
    fr.merge(prism_z([(2.21, -0.93), (2.25, -0.93), (2.33, -0.66), (2.33, 0.66), (2.25, 0.93), (2.21, 0.93)],
                     0.47, 0.68, M["bumper"]))
    fog_lens = MeshData()
    for side in (1, -1):                                                # Zusatzscheinwerfer auf dem Stoßfänger
        c = V((2.33, side * 0.56, 0.75))
        fr.merge(box((2.29, side * 0.56, 0.695), (0.05, 0.03, 0.03), M["black"]))
        fr.merge(lathe([(0.0, -0.075), (0.045, -0.07), (0.056, -0.04), (0.058, 0.0), (0.054, 0.006)], 20, c, X_AXIS,
                       M["black"]))
        fr.merge(lathe([(0.0, -0.002), (0.03, 0.0), (0.0, 0.006)], 12, c, X_AXIS, M["lamp"]))
        fog_lens.merge(lathe([(0.052, 0.004), (0.035, 0.012), (0.0, 0.015)], 20, c, X_AXIS, M["lens"]))
    fr.merge(box((1.95, 0.0, 0.445), (0.55, 0.78, 0.012), M["bumper"]))                   # Unterfahrschutz
    fr.merge(box((2.385, 0.0, 0.525), (0.11, 0.62, 0.014), M["bumper"]))                  # Windenplatte
    fr.merge(cylinder(V((2.39, -0.15, 0.59)), V((2.39, 0.15, 0.59)), 0.055, 20, M["steel"]))     # Trommel
    fr.merge(cylinder(V((2.39, -0.16, 0.59)), V((2.39, -0.31, 0.59)), 0.06, 20, M["black"]))     # Motor
    fr.merge(box((2.39, 0.215, 0.59), (0.12, 0.13, 0.12), M["black"]))                     # Getriebe
    fr.merge(box((2.452, 0.0, 0.585), (0.018, 0.26, 0.09), M["alu"]))                      # Seilführung
    fr.merge(tube(fillet_path([(2.462, 0.0, 0.585), (2.50, 0.0, 0.585), (2.50, 0.0, 0.535)], 0.02), 0.008, 8,
                  M["steel"]))                                                              # Haken
    for side in (1, -1):                                                                    # Bergeösen
        y = side * 0.70
        fr.merge(tube(fillet_path([(2.30, y, 0.53), (2.36, y, 0.53), (2.36, y, 0.61), (2.30, y, 0.61)], 0.03), 0.011,
                      8, M["steel"]))
    zg0, zg1 = 0.74, 1.17                                                                   # Grill mit Rahmen
    fr.merge(box((2.226, 0.0, (zg0 + zg1) / 2), (0.022, 0.98, zg1 - zg0), M["black"]))
    fr.merge(tube(fillet_path([(2.24, -0.49, zg0), (2.24, 0.49, zg0), (2.24, 0.49, zg1), (2.24, -0.49, zg1),
                               (2.24, -0.49, zg0), (2.24, 0.0, zg0)], 0.03, 3), 0.014, 8, M["chrome"]))
    for i in range(7):
        fr.merge(box((2.241, 0.0, zg0 + 0.045 + i * 0.056), (0.014, 0.94, 0.026), M["black"]))
    for y in (-0.165, 0.165):
        fr.merge(box((2.243, y, (zg0 + zg1) / 2), (0.016, 0.026, zg1 - zg0 - 0.03), M["black"]))
    fr.merge(box((2.247, 0.0, (zg0 + zg1) / 2), (0.01, 0.16, 0.07), M["chrome"]))          # Emblem-Plakette
    lens = fog_lens
    for side in (1, -1):
        c = V((2.245, side * 0.665, 1.00))
        fr.merge(box((2.228, side * 0.665, 0.97), (0.036, 0.28, 0.40), M["black"]))          # Lampentopf
        fr.merge(lathe([(0.0, -0.010), (0.08, 0.0), (0.100, 0.013), (0.110, 0.022)], 32, c, X_AXIS, M["chrome"]))
        fr.merge(lathe([(0.0, 0.003), (0.035, 0.006), (0.0, 0.015)], 16, c, X_AXIS, M["lamp"]))
        lens.merge(lathe([(0.108, 0.020), (0.092, 0.033), (0.057, 0.042), (0.0, 0.045)], 32, c, X_AXIS, M["lens"]))
        fr.merge(box((2.249, side * 0.665, 0.825), (0.016, 0.16, 0.05), M["amber"]))          # Blinker
        fr.merge(box((1.98, side * (g.hw + 0.003), 1.12), (0.07, 0.006, 0.03), M["amber"]))  # Seitenblinker
    asm.add(fr, "Front", mods=[bevel_s])
    asm.add(lens, "HeadlightLens", smooth=60)

    # ------------------------------------------------------------- Heck: Stoßfänger, Leuchten, Anhängerkupplung
    rr = MeshData()
    rr.merge(prism_z([(-2.21, -0.93), (-2.21, 0.93), (-2.27, 0.93), (-2.33, 0.72), (-2.33, -0.72), (-2.27, -0.93)],
                     0.47, 0.65, M["bumper"]))
    rr.merge(box((-2.40, 0.0, 0.50), (0.20, 0.07, 0.07), M["bumper"]))                     # Aufnahme
    rr.merge(cylinder(V((-2.43, -0.05, 0.50)), V((-2.43, 0.05, 0.50)), 0.008, 8, M["steel"]))
    for side in (1, -1):
        y = side * 0.845
        rr.merge(box((-2.224, y, 1.05), (0.016, 0.085, 0.42), M["black"]))                    # Leuchtengehäuse
        rr.merge(box((-2.233, y, 1.165), (0.012, 0.07, 0.17), M["tail"]))
        rr.merge(box((-2.233, y, 1.025), (0.012, 0.07, 0.09), M["amber"]))
        rr.merge(box((-2.233, y, 0.915), (0.012, 0.07, 0.10), M["reverse"]))
        rr.merge(box((-2.333, side * 0.60, 0.56), (0.006, 0.10, 0.03), M["tail"]))           # Rückstrahler
    rr.merge(box((-2.18, 0.0, g.z_roof - 0.015), (0.03, 0.30, 0.03), M["tail"]))             # 3. Bremsleuchte
    asm.add(rr, "Rear", mods=[bevel_s])

    # ------------------------------------------------------------- Reserverad an der Hecktür
    wheel = offroad_wheel(M)
    spare_c = V((g.x_rear - 0.165, 0.0, 1.12))
    sm = Matrix.Translation(spare_c) @ Matrix.Rotation(math.pi / 2, 4, "Z")              # Achse +Y → −X
    spare = MeshData()
    for part in ("tire", "rim"):
        spare.merge(wheel[part], sm)
    spare.merge(box((g.x_rear - 0.025, 0.0, spare_c.z), (0.06, 0.36, 0.08), M["bumper"]))  # Träger
    spare.merge(box((g.x_rear - 0.025, 0.0, spare_c.z), (0.06, 0.08, 0.36), M["bumper"]))
    asm.add(spare, "Spare", "Tailgate")
    asm.add(wheel["tread"].copy().transform(sm), "SpareTread", "Tailgate", detail=1)

    # ------------------------------------------------------------- Dach: Gepäckträger mit Lichtleiste, Schnorchel
    rz = g.z_roof + 0.085
    ring = rrect_ring(-0.815, 1.135, 0.72, 0.10, 6, 12, 3, rz)
    rack = MeshData()
    rack.merge(tube(ring + [ring[0]], 0.016, 8, M["bumper"], caps=False))
    for x in (-0.05, -0.43, -0.81, -1.19, -1.57):
        rack.merge(cylinder(V((x, -0.72, rz)), V((x, 0.72, rz)), 0.012, 8, M["bumper"]))
    for x in (0.22, -0.82, -1.86):
        for side in (1, -1):
            rack.merge(cylinder(V((x, side * 0.72, g.z_roof - 0.01)), V((x, side * 0.72, rz)), 0.014, 8, M["bumper"]))
            rack.merge(box((x, side * 0.72, g.z_roof + 0.005), (0.08, 0.05, 0.02), M["black"]))
    rack.merge(box((0.36, 0.0, rz + 0.01), (0.07, 1.00, 0.08), M["black"]))                 # Lichtleiste
    asm.add(rack, "RoofRack", mods=[bevel_s])
    asm.add(box((0.396, 0.0, rz + 0.01), (0.006, 0.95, 0.055), M["lens"]), "LightBarLens", detail=2)

    snork = [V((1.02, -(g.hw + 0.035), 1.14)), V((0.87, -(g.hw + 0.042), 1.26))]     # Einlass über der Schutzleiste
    snork += [V((pillar(z, 0.035), -(g.cab_hw(z) + 0.046), z)) for z in (g.z_belt + 0.10, g.z_belt + 0.38, g.z_roof - 0.07)]
    snork.append(snork[-1] + V((0.0, 0.0, 0.07)))
    sn = tube(spline(snork, 6), 0.043, 12, M["black"])
    top = snork[-1]
    sn.merge(cylinder(top - X_AXIS * 0.06, top + X_AXIS * 0.08, 0.062, 16, M["black"]))     # Ansaugkopf
    sn.merge(cylinder(top + X_AXIS * 0.08, top + X_AXIS * 0.085, 0.05, 16, M["rubber"]))
    snorkel = asm.add(sn, "Snorkel", mods=[bevel_s])

    # ------------------------------------------------------------- Kleinteile: Schweller, Schmutzfänger, Wischer …
    sd = MeshData()
    for side in (1, -1):
        sd.merge(tube([V((0.80, side * (g.hw + 0.035), 0.47)), V((-0.80, side * (g.hw + 0.035), 0.47))], 0.032, 12,
                      M["bumper"]))
        for x in (0.55, -0.05, -0.65):
            sd.merge(box((x, side * 0.705, 0.495), (0.05, 0.52, 0.04), M["bumper"]))
        for x0, x1 in ((0.745, 2.10), (-2.12, -1.035)):                                     # Schutzleisten Karosserie
            sd.merge(box(((x0 + x1) / 2, side * (g.hw + 0.006), 1.03), (x1 - x0, 0.012, 0.055), M["black"]))
        for a in (g.fa, g.ra):
            sd.merge(box((a.x - 0.56, side * 0.80, 0.39), (0.01, 0.30, 0.30), M["rubber"]))
    sd.merge(cylinder(V((-1.70, g.hw - 0.002, 1.17)), V((-1.70, g.hw + 0.006, 1.17)), 0.055, 20, M["paint"]))  # Tank
    zw = g.z_belt + 0.068
    for y0, y1 in ((0.52, -0.12), (-0.02, -0.66)):                                                  # Wischer
        p0 = V((g.ws_x(zw), y0, zw)) + n_ws * 0.012
        p1 = V((g.ws_x(zw + 0.034), y1, zw + 0.034)) + n_ws * 0.012
        sd.merge(tube([p0, p1], 0.0065, 6, M["black"]))
        sd.merge(cylinder(p0 - n_ws * 0.01, p0 + n_ws * 0.012, 0.012, 10, M["black"]))
    asm.add(sd, "Details", mods=[bevel_s], detail=2)
    antenna = asm.add(cylinder(V((1.30, g.hw - 0.035, 1.24)), V((1.26, g.hw - 0.035, 2.12)), 0.004, 6, M["black"]), "Antenna",
                      detail=2, smooth=60)
    plate = MeshData()
    reg = "4X4 " + "".join(rng.choice("ABCDEFGHKLMNPRSTUVWXYZ") for _ in range(2)) + f" {rng.randint(100, 999)}"
    for x, y, w, fwd in ((2.333, 0.45, 0.34, 1.0), (-2.333, -0.40, 0.52, -1.0)):
        plate.merge(box((x + fwd * 0.003, y, 0.565), (0.006, w, 0.11), M["plate"]))
        fm = frame_matrix(V((x + fwd * 0.0075, y, 0.565)), Y_AXIS * fwd, Z_AXIS, X_AXIS * fwd)
        plate.merge(text_md(reg, 0.075 if w > 0.4 else 0.05, fm, M["decal"], bold=0.03))
    asm.add(plate, "Plates", detail=2)

    # ------------------------------------------------------------- Innenraum
    it = MeshData()
    xa0, xa1 = g.ra.x - g.arch_r, g.ra.x + g.arch_r                                         # hinteres Radhaus
    for x0, x1, hw in ((xa1, 0.65, 0.82), (xa0, xa1, 0.44), (-1.95, xa0, 0.82)):
        it.merge(box(((x0 + x1) / 2, 0.0, 0.715), (x1 - x0, 2 * hw, 0.03), M["floor"]))
    dash = rounded_poly_2d([(0.71, 0.82), (0.71, 1.26), (0.56, 1.27), (0.50, 1.19), (0.52, 0.95), (0.60, 0.82)],
                           [0.0, 0.0, 0.04, 0.05, 0.05, 0.04], 3)
    it.merge(extrude(dash, 1.72, frame_matrix(V((0, 0, 0)), X_AXIS, Z_AXIS, -Y_AXIS), M["panel"]))
    for y, w in ((0.40, 0.26), (0.0, 0.22)):                                                  # Instrumente, Display
        it.merge(box((0.504, y, 1.12), (0.006, w, 0.10), M["screen"]))
    it.merge(box((0.25, 0.0, 0.82), (0.40, 0.20, 0.20), M["panel"]))                         # Mittelkonsole
    for x, h in ((0.30, 0.16), (0.20, 0.12)):                                                 # Schalt-/Verteilerhebel
        it.merge(cylinder(V((x, 0.0, 0.92)), V((x - 0.02, 0.0, 0.92 + h)), 0.008, 8, M["chrome"]))
        it.merge(lathe([(0.0, -0.02), (0.022, 0.0), (0.0, 0.022)], 12, V((x - 0.022, 0.0, 0.94 + h)), Z_AXIS,
                       M["panel"]))

    def seat(x, y, w):
        sm_ = MeshData()
        sm_.merge(box((x, y, 0.95), (0.50, w, 0.12), M["fabric"]))
        sm_.merge(box((x, y, 0.815), (0.42, min(w * 0.86, 0.86), 0.17), M["panel"]))
        bm = Matrix.Translation((x - 0.24, y, 1.0)) @ Matrix.Rotation(math.radians(-14), 4, "Y")
        sm_.merge(box((0.0, 0.0, 0.31), (0.11, w, 0.62), M["fabric"], bm))
        for yy in ([0.0] if w < 1.0 else [-0.48, 0.0, 0.48]):
            sm_.merge(box((0.0, yy, 0.72), (0.09, 0.25, 0.17), M["fabric"], bm))
        return sm_
    for y in (0.40, -0.40):
        it.merge(seat(-0.02, y, 0.50))
    it.merge(seat(-0.86, 0.0, 1.46))
    asm.add(it, "Interior", mods=[bevel], detail=1)
    hub = V((0.40, 0.40, 1.12))
    col = V((-math.cos(math.radians(25)), 0.0, math.sin(math.radians(25))))                 # Lenksäule → Fahrer
    asm.add(cylinder(hub - col * 0.27, hub - col * 0.03, 0.024, 12, M["panel"]), "SteeringColumn", detail=1)
    asm.pivot("SteeringWheel", hub, col, orient=True, positive="Lenkrad nach links (gegen den Uhrzeigersinn)",
              limits=(-540, 540))
    u, w = basis_from(col)
    swm = lathe([(0.19 + 0.017 * math.cos(TAU * i / 10), 0.017 * math.sin(TAU * i / 10)) for i in range(10)], 40, hub,
                col, M["panel"], closed_profile=True)
    swm.merge(lathe([(0.0, -0.025), (0.06, -0.02), (0.06, 0.012), (0.0, 0.02)], 20, hub, col, M["panel"]))
    for d in (w, -w, -u):
        swm.merge(box((0, 0, 0), (0.016, 0.045, 0.13), M["panel"], frame_matrix(hub + d * 0.125, col.cross(d), col, d)))
    asm.add(swm, "SteeringWheel", "SteeringWheel", mods=[bevel_s], detail=1)

    # ------------------------------------------------------------- Motorraum (bei offener Haube sichtbar)
    eng, eng_small = offroad_engine_bay(g, M)
    asm.add(eng, "Engine", mods=[bevel_s], detail=1)
    asm.add(eng_small, "EngineDetails", detail=2)

    # ------------------------------------------------------------- Fahrwerk: Starrachsen, Achsschenkel, Räder
    chassis = MeshData()
    for side in (1, -1):                                                                     # Leiterrahmen
        chassis.merge(box((-0.05, side * 0.40, 0.59), (4.00, 0.07, 0.14), M["chassis"]))
    for x in (1.90, 0.95, 0.10, -0.80, -2.03):
        chassis.merge(box((x, 0.0, 0.59), (0.08, 0.80, 0.10), M["chassis"]))
    for a in (g.fa, g.ra):                                  # Federbein-Konsolen innen am Rahmen (Reifen bleibt frei)
        for side in (1, -1):
            chassis.merge(box((a.x, side * 0.4475, 0.6975), (0.12, 0.025, 0.195), M["chassis"]))
    chassis.merge(box((0.25, 0.0, 0.56), (0.50, 0.32, 0.20), M["chassis"]))                 # Getriebe/Verteiler
    chassis.merge(box((1.32, 0.0, 0.67), (0.62, 0.28, 0.22), M["chassis"]))                 # Ölwanne
    chassis.merge(box((-1.80, 0.08, 0.55), (0.38, 0.56, 0.18), M["chassis"]))               # Tank
    for a, sgn in ((g.fa, 1.0), (g.ra, -1.0)):                                             # Kardanwellen
        p0, p1 = V((0.25 + sgn * 0.27, 0.0, 0.52)), V((a.x - sgn * 0.22, 0.0, 0.42))
        chassis.merge(cylinder(p0, p1, 0.032, 12, M["steel"]))
        for p in (p0, p1):
            chassis.merge(box(p, (0.06, 0.07, 0.07), M["chassis"]))
    # Auspuff mit Seitenausgang vor dem rechten Hinterrad: kreuzt die verschränkende Hinterachse nicht
    ex = [V((1.05, -0.26, 0.55)), V((0.70, -0.28, 0.50)), V((-0.30, -0.28, 0.49)), V((-0.62, -0.28, 0.49)),
          V((-0.74, -0.42, 0.47)), V((-0.80, -0.65, 0.45)), V((-0.82, -0.86, 0.44))]
    chassis.merge(tube(spline(ex, 6), 0.03, 12, M["soot"]))
    chassis.merge(box((-0.375, -0.28, 0.49), (0.45, 0.16, 0.12), M["soot"]))               # Schalldämpfer
    asm.add(chassis, "Chassis", mods=[bevel_s], detail=1)
    tail_end, tail_dir = ex[-1], (ex[-1] - ex[-2]).normalized()

    for axle, a, nose in (("Axle_F", g.fa, -1.0), ("Axle_R", g.ra, 1.0)):
        asm.pivot(axle, a, X_AXIS, orient=True, positive="linkes Rad federt ein (Achsverschränkung)", limits=(-6, 6))
        ax = MeshData()
        ax.merge(cylinder(V((a.x, -0.67, a.z)), V((a.x, 0.67, a.z)), 0.045, 16, M["chassis"]))
        ax.merge(lathe([(0.0, -0.13), (0.10, -0.12), (0.15, -0.05), (0.155, 0.02), (0.12, 0.09), (0.06, 0.12),
                        (0.0, 0.125)], 24, a, X_AXIS * nose, M["chassis"]))                         # Differential
        ax.merge(cylinder(a + X_AXIS * nose * 0.10, a + X_AXIS * nose * 0.22, 0.05, 16, M["chassis"]))
        for side in (1, -1):
            ax.merge(box((a.x, side * 0.535, 0.455), (0.12, 0.10, 0.02), M["chassis"]))      # Federbeinaufnahme
        if axle == "Axle_R":
            for side in (1, -1):                                                             # Bremssättel hinten
                ax.merge(box((a.x - 0.085, side * 0.767, a.z + 0.095), (0.08, 0.055, 0.06), M["chassis"]))
        asm.add(ax, axle, axle, mods=[bevel_s])
    for side in (1, -1):
        s = "L" if side > 0 else "R"
        kp = V((-math.sin(math.radians(4)), -side * math.sin(math.radians(8)), 1.0)).normalized()   # Nachlauf, Spreizung
        key = f"Steer_F{s}"
        kc = V((g.fa.x, side * 0.69, g.fa.z))
        asm.pivot(key, kc, kp, orient=True, parent="Axle_F", positive="Rad lenkt nach links", limits=(-30, 30))
        kn = lathe([(0.0, -0.10), (0.05, -0.09), (0.07, -0.03), (0.07, 0.03), (0.05, 0.09), (0.0, 0.10)], 16, kc, kp,
                   M["chassis"])
        kn.merge(cylinder(kc, kc + Y_AXIS * side * 0.05, 0.042, 16, M["chassis"]))
        kn.merge(box((g.fa.x - 0.085, side * 0.767, g.fa.z + 0.095), (0.08, 0.055, 0.06), M["chassis"]))   # Bremssattel
        asm.add(kn, key, key, mods=[bevel_s])
    # Federbeine (Coilover): untere Hälfte an der Achse, obere am Rahmen. Beide visieren sich per Aim an und
    # teleskopieren so bei Achsverschränkung, statt Achse oder Federturm zu durchdringen.
    for axle, a in (("Axle_F", g.fa), ("Axle_R", g.ra)):
        for side in (1, -1):
            s = axle[-1] + ("L" if side > 0 else "R")
            lo_key, up_key = f"Coilover_{s}", f"CoiloverMount_{s}"
            lo, up = V((a.x, side * 0.535, 0.465)), V((a.x, side * 0.535, 0.795))
            d = (up - lo).normalized()
            asm.pivot(lo_key, lo, d, orient=True, parent=axle, motion="aim", target=up_key,
                      positive=f"lokales +Y zeigt auf {up_key}")
            asm.pivot(up_key, up, -d, orient=True, motion="aim", target=lo_key, positive=f"lokales +Y zeigt auf {lo_key}")
            cl, cu = MeshData(), MeshData()
            cl.merge(cylinder(lo - X_AXIS * 0.03, lo + X_AXIS * 0.03, 0.02, 12, M["steel"]))           # Auge
            cl.merge(cylinder(lo + d * 0.02, lo + d * 0.19, 0.026, 16, M["black"]))                     # Dämpfer
            cl.merge(cylinder(lo + d * 0.055, lo + d * 0.067, 0.066, 24, M["alu"]))                     # Federteller
            cl.merge(tube(helix(lo + d * 0.067, d, 0.056, 0.188, 4.5, 16), 0.009, 8, M["valve"], caps=False))
            cu.merge(lathe([(0.068, 0.002), (0.073, 0.002), (0.073, 0.145), (0.068, 0.145)], 32, up, -d, M["chassis"],
                           closed_profile=True))                                                        # Federtopf
            cu.merge(cylinder(up - d * 0.002, up - d * 0.014, 0.073, 32, M["chassis"]))                 # Deckel
            cu.merge(cylinder(up - d * 0.01, up - d * 0.25, 0.012, 10, M["chrome"]))                    # Kolbenstange
            asm.add(cl, lo_key, lo_key, mods=[bevel_s], detail=1)
            asm.add(cu, up_key, up_key, mods=[bevel_s], detail=1)
    for wk in ("FL", "FR", "RL", "RR"):
        c = g.wheel(wk)
        parent = f"Steer_F{wk[1]}" if wk[0] == "F" else "Axle_R"
        asm.pivot(f"Wheel_{wk}", c, Y_AXIS, parent=parent, positive="Rad rollt vorwärts", motion="spin")
        m = Matrix.Translation(c) if wk[1] == "L" else Matrix.Translation(c) @ Matrix.Rotation(math.pi, 4, "Z")
        wm = MeshData()
        for part in ("tire", "rim", "brake"):
            wm.merge(wheel[part], m)
        asm.add(wm, f"Wheel_{wk}", f"Wheel_{wk}", smooth=40)
        asm.add(wheel["tread"].copy().transform(m), f"Tread_{wk}", f"Wheel_{wk}", detail=1)

    # ------------------------------------------------------------- Sockets & Kollision
    asm.socket("Driver", (-0.02, 0.40, 1.00))
    asm.socket("Passenger", (-0.02, -0.40, 1.00))
    asm.socket("Rear_L", (-0.86, 0.45, 0.98))
    asm.socket("Rear_R", (-0.86, -0.45, 0.98))
    for side in (1, -1):
        asm.socket(f"Headlight_{'L' if side > 0 else 'R'}", (2.30, side * 0.665, 1.00), forward=X_AXIS)
    asm.socket("Exhaust", tail_end, forward=tail_dir)
    asm.socket("TowHitch", (-2.50, 0.0, 0.50), forward=-X_AXIS)
    asm.socket("Winch", (2.51, 0.0, 0.585), forward=X_AXIS)
    asm.socket("Roof", (-0.815, 0.0, rz + 0.02))

    def by_pivot(*keys):
        return [o for o in asm.meshes() if o["pivot"] in keys]
    corners = ("FL", "FR", "RL", "RR")
    wheels_axles = ({f"Wheel_{k}" for k in corners} | {"Axle_F", "Axle_R", "Steer_FL", "Steer_FR"}
                    | {f"Coilover_{k}" for k in corners} | {f"CoiloverMount_{k}" for k in corners})
    mirrors = [o for o in asm.meshes() if "_Mirror_" in o.name]
    spare_objs = [o for o in asm.meshes() if o.name.endswith(("_Spare", "_SpareTread"))]
    chassis_objs = [o for o in asm.meshes() if o.name.endswith("_Chassis")]
    shell = [o for o in asm.meshes() if o["pivot"] not in wheels_axles and o not in mirrors + spare_objs + chassis_objs
             and o not in (snorkel, antenna)]
    asm.collide("Lower", shell, lambda p: p.z <= g.z_belt + 0.03 or p.x > g.x_ws0 + 0.05)
    asm.collide("Cabin", shell, lambda p: p.z >= g.z_belt - 0.03 and p.x <= g.x_ws0 + 0.05)
    asm.collide("Chassis", chassis_objs + by_pivot(*(f"CoiloverMount_{k}" for k in corners)), None, 64)
    asm.collide("Axle_F", by_pivot("Axle_F", "Steer_FL", "Steer_FR", "Coilover_FL", "Coilover_FR"), None, 32)
    asm.collide("Axle_R", by_pivot("Axle_R", "Coilover_RL", "Coilover_RR"), None, 32)
    for wk in ("FL", "FR", "RL", "RR"):
        asm.collide(f"Wheel_{wk}", by_pivot(f"Wheel_{wk}"), None, 48)
    asm.collide("Spare", spare_objs, None, 48)
    for o in mirrors:
        asm.collide(o.name.split("_")[-2] + "_" + o.name.split("_")[-1], [o], None, 24)
    asm.collide("Snorkel", [snorkel], None, 32)
    asm.hull_exempt.append(([antenna], "dünne Antenne"))
    return asm


# =============================================================================
# 8d) Motorboot (Bowrider ~6,4 m, Tiefen-V-Rumpf, Außenborder ~200 PS)
# =============================================================================

BOAT_LIVERIES = {
    "navy": dict(top=(0.015, 0.035, 0.12), stripe=(0.45, 0.02, 0.015), bottom=(0.80, 0.80, 0.78),
                 accent=(0.015, 0.035, 0.12), cowl=(0.02, 0.02, 0.022)),
    "white": dict(top=(0.80, 0.80, 0.78), stripe=(0.01, 0.10, 0.36), bottom=(0.80, 0.80, 0.78),
                  accent=(0.01, 0.10, 0.36), cowl=(0.80, 0.80, 0.78)),
    "red": dict(top=(0.42, 0.02, 0.015), stripe=(0.02, 0.02, 0.022), bottom=(0.80, 0.80, 0.78),
                accent=(0.42, 0.02, 0.015), cowl=(0.02, 0.02, 0.022)),
    "graphite": dict(top=(0.05, 0.055, 0.06), stripe=(0.85, 0.32, 0.02), bottom=(0.025, 0.025, 0.03),
                     accent=(0.85, 0.32, 0.02), cowl=(0.02, 0.02, 0.022)),
}


def mat_teak(name):
    """Teak-Laufdeck: Planken längs (Maserung über gestreckte Noise), dunkle Fugen alle ~6,5 cm."""
    m, nd = new_mat(name)
    bsdf = _principled(nd)
    co = nd.coords("Object")
    wave = nd.new("ShaderNodeTexWave", wave_type="BANDS", bands_direction="Y")
    wave.inputs["Scale"].default_value = 4.8
    nd.link(co, wave.inputs["Vector"])
    caulk = nd.map_range(wave.outputs["Fac"], 0.965, 0.985)
    grain = nd.noise(nd.coords("Object", (1.5, 40.0, 40.0)), 3.0, 4.0, 0.6)
    wood = nd.mix_color(grain, (0.30, 0.17, 0.075), (0.20, 0.11, 0.045))
    nd.link(nd.mix_color(caulk, wood, (0.02, 0.02, 0.02)), bsdf.inputs["Base Color"])
    nd.link(nd.map_range(caulk, 0.0, 1.0, 0.62, 0.45), bsdf.inputs["Roughness"])
    b = nd.new("ShaderNodeBump")
    b.inputs["Strength"].default_value = 0.4
    b.inputs["Distance"].default_value = 0.002
    nd.link(nd.math("MULTIPLY", caulk, -1.0), b.inputs["Height"])
    nd.link(b.outputs["Normal"], bsdf.inputs["Normal"])
    m.diffuse_color = (0.28, 0.16, 0.07, 1.0)
    return m


def boat_materials(prefix, liv, dirt):
    P = f"M_{prefix}_"
    bands = [(-9.0, 0.03, liv["bottom"]), (0.03, 0.085, liv["stripe"]), (0.105, 0.12, liv["stripe"])]  # Wasserpass
    dark_top = sum(liv["top"]) < 0.9
    return {
        "hull": mat_paint(P + "Hull", liv["top"], 0.22, coat=0.8, livery=bands, dirt=dirt * 0.4, dirt_z=(-0.35, 0.05)),
        "deck": mat_plastic(P + "Deck", (0.82, 0.82, 0.80), 0.45, grain=0.25),           # rutschfest strukturiert
        "liner": mat_paint(P + "Liner", (0.80, 0.80, 0.78), 0.28, coat=0.4),
        "vinyl": uv_weight(mat_plastic(P + "Vinyl", (0.83, 0.82, 0.77), 0.42, grain=0.12), 0.6),
        "accent": uv_weight(mat_plastic(P + "VinylAccent", liv["accent"], 0.42, grain=0.12), 0.6),
        "teak": uv_weight(mat_teak(P + "Teak"), 0.8),
        "steel": mat_metal(P + "Stainless", (0.80, 0.80, 0.82), 0.12),
        "black": mat_plastic(P + "PlasticBlack", (0.02, 0.02, 0.022), 0.5, grain=0.1),
        "rubber": mat_rubber(P + "Rubber", (0.015, 0.015, 0.015), 0.7),
        "glass": mat_glass(P + "Windshield", (0.45, 0.50, 0.52)),
        "lens": mat_glass(P + "Lens", (0.95, 0.95, 0.95)),
        "red": mat_emit(P + "NavRed", (1.0, 0.03, 0.02), 4.0, base=(0.5, 0.01, 0.01)),
        "green": mat_emit(P + "NavGreen", (0.02, 1.0, 0.15), 4.0, base=(0.01, 0.4, 0.05)),
        "white": mat_emit(P + "NavWhite", (1.0, 0.97, 0.9), 4.0),
        "panel": uv_weight(mat_plastic(P + "Dash", (0.025, 0.025, 0.028), 0.55), 0.5),
        "screen": uv_weight(mat_emit(P + "Screen", (0.10, 0.22, 0.32), 1.5, base=(0.01, 0.01, 0.012)), 0.5),
        "cowl": mat_paint(P + "Cowling", liv["cowl"], 0.25, coat=0.7),
        "motor": mat_paint(P + "LowerUnit", (0.03, 0.03, 0.033), 0.35, coat=0.3, dirt=dirt * 0.5, dirt_z=(-0.7, -0.2)),
        "prop": mat_metal(P + "Propeller", (0.75, 0.75, 0.77), 0.18),
        "zinc": mat_metal(P + "Anode", (0.45, 0.45, 0.46), 0.6),
        "decal": uv_weight(mat_paint(P + "Decal", (0.85, 0.85, 0.83) if dark_top else (0.02, 0.02, 0.022), 0.3), 2.0),
        "cowl_decal": uv_weight(mat_paint(P + "CowlDecal", (0.75, 0.75, 0.76) if sum(liv["cowl"]) < 0.9
                                          else (0.02, 0.02, 0.022), 0.3), 2.0),
    }


class BoatGeo:
    """Bowrider ~6,4 m (Länge über alles mit Außenborder ≈ 7,1 m), Breite 2,42 m, Aufkimmung 20° am Spiegel.
    Ursprung mittschiffs auf der Konstruktionswasserlinie (z = 0) – das Boot schwimmt, wenn man es mit dem
    Ursprung auf die Wasseroberfläche setzt; Kiel und Unterwasserschiff liegen darunter."""

    X_TR, X_BOW = -3.10, 3.25
    KEEL = [(-3.10, -0.30), (0.0, -0.30), (1.5, -0.24), (2.4, -0.10), (2.9, 0.12), (3.15, 0.45), (3.25, 0.82)]
    CHINE_Z = [(-3.10, 0.03), (0.0, 0.11), (1.5, 0.22), (2.4, 0.33), (2.9, 0.43), (3.15, 0.58), (3.25, 0.84)]
    CHINE_W = [(-3.10, 0.92), (-1.0, 0.98), (0.5, 0.95), (1.5, 0.82), (2.4, 0.55), (2.9, 0.27), (3.15, 0.09),
               (3.25, 0.01)]
    SHEER_Z = [(-3.10, 0.76), (-1.0, 0.78), (0.5, 0.83), (1.5, 0.90), (2.4, 0.97), (2.9, 1.01), (3.15, 1.03),
               (3.25, 1.04)]
    SHEER_W = [(-3.10, 1.12), (-1.0, 1.20), (0.0, 1.21), (1.0, 1.15), (1.8, 1.00), (2.4, 0.78), (2.9, 0.46),
               (3.15, 0.19), (3.25, 0.01)]

    def __init__(self):
        def tab(t):
            return Pchip([p[0] for p in t], [p[1] for p in t])
        self.zk, self.zc, self.wc, self.zs, self.ws = (tab(t) for t in (self.KEEL, self.CHINE_Z, self.CHINE_W,
                                                                         self.SHEER_Z, self.SHEER_W))
        self.crown, self.shell = 0.04, 0.03
        self.gunwale = 0.17                          # Breite des Seitendecks neben dem Cockpit
        self.x_cp0, self.x_cp1 = -2.45, 2.35         # Cockpit von achtern bis vorn
        self.x_step, self.z_sole, self.z_sole_bow = 1.20, 0.10, 0.22
        self.well = (-2.58, 0.40, 0.335)             # Motorschacht: Vorderwand x, halbe Breite, Boden (25"-Schaft)
        self.tilt = V((-3.24, 0.0, 0.30))            # Kipprohr des Außenborders
        self.steer = V((-3.34, 0.0, 0.30))           # Lenkachse (Schwenkrohr)
        self.prop = V((-3.90, 0.0, -0.42))

    def clampx(self, x):
        return clamp(x, self.X_TR, self.X_BOW)

    def sheer(self, x):
        return self.zs(self.clampx(x))

    def half(self, x):
        """Halber Spant (y ≥ 0, z) von der Decksmitte über Deckskante, Bordwand, Kimm und Boden zum Kiel."""
        zk, zc, wc, zs, ws = self.zk(x), self.zc(x), max(self.wc(x), 0.004), self.zs(x), max(self.ws(x), 0.005)
        bow = smoothstep(1.4, 3.1, x)
        pts = [(0.0, zs + self.crown)]
        pts += [(ws * f, zs + self.crown * (1.0 - f * f)) for f in (0.3, 0.6, 0.85, 0.96)]
        pts += [(ws, zs), (ws * 0.997, zs - 0.025)]
        flare = lerp(1.15, 2.2, bow)                 # vorn stark ausgestellte Bordwand
        for t in (0.85, 0.65, 0.45, 0.25, 0.1):
            pts.append((wc + (ws * 0.997 - wc) * t ** flare, zc + (zs - 0.025 - zc) * t))
        wf = lerp(0.07, 0.015, bow)                  # Kimmkante, leicht nach unten geneigt (Spritzabweiser)
        pts += [(wc, zc), (max(wc - wf, wc * 0.6), zc - 0.012)]
        yi, zi = pts[-1]
        for f in (0.75, 0.5, 0.25):
            pts.append((yi * f, zk + (zi - zk) * f))
        pts.append((0.0, zk))
        return pts

    def ring(self, x):
        h = self.half(x)
        return [V((x, -y, z)) for y, z in h] + [V((x, y, z)) for y, z in reversed(h[1:-1])]

    def hull_y(self, x, z):
        """Halbe Außenbreite des Rumpfs auf Höhe z (zwischen Kiel und Deckskante)."""
        x = self.clampx(x)
        h = self.half(x)[5:]                          # ab Deckskante abwärts (z fällt monoton)
        if z >= h[0][1]:
            return h[0][0]
        for (y0, z0), (y1, z1) in zip(h, h[1:]):
            if z1 <= z <= z0:
                return lerp(y1, y0, (z - z1) / max(z0 - z1, 1e-9))
        return 0.0

    def cockpit_w(self, x):
        return max(self.ws(self.clampx(x)) - self.gunwale, 0.05)

    def cockpit_outline(self):
        """Draufsicht des Cockpit-Ausschnitts (CCW): gerade Achterkante, vorn halbelliptisch geschlossen."""
        x0, x1 = self.x_cp0, self.x_cp1 - 0.22
        xs = [lerp(x0 + 0.06, x1, i / 28) for i in range(29)]
        w1 = self.cockpit_w(x1)
        right = [(x, -self.cockpit_w(x)) for x in xs]
        arc = [(x1 + 0.22 * math.sin(math.pi * k / 12), -w1 * math.cos(math.pi * k / 12)) for k in range(1, 12)]
        left = [(x, self.cockpit_w(x)) for x in reversed(xs)]
        w0 = self.cockpit_w(x0)
        aft = [(x0, w0 - 0.06), (x0, -(w0 - 0.06))]
        return ensure_ccw([V(p) for p in right + arc + left + aft])

    def sole_z(self, x):
        return self.z_sole if x < self.x_step else self.z_sole_bow


def clip_x(poly, x, keep_below):
    """Polygon (Draufsicht) an der Geraden x = const abschneiden (Sutherland–Hodgman)."""
    def inside(p):
        return p.x <= x if keep_below else p.x >= x
    out = []
    for a, b in zip(poly, poly[1:] + poly[:1]):
        if inside(a):
            out.append(a)
        if inside(a) != inside(b):
            t = (x - a.x) / (b.x - a.x)
            out.append(a.lerp(b, t))
    return out


def boat_outboard(g, M):
    """Außenborder (V6, ~200 PS, 25"-Schaft): Teile je Pivot. Gibt dict(tilt, steer, prop, decal) zurück."""
    tilt, steer, prop = MeshData(), MeshData(), MeshData()
    s = g.steer
    tilt.merge(cylinder(V((g.tilt.x, -0.17, g.tilt.z)), V((g.tilt.x, 0.17, g.tilt.z)), 0.022, 14, M["steel"]))
    tilt.merge(box((g.tilt.x - 0.05, 0.0, 0.25), (0.14, 0.20, 0.16), M["motor"]))           # Schwenklager
    tilt.merge(cylinder(V((s.x, 0.0, 0.02)), V((s.x, 0.0, 0.56)), 0.036, 16, M["motor"]))     # Schwenkrohr
    # Motorhaube (oben) und Unterwanne, Schaft als Profil, Antikavitationsplatte, Getriebe, Skeg
    up = [dict(x=-4.05, hw=0.22, zt=1.03, zb=0.60, et=3.0, eb=6.0), dict(x=-3.95, hw=0.27, zt=1.10, zb=0.60, et=3.0, eb=6.0),
          dict(x=-3.70, hw=0.285, zt=1.14, zb=0.60, et=3.2, eb=6.0), dict(x=-3.45, hw=0.27, zt=1.10, zb=0.60, et=3.2, eb=6.0),
          dict(x=-3.33, hw=0.20, zt=0.99, zb=0.60, et=3.0, eb=6.0)]
    cowl = loft_stations(up, cosine_spacing(-4.05, -3.33, 20, 0.6, 0.6), 40, "ngon", "ngon", M["cowl"])
    steer.merge(cowl)
    pan = [dict(x=-4.00, hw=0.25, zt=0.65, zb=0.44, et=5.0, eb=4.0), dict(x=-3.70, hw=0.275, zt=0.66, zb=0.42, et=5.0, eb=4.0),
           dict(x=-3.37, hw=0.21, zt=0.65, zb=0.45, et=5.0, eb=4.0)]
    steer.merge(loft_stations(pan, cosine_spacing(-4.00, -3.37, 12, 0.5, 0.5), 32, "ngon", "ngon", M["motor"]))
    secs = []
    for z, x_le, c, t in ((0.46, -3.36, 0.34, 0.20), (0.15, -3.37, 0.31, 0.18), (-0.20, -3.38, 0.30, 0.16),
                          (-0.33, -3.39, 0.36, 0.13), (-0.44, -3.40, 0.40, 0.10)):
        secs.append(dict(le=V((x_le, 0.0, z)), chord_dir=-X_AXIS, thick_dir=Y_AXIS, chord=c, t=t))
    steer.merge(airfoil_loft(secs, 20, M["motor"]))
    plate = rounded_poly_2d([(-3.38, -0.17), (-3.86, -0.14), (-3.86, 0.14), (-3.38, 0.17)], [0.06, 0.04, 0.04, 0.06], 3)
    steer.merge(prism_z(plate, -0.285, -0.270, M["motor"]))
    steer.merge(lathe([(0.0, 0.0), (0.035, 0.012), (0.06, 0.05), (0.072, 0.12), (0.072, 0.36), (0.064, 0.44),
                       (0.052, 0.48)], 28, V((-3.38, 0.0, g.prop.z)), -X_AXIS, M["motor"]))   # Getriebegehäuse
    steer.merge(prism_y([(-3.60, -0.47), (-3.84, -0.47), (-3.82, -0.66), (-3.70, -0.66)], -0.011, 0.011, M["motor"]))
    steer.merge(box((-3.62, 0.0, -0.293), (0.12, 0.05, 0.016), M["zinc"]))                       # Anode
    for side in (1, -1):                                                                            # Kühlwassereinlass
        steer.merge(box((-3.47, side * 0.052, -0.36), (0.07, 0.006, 0.04), M["black"]))
    # Propeller: Nabe mit Abgasaustritt, drei breite Blätter (Steigung 19")
    p = g.prop
    prop.merge(lathe([(0.052, 0.0), (0.056, 0.02), (0.056, 0.10), (0.046, 0.14), (0.028, 0.155)], 28, V((-3.86, 0.0, p.z)),
                     -X_AXIS, M["prop"]))
    et = X_AXIS.cross(Z_AXIS)                         # Drehrichtung im Uhrzeigersinn (von hinten gesehen)
    blade_secs = []
    R, pitch = 0.18, 0.48
    for r in cosine_spacing(0.052, R, 12, 0.2, 0.9):
        f = (r - 0.052) / (R - 0.052)
        beta = math.atan(pitch / (TAU * r))
        c = lerp(0.09, 0.16, smoothstep(0.0, 0.55, f)) * math.sqrt(max(0.1, 1.0 - smoothstep(0.75, 1.0, f) ** 2))
        cd = -math.cos(beta) * et - math.sin(beta) * X_AXIS
        td = math.cos(beta) * X_AXIS - math.sin(beta) * et
        t = lerp(0.16, 0.05, f)
        skew = -et * (0.03 * f * f)                   # Blattspitzen nach hinten gebogen (Skew)
        blade_secs.append(dict(le=p + V((-0.07, 0.0, 0.0)) + Z_AXIS * r + skew - cd * (0.4 * c), chord_dir=cd,
                               thick_dir=td, chord=c, t=t, ring=airfoil_ring(12, t, 0.03)))
    blade = airfoil_loft(blade_secs, mat=M["prop"])
    for k in range(3):
        rot = Matrix.Translation(p) @ Matrix.Rotation(TAU * k / 3, 4, "X") @ Matrix.Translation(-p)
        prop.merge(blade.copy().transform(rot))
    # Schriftzug auf beiden Haubenseiten
    decal = MeshData()
    for side in (1, -1):
        fm = frame_matrix(V((-3.69, side * 0.40, 0.90)), -X_AXIS * side, Z_AXIS, Y_AXIS * side)
        decal.merge(project_onto(text_md("V6 200", 0.075, fm, M["cowl_decal"], bold=0.02), cowl, -Y_AXIS * side,
                                 max_edge=0.02))
    return dict(tilt=tilt, steer=steer, prop=prop, decal=decal)


def build_boat(cfg):
    g = BoatGeo()
    liv = BOAT_LIVERIES.get(cfg.boat_color, BOAT_LIVERIES["navy"])
    name = cfg.boat_name
    M = boat_materials(name, liv, cfg.dirt)
    asm = Assembly(name)
    asm.ao_distance = 0.3
    bevel_s = ("bevel", dict(width=0.0015, segments=1, angle=40))
    bevel = ("bevel", dict(width=0.006, segments=2, angle=35))
    rng = random.Random(cfg.seed)
    st = g.shell

    # ------------------------------------------------------------- Rumpf + Deck: geschlossener Loft → Schale
    rings = [g.ring(x) for x in cosine_spacing(g.X_TR, g.X_BOW, 64, 0.3, 0.9)]
    # Spiegel als Dreiecksfächer: ein konkaves N-Eck bringt den exakten Boolean nach dem Solidify durcheinander
    hull_md = grid_rings(rings, True, sum(rings[0], V()) / len(rings[0]), "ngon", M["hull"])
    hull_md.recalc = True
    cols = len(rings[0])
    deck_j = set(range(5)) | set(range(cols - 5, cols))                     # Segmente Decksmitte ↔ Deckskante
    for i in range((len(rings) - 1) * cols):
        if i % cols in deck_j:
            hull_md.mats[i] = M["deck"]
    body = hull_md.to_object(f"{name}_Hull", smooth=35)
    asm.coll.objects.link(body)
    for _ in range(2):
        body.data.materials.append(M["liner"])     # Innenseite von Deck und Rumpf → Liner (Slot +2)
    add_modifier(body, "solidify", thickness=st, material_offset=2)
    apply_modifiers(body)
    idx = [0] * len(body.data.polygons)              # beide Innenseiten auf einen Liner-Slot
    body.data.polygons.foreach_get("material_index", idx)
    body.data.polygons.foreach_set("material_index", [min(i, 2) for i in idx])
    body.data.materials.pop()
    src = body.data.copy()
    outline = g.cockpit_outline()
    wx, wy, wz = g.well
    # Cockpit-Ausschnitt nur durch das Deck: Unterkante folgt der Deckslinie (vorn ist die ausgestellte
    # Bordwand weiter unten schmaler als der Ausschnitt – ein flaches Prisma würde die Außenhaut lochen)
    cut = grid_rings([[V((p.x, p.y, g.sheer(p.x) - 0.07)) for p in outline], [V((p.x, p.y, 2.0)) for p in outline]],
                     True, "ngon", "ngon", M["deck"])
    cut.recalc = True
    notch = prism_x(rounded_poly_2d([(-wy, wz), (wy, wz), (wy, 1.4), (-wy, 1.4)], [0.06, 0.06, 0.0, 0.0], 3),
                    g.X_TR - 0.2, wx, M["deck"])
    # einzeln schneiden: mit einer Cutter-Sammlung übernimmt der exakte Boolean hier die Unterseite des
    # Cockpit-Cutters als Fläche (ein 9-m²-„Deckel“ über dem Cockpit)
    for c in (cut, notch):
        boolean(body, [c.to_object("_c")], "DIFFERENCE")
    transfer_normals(body, src)
    bpy.data.meshes.remove(src)
    asm.add(body, "Hull")

    # Spritzleisten (je zwei pro Seite), Trimmklappen
    strakes = MeshData()
    for frac, x_end in ((0.62, 2.55), (0.30, 2.25)):
        srings = []
        for x in [lerp(g.X_TR, x_end, i / 40) for i in range(41)]:
            h = g.half(x)
            (yi, zi), (yk, zk) = h[-5], h[-1]
            fade = smoothstep(x_end, x_end - 0.5, x)
            pa = V((x, yk + (yi - yk) * frac, zk + (zi - zk) * frac))
            pb = V((x, yk + (yi - yk) * (frac + 0.035 * fade + 0.002), zk + (zi - zk) * (frac + 0.035 * fade + 0.002)))
            srings.append([pa, V((x, pb.y, pa.z)), pb])
        md = grid_rings(srings, True, "ngon", "ngon", M["hull"])
        md.recalc = True
        strakes.merge(md)
        strakes.merge(md.mirrored())
    h = g.half(g.X_TR)
    (yi, zi), (yk, zk) = h[-5], h[-1]
    s_dir = V((0.0, yi, zi - zk)).normalized()
    for sgn in (1, -1):
        c = V((g.X_TR - 0.11, 0.0, zk)) + s_dir * (0.66 / s_dir.y)
        n = X_AXIS.cross(s_dir)
        tab = box((0, 0, 0), (0.22, 0.30, 0.012), M["steel"], frame_matrix(c - n * 0.006, X_AXIS, s_dir, n))
        strakes.merge(tab if sgn > 0 else tab.mirrored())
    asm.add(strakes, "Strakes", detail=1)

    # ------------------------------------------------------------- Cockpit: Liner-Wände, Laufdeck, Motorschacht
    def wall_top(x):
        return g.sheer(x) - st - 0.005

    def wall_y(x, z):
        """Halbe Breite der Cockpitwand auf Höhe z: Gerade vom Laufdeck zur Deckskante, aber nie außerhalb
        der (vorn konkav ausgestellten) Rumpfinnenseite."""
        wt = g.cockpit_w(x) - 0.005
        zb, zt = g.sole_z(x), wall_top(x)
        wb = min(wt, g.hull_y(x, zb) - st - 0.03)
        z = min(z, zt)
        return min(lerp(wb, wt, clamp((z - zb) / max(zt - zb, 1e-6))), g.hull_y(x, z) - st - 0.02)

    top = offset_poly(outline, -0.005)
    wall_rings = []
    for f in (1.0, 0.8, 0.6, 0.4, 0.2, 0.0):                  # von der Deckskante zum Laufdeck
        ring = []
        for p in top:
            z = lerp(g.sole_z(p.x), wall_top(p.x), f)
            ring.append(V((p.x, p.y * wall_y(p.x, z) / wall_y(p.x, 9.0), z)))
        wall_rings.append(ring)
    ring_b = wall_rings[-1]
    a, b, c_ = wall_rings[0][0], wall_rings[0][1], wall_rings[1][0]
    inward = V((0.0, -math.copysign(1.0, a.y), 0.0))
    flip = (b - a).cross(c_ - a).dot(inward) < 0
    liner = grid_rings(wall_rings, True, None, None, M["liner"], flip=flip)
    soles = MeshData()
    floor_poly = [V((p.x, p.y)) for p in ring_b]
    for keep_below, z in ((True, g.z_sole), (False, g.z_sole_bow)):
        poly = clip_x(floor_poly, g.x_step, keep_below)
        soles.merge(prism_z(poly, z - 0.03, z, M["teak"]))
    w_step = wall_y(g.x_step, g.z_sole)
    liner.merge(box((g.x_step, 0.0, (g.z_sole + g.z_sole_bow) / 2), (0.02, 2 * w_step, g.z_sole_bow - g.z_sole), M["liner"]))
    zd = g.sheer(g.X_TR) - st                                                   # Motorschacht (Splashwell)
    liner.merge(box(((g.X_TR + 0.03 + wx) / 2, 0.0, wz - 0.015), (wx - g.X_TR - 0.03, 2 * wy, 0.03), M["liner"]))
    for side in (1, -1):
        liner.merge(box(((g.X_TR + 0.03 + wx) / 2, side * (wy + 0.01), (wz + zd) / 2), (wx - g.X_TR - 0.03, 0.02, zd - wz),
                        M["liner"]))
    liner.merge(box((wx + 0.02, 0.0, (wz + zd) / 2), (0.02, 2 * wy + 0.04, zd - wz), M["liner"]))
    asm.add(liner, "Liner")
    asm.add(soles, "Sole")

    # Badeplattformen beidseits des Motors, Badeleiter (Pivot)
    plat = MeshData()
    for side in (1, -1):
        poly = rounded_poly_2d([(g.X_TR + 0.01, side * 0.47), (-3.52, side * 0.47), (-3.52, side * 1.02),
                                (g.X_TR + 0.01, side * 1.04)], [0.0, 0.03, 0.09, 0.0], 4)
        plat.merge(prism_z(poly, 0.115, 0.17, M["deck"]))
        plat.merge(prism_y([(g.X_TR + 0.02, -0.06), (g.X_TR + 0.02, 0.118), (-3.46, 0.118)], side * 0.50, side * 0.98,
                           M["hull"]))
    plat.merge(box((-3.515, -0.74, 0.18), (0.03, 0.22, 0.02), M["steel"]))                      # Leiterhalter
    asm.add(plat, "Platforms", mods=[bevel_s])
    lad_hinge = V((-3.545, -0.74, 0.19))
    asm.pivot("SwimLadder", lad_hinge, -Y_AXIS, orient=True, positive="Badeleiter klappt nach hinten ins Wasser",
              limits=(0, 180))
    lad = MeshData()
    for y in (-0.655, -0.825):
        lad.merge(tube(fillet_path([(lad_hinge.x, y, 0.19), (lad_hinge.x, y, 0.70), (lad_hinge.x, -0.74, 0.70)], 0.04, 4),
                       0.011, 10, M["steel"]))
    for z in (0.32, 0.46, 0.60):
        lad.merge(box((lad_hinge.x, -0.74, z), (0.06, 0.17, 0.016), M["black"]))
    asm.add(lad, "SwimLadder", "SwimLadder", detail=1)

    # ------------------------------------------------------------- Scheuerleiste, Bugreling, Klampen, Lichter
    rail = MeshData()
    xs_r = [lerp(g.X_TR, 3.18, i / 60) for i in range(61)]
    stbd = [V((x, -(g.ws(x) + 0.012), g.zs(x) - 0.016)) for x in xs_r]
    path = stbd + [V((3.262, 0.0, g.zs(3.25) - 0.016))] + [V((p.x, -p.y, p.z)) for p in reversed(stbd)]
    rail.merge(tube(path, (0.022, 0.017), 10, M["rubber"], up=Z_AXIS, exponent=3.0))
    ins = [p + V((0.0, math.copysign(0.017, p.y), 0.0)) if abs(p.y) > 1e-6 else p + X_AXIS * 0.017 for p in path]
    rail.merge(tube(ins, 0.006, 6, M["steel"]))
    pts = [V((x, -(g.ws(x) - 0.06), g.zs(x) + 0.30)) for x in (1.45, 1.9, 2.35, 2.75, 3.0)]
    bow_rail = [V((1.40, pts[0].y, g.zs(1.40) + 0.02))] + pts + [V((3.10, 0.0, g.zs(3.1) + 0.30))]
    bow_rail = bow_rail + [V((p.x, -p.y, p.z)) for p in reversed(bow_rail[:-1])]
    rail.merge(tube(fillet_path(bow_rail, 0.08, 4), 0.0125, 10, M["steel"]))
    for x in (1.9, 2.35, 2.75):
        for side in (1, -1):
            y = side * (g.ws(x) - 0.06)
            rail.merge(cylinder(V((x, y, g.zs(x))), V((x, y, g.zs(x) + 0.30)), 0.011, 8, M["steel"]))
            rail.merge(cylinder(V((x, y, g.zs(x))), V((x, y, g.zs(x) + 0.01)), 0.03, 12, M["steel"]))

    def cleat(p, md):
        u, w = X_AXIS, Y_AXIS
        md.merge(box((0, 0, 0), (0.06, 0.03, 0.012), M["steel"], frame_matrix(p + Z_AXIS * 0.006, u, w, Z_AXIS)))
        for d in (-0.022, 0.022):
            md.merge(cylinder(p + u * d, p + u * d + Z_AXIS * 0.032, 0.009, 8, M["steel"]))
        md.merge(tube([p + Z_AXIS * 0.036 - u * 0.075, p + Z_AXIS * 0.036, p + Z_AXIS * 0.036 + u * 0.075],
                      lambda s: 0.006 + 0.006 * math.sin(math.pi * s), 10, M["steel"]))
    for x in (2.55, -2.95):
        for side in (1, -1):
            cleat(V((x, side * (g.ws(x) - 0.09), g.zs(x) + g.crown * 0.3)), rail)
    eye = [V((3.185 + 0.03 * math.cos(a), 0.0, 0.45 + 0.03 * math.sin(a))) for a in (TAU * i / 16 for i in range(17))]
    rail.merge(tube(eye, 0.007, 8, M["steel"], caps=False))                                        # Bugauge
    for side in (1, -1):                                                                             # Ski-Zugösen
        rail.merge(tube([V((g.X_TR - 0.032 * math.cos(a), side * 0.88, 0.58 + 0.032 * math.sin(a)))
                         for a in (math.pi * (i / 10 - 0.5) for i in range(11))], 0.006, 8, M["steel"]))
    rail.merge(cylinder(V((-1.60, -(g.ws(-1.6) - 0.08), g.zs(-1.6))), V((-1.60, -(g.ws(-1.6) - 0.08), g.zs(-1.6) + 0.012)),
                        0.04, 16, M["steel"]))                                                       # Tankstutzen
    hatch = rounded_poly_2d([(2.52, -0.22), (2.88, -0.12), (2.88, 0.12), (2.52, 0.22)], 0.04, 3)
    rail.merge(prism_z(hatch, g.zs(2.7) + g.crown * 0.9, g.zs(2.7) + g.crown + 0.012, M["liner"]))   # Ankerkasten
    asm.add(rail, "Hardware", mods=[bevel_s], detail=1)
    lights = MeshData()                                                                              # Positionslichter
    zl = g.zs(2.98) + g.crown
    lights.merge(box((2.98, 0.0, zl + 0.02), (0.08, 0.10, 0.04), M["black"]))
    for side, mat in ((1, M["red"]), (-1, M["green"])):
        lights.merge(box((2.99, side * 0.026, zl + 0.022), (0.05, 0.045, 0.026), mat, None))
    asm.add(lights, "NavLights")
    pole = MeshData()
    pole_p = V((-2.95, 0.88, g.zs(-2.95)))
    pole.merge(cylinder(pole_p, pole_p + Z_AXIS * 0.80, 0.012, 8, M["steel"]))
    pole.merge(lathe([(0.0, 0.0), (0.03, 0.005), (0.032, 0.05), (0.02, 0.07), (0.0, 0.075)], 16, pole_p + Z_AXIS * 0.80,
                     Z_AXIS, M["white"]))
    stern_light = asm.add(pole, "SternLight", smooth=60)

    # Kennzeichen am Bug (projiziert), Name am Spiegel
    reg = "".join(rng.choice("ABDFHKMNRSTW") for _ in range(2)) + f"-{rng.randint(1000, 9999)}"
    dec = MeshData()
    for side in (1, -1):
        fm = frame_matrix(V((1.75, side * 1.4, 0.62)), -X_AXIS * side, Z_AXIS, Y_AXIS * side)
        dec.merge(project_onto(text_md(reg, 0.11, fm, M["decal"], bold=0.03), hull_md, -Y_AXIS * side, max_edge=0.03))
    fm = frame_matrix(V((g.X_TR - 0.0015, 0.70, 0.44)), -Y_AXIS, Z_AXIS, -X_AXIS)
    dec.merge(text_md("AURORA", 0.09, fm, M["decal"], bold=0.03))
    asm.add(dec, "Decals", detail=2)

    # ------------------------------------------------------------- Konsolen, Armaturen, Windschutzscheibe
    def console(y_in, sgn, mat):
        prof = [(0.45, g.z_sole), (1.12, g.z_sole), (1.12, 0.80), (1.02, 0.93), (0.62, 0.93), (0.47, 0.76)]
        ring_in = [V((x, sgn * y_in, z)) for x, z in prof]
        ring_out = [V((x, sgn * wall_y(x, z), z)) for x, z in prof]
        md = grid_rings([ring_in, ring_out], True, "ngon", "ngon", mat)
        md.recalc = True
        return md
    cons = MeshData()
    cons.merge(console(0.30, -1, M["liner"]))
    cons.merge(console(0.30, 1, M["liner"]))
    p0, p1 = V((0.62, 0.0, 0.93)), V((0.47, 0.0, 0.76))
    d3 = (p1 - p0).normalized()
    n3 = V((d3.z, 0.0, -d3.x))
    if n3.z < 0:
        n3 = -n3
    dash_c = (p0 + p1) / 2
    for y, w, mat in ((-0.48, 0.20, M["screen"]), (-0.74, 0.16, M["screen"]), (-0.62, 0.62, M["panel"])):
        th = 0.004 if mat is M["screen"] else 0.002
        off = 0.004 if mat is M["screen"] else 0.001
        cons.merge(box((0, 0, 0), (0.17 if mat is M["panel"] else 0.11, w, th), mat,
                       frame_matrix(dash_c + Y_AXIS * y + n3 * off, -d3, Y_AXIS, n3)))
    cons.merge(lathe([(0.0, 0.0), (0.045, 0.0), (0.045, 0.02)], 16, V((0.86, -0.62, 0.93)), Z_AXIS, M["black"]))
    cons.merge(lathe([(0.04, 0.0), (0.03, 0.025), (0.0, 0.035)], 16, V((0.86, -0.62, 0.95)), Z_AXIS, M["lens"]))  # Kompass
    cons.merge(tube(fillet_path([(0.60, 0.42, 0.93), (0.66, 0.42, 1.00), (0.94, 0.42, 1.00), (1.0, 0.42, 0.93)], 0.04, 4),
                    0.011, 8, M["steel"]))                                                      # Haltegriff Beifahrer
    cons.merge(box((0, 0, 0), (0.16, 0.30, 0.004), M["panel"], frame_matrix(dash_c + Y_AXIS * 0.62 + n3 * 0.002, -d3,
                                                                                   Y_AXIS, n3)))  # Handschuhfach
    asm.add(cons, "Consoles", mods=[bevel])
    hub = dash_c + Y_AXIS * -0.60 + n3 * 0.22
    asm.pivot("SteeringWheel", hub, n3, orient=True, positive="Lenkrad nach links (gegen den Uhrzeigersinn)",
              limits=(-540, 540))
    asm.add(cylinder(dash_c + Y_AXIS * -0.60, hub - n3 * 0.03, 0.022, 12, M["black"]), "SteeringColumn", detail=1)
    u, w = basis_from(n3)
    sw = lathe([(0.17 + 0.014 * math.cos(TAU * i / 10), 0.014 * math.sin(TAU * i / 10)) for i in range(10)], 40, hub, n3,
               M["steel"], closed_profile=True)
    sw.merge(lathe([(0.0, -0.03), (0.045, -0.025), (0.045, 0.01), (0.0, 0.02)], 20, hub, n3, M["black"]))
    for k in range(3):
        dk = u * math.cos(TAU * k / 3) + w * math.sin(TAU * k / 3)
        sw.merge(cylinder(hub + dk * 0.04, hub + dk * 0.165, 0.008, 8, M["steel"]))
    asm.add(sw, "SteeringWheel", "SteeringWheel", detail=1)
    th_p = V((0.52, -0.86, p0.z + (0.52 - p0.x) / d3.x * d3.z)) + n3 * 0.04      # Gashebel auf dem Armaturenbrett
    asm.pivot("Throttle", th_p, Y_AXIS, orient=True, positive="Gashebel nach vorn (Vorwärtsfahrt)", limits=(-35, 35))
    asm.add(box((0, 0, 0), (0.12, 0.06, 0.04), M["black"], frame_matrix(th_p - n3 * 0.02, -d3, Y_AXIS, n3)),
            "ThrottleBase", detail=1)
    thr = cylinder(th_p, th_p + V((0.0, 0.0, 0.15)), 0.008, 8, M["steel"])
    thr.merge(lathe([(0.0, -0.02), (0.02, 0.0), (0.0, 0.02)], 12, th_p + V((0.0, 0.0, 0.165)), Z_AXIS, M["black"]))
    asm.add(thr, "Throttle", "Throttle", detail=1)

    yb = 1.06                                                                   # Windschutzscheibe (gebogen)

    def ws_point(y, v):
        q = clamp(abs(y) / yb, 0.0, 1.0)
        x = 0.62 + 0.50 * math.sqrt(max(1.0 - q * q, 0.0))
        dxdy = -0.50 * (y / yb ** 2) / math.sqrt(max(1.0 - q * q, 1e-4))
        n = V((1.0, -dxdy, 0.0)).normalized()
        zdeck = g.sheer(x) + g.crown * (1.0 - min(abs(y) / g.ws(x), 1.0) ** 2)
        zb = lerp(0.93, zdeck, smoothstep(0.96, yb, abs(y)))
        hgt = lerp(0.40, 0.22, smoothstep(0.55, yb, abs(y)))
        rake = math.radians(35)
        return V((x, y, zb)) + (-n * math.sin(rake) + Z_AXIS * math.cos(rake)) * (hgt * v)

    def ws_panel(y0, y1, nu):
        return surface_grid(lambda u_, v_: ws_point(lerp(y0, y1, u_), v_), nu, 6, M["glass"])
    glass = MeshData()
    glass.merge(ws_panel(-yb + 0.01, -0.31, 18))
    glass.merge(ws_panel(0.31, yb - 0.01, 18))
    frame = MeshData()
    for y0, y1 in ((-yb + 0.01, -0.31), (0.31, yb - 0.01)):
        frame.merge(tube([ws_point(lerp(y0, y1, i / 24), 1.0) for i in range(25)], 0.012, 8, M["steel"]))
        for y in (y0, y1):
            frame.merge(tube([ws_point(y, v) for v in (0.0, 0.5, 1.0)], 0.009, 8, M["steel"]))
    asm.add(glass, "Windshield", smooth=60)
    asm.add(frame, "WindshieldFrame", detail=1)
    h0, h1 = ws_point(0.29, 0.0), ws_point(0.29, 1.0)
    asm.pivot("WindshieldDoor", (h0 + h1) / 2, (h0 - h1).normalized(), orient=True,
              positive="Durchgangstür schwenkt nach achtern auf", limits=(0, 80))
    asm.add(ws_panel(-0.29, 0.29, 10), "WindshieldDoorGlass", "WindshieldDoor", smooth=60)
    door = MeshData()
    door.merge(tube([ws_point(lerp(-0.29, 0.29, i / 10), 1.0) for i in range(11)], 0.011, 8, M["steel"]))
    for z in (0.25, 0.75):
        door.merge(cylinder(ws_point(0.29, z) - (h1 - h0).normalized() * 0.03, ws_point(0.29, z) + (h1 - h0).normalized() * 0.03,
                            0.012, 10, M["steel"]))                                         # Scharniere
    asm.add(door, "WindshieldDoorFrame", "WindshieldDoor", detail=1)

    # ------------------------------------------------------------- Sitze
    def block(x0, x1, z0, z1, y_a, y_b, mat, n=6, inset=0.008):
        """Polsterblock x0..x1, z0..z1, quer von y_a bis y_b; None = bis an die Cockpitwand, Funktion f(x, z)."""
        rs = []
        for x in (lerp(x0, x1, i / n) for i in range(n + 1)):
            def yy(v, z, sgn):
                if v is None:
                    return sgn * (wall_y(x, z) - inset)
                return v(x, z) if callable(v) else v
            rs.append([V((x, yy(y_a, z0, -1), z0)), V((x, yy(y_b, z0, 1), z0)), V((x, yy(y_b, z1, 1), z1)),
                       V((x, yy(y_a, z1, -1), z1))])
        md = grid_rings(rs, True, "ngon", "ngon", mat)
        md.recalc = True
        return md
    seats = MeshData()
    zs0, zb0 = g.z_sole, g.z_sole_bow
    x0 = g.x_cp0
    seats.merge(block(x0 + 0.01, -1.92, zs0, 0.45, None, None, M["liner"]))                    # Heckbank
    seats.merge(block(x0 + 0.12, -1.90, 0.45, 0.55, None, None, M["vinyl"]))
    seats.merge(block(x0 + 0.01, x0 + 0.12, 0.45, 0.80, None, None, M["vinyl"]))
    seats.merge(block(x0 + 0.01, x0 + 0.125, 0.62, 0.66, None, None, M["accent"], inset=0.004))
    seats.merge(block(-1.92, -0.95, zs0, 0.45, 0.50, None, M["liner"]))                         # Seitenbank Bb
    seats.merge(block(-1.90, -0.97, 0.45, 0.55, 0.52, None, M["vinyl"]))
    back = (lambda x, z: wall_y(x, z) - 0.09)
    seats.merge(block(-1.90, -0.97, 0.56, 0.76, back, None, M["vinyl"]))
    for sgn in (1, -1):                                                                          # Bugsitze
        inner, outer = (0.20, None) if sgn > 0 else (None, -0.20)
        seats.merge(block(g.x_step + 0.03, 2.12, zb0, 0.46, inner, outer, M["liner"]))
        seats.merge(block(g.x_step + 0.05, 2.10, 0.46, 0.56, *((0.22, None) if sgn > 0 else (None, -0.22)), M["vinyl"]))
        bk = (lambda x, z, s_=sgn: s_ * (wall_y(x, z) - 0.09))
        seats.merge(block(g.x_step + 0.05, 2.10, 0.57, 0.80, *((bk, None) if sgn > 0 else (None, bk)), M["vinyl"]))
    for sgn in (1, -1):                                                                          # Fahrer-/Beifahrersitz
        c = V((-0.10, sgn * 0.55, 0.0))
        seats.merge(cylinder(c + Z_AXIS * zs0, c + Z_AXIS * 0.50, 0.04, 16, M["steel"]))
        seats.merge(cylinder(c + Z_AXIS * zs0, c + Z_AXIS * (zs0 + 0.012), 0.16, 24, M["steel"]))
        seats.merge(box(c + V((0.0, 0.0, 0.565)), (0.48, 0.48, 0.11), M["vinyl"]))
        seats.merge(box(c + V((0.20, 0.0, 0.57)), (0.08, 0.485, 0.115), M["accent"]))
        bm = Matrix.Translation(c + V((-0.22, 0.0, 0.62))) @ Matrix.Rotation(math.radians(-12), 4, "Y")
        seats.merge(box((0.0, 0.0, 0.28), (0.10, 0.46, 0.56), M["vinyl"], bm))
        seats.merge(box((0.002, 0.0, 0.45), (0.105, 0.465, 0.06), M["accent"], bm))
    for sgn in (1, -1):                                                                          # Sonnenpolster achtern
        pad = rounded_poly_2d([(-3.03, sgn * 0.47), (-2.62, sgn * 0.47), (-2.62, sgn * 0.82), (-3.03, sgn * 0.82)], 0.04, 3)
        z_pad = g.sheer(-2.8) + g.crown * (1.0 - (0.47 / g.ws(-2.8)) ** 2)
        seats.merge(prism_z(pad, z_pad - 0.005, z_pad + 0.055, M["vinyl"]))
        seats.merge(box((-2.64, sgn * 0.645, z_pad + 0.025), (0.03, 0.352, 0.062), M["accent"]))
    asm.add(seats, "Seats", mods=[bevel])

    # ------------------------------------------------------------- Außenborder: Kippen → Lenken → Propeller
    ob = boat_outboard(g, M)
    brk = MeshData()                                                                             # Spiegelhalterung
    for side in (1, -1):
        brk.merge(box(((g.X_TR + g.tilt.x) / 2 - 0.01, side * 0.14, 0.20), (g.X_TR - g.tilt.x + 0.04, 0.02, 0.32), M["motor"]))
        brk.merge(box((g.X_TR + 0.04, side * 0.14, wz + 0.02), (0.12, 0.02, 0.04), M["motor"]))
    asm.add(brk, "OutboardBracket", mods=[bevel_s])
    asm.pivot("Outboard_Tilt", g.tilt, Y_AXIS, orient=True, positive="Außenborder kippt hoch (Unterwasserteil nach hinten)",
              limits=(0, 65))
    asm.add(ob["tilt"], "Outboard_Tilt", "Outboard_Tilt", mods=[bevel_s])
    asm.pivot("Outboard_Steer", g.steer, -Z_AXIS, orient=True, parent="Outboard_Tilt",
              positive="Boot dreht nach Backbord (links)", limits=(-35, 35))
    asm.add(ob["steer"], "Outboard", "Outboard_Steer", mods=[bevel_s])
    asm.add(ob["decal"], "OutboardDecal", "Outboard_Steer", detail=2)
    asm.pivot("Propeller", g.prop, X_AXIS, parent="Outboard_Steer", motion="spin",
              positive="Vorwärtsschub (rechtsdrehend, von hinten gesehen im Uhrzeigersinn)")
    asm.add(ob["prop"], "Propeller", "Propeller", smooth=45)

    # ------------------------------------------------------------- Sockets & Kollision
    asm.socket("Driver", (-0.10, -0.55, 0.64))
    asm.socket("Passenger", (-0.10, 0.55, 0.64))
    for sgn, k in ((1, "L"), (-1, "R")):
        asm.socket(f"Rear_{k}", (-2.10, sgn * 0.45, 0.58))
        asm.socket(f"Bow_{k}", (1.75, sgn * 0.36, 0.58))
        asm.socket(f"TowEye_{k}", (g.X_TR - 0.03, sgn * 0.88, 0.58), forward=-X_AXIS)
        for x, pos in ((-2.6, "Aft"), (-0.5, "Mid"), (1.6, "Fwd")):                  # Auftriebspunkte (Wasserlinie)
            asm.socket(f"Float_{pos}{k}", (x, sgn * g.hull_y(x, 0.0), 0.0))
    asm.socket("NavLight_Port", (2.99, 0.05, zl + 0.022), forward=Y_AXIS)
    asm.socket("NavLight_Starboard", (2.99, -0.05, zl + 0.022), forward=-Y_AXIS)
    asm.socket("SternLight", pole_p + Z_AXIS * 0.84, forward=Z_AXIS)
    asm.socket("PropWash", g.prop, forward=-X_AXIS, parent="Outboard_Steer")
    asm.socket("BowWave", (2.72, 0.0, 0.0))
    motor_keys = {"Outboard_Tilt", "Outboard_Steer", "Propeller"}
    allm = asm.meshes()
    shell = [o for o in allm if o["pivot"] not in motor_keys and o is not stern_light]
    asm.collide("Hull", shell, lambda p: p.z <= g.sheer(p.x) + 0.05, 96)
    asm.collide("TopFwd", shell, lambda p: p.z >= g.sheer(p.x) - 0.03 and p.x > 0.3, 64)
    asm.collide("TopAft", shell, lambda p: p.z >= g.sheer(p.x) - 0.03 and p.x <= 0.3, 48)
    asm.collide("Outboard", [o for o in allm if o["pivot"] in motor_keys], None, 64)
    asm.hull_exempt.append(([stern_light], "dünner Lichtmast"))
    return asm


# =============================================================================
# 8e) Elektro-SUV (5-Türer, Allrad mit zwei E-Motoren, Einzelradaufhängung, Reifen 275/55 R20)
# =============================================================================

SUV_LIVERIES = {
    # Lack, Rauheit, Klarlack, Metallic-Anteil; Dach und Säulen schwarz ("schwebendes Dach")
    "forest": dict(base=(0.050, 0.068, 0.052), rough=0.40, coat=0.45, metallic=0.45, roof=(0.010, 0.010, 0.012),
                   rim=(0.018, 0.018, 0.020), caliper=(0.05, 0.05, 0.055)),
    "graphite": dict(base=(0.075, 0.08, 0.085), rough=0.32, coat=0.6, metallic=0.65, roof=(0.010, 0.010, 0.012),
                     rim=(0.018, 0.018, 0.020), caliper=(0.55, 0.20, 0.02)),
    "glacier": dict(base=(0.78, 0.79, 0.80), rough=0.28, coat=0.6, metallic=0.15, roof=(0.010, 0.010, 0.012),
                    rim=(0.018, 0.018, 0.020), caliper=(0.05, 0.05, 0.055)),
    "dune": dict(base=(0.40, 0.33, 0.22), rough=0.36, coat=0.45, metallic=0.35, roof=(0.010, 0.010, 0.012),
                 rim=(0.018, 0.018, 0.020), caliper=(0.05, 0.05, 0.055)),
}


def suv_materials(prefix, liv, dirt):
    P = f"M_{prefix}_"
    roof = [(SuvGeo.Z_BELT + 0.012, 9.0, liv["roof"])]          # alles über der Gürtellinie: Hochglanz-Schwarz
    return {
        "paint": mat_paint(P + "Paint", liv["base"], liv["rough"], coat=liv["coat"], metallic=liv["metallic"],
                           livery=roof, dirt=dirt, dirt_z=(0.15, 0.95)),
        "clad": mat_plastic(P + "Cladding", (0.022, 0.022, 0.024), 0.68, dirt, (0.1, 0.9), grain=0.35),
        "gloss": mat_paint(P + "GlossBlack", (0.008, 0.008, 0.009), 0.12, coat=0.9),
        "black": mat_plastic(P + "PlasticBlack", (0.02, 0.02, 0.022), 0.55, dirt * 0.6, (0.1, 0.8), grain=0.12),
        "skid": mat_metal(P + "SkidPlate", (0.60, 0.61, 0.63), 0.32, brushed=True, dirt=dirt * 0.8, dirt_z=(0.2, 0.6)),
        "chrome": mat_metal(P + "Chrome", (0.9, 0.9, 0.92), 0.05),
        "chassis": mat_metal(P + "Chassis", (0.04, 0.04, 0.042), 0.55, dirt=dirt * 1.2, dirt_z=(0.0, 0.9)),
        "forged": mat_metal(P + "ForgedAlu", (0.52, 0.53, 0.55), 0.42, dirt=dirt, dirt_z=(0.0, 0.8)),
        "steel": mat_metal(P + "Steel", (0.30, 0.30, 0.31), 0.35, dirt=dirt * 0.8, dirt_z=(0.0, 0.7)),
        "disc": mat_metal(P + "BrakeDisc", (0.36, 0.35, 0.34), 0.42, brushed=True),
        "caliper": mat_paint(P + "Caliper", liv["caliper"], 0.3, coat=0.5),
        "spring": mat_paint(P + "Spring", (0.02, 0.02, 0.022), 0.4, coat=0.3),
        "rim": mat_paint(P + "Rim", liv["rim"], 0.3, coat=0.5, dirt=dirt * 0.9, dirt_z=(0.0, 0.8)),
        "machined": mat_metal(P + "MachinedAlu", (0.72, 0.73, 0.74), 0.2, brushed=True),
        "tire": mat_rubber(P + "Tire", (0.024, 0.024, 0.024), 0.86, dirt * 1.1, (0.0, 0.85)),
        "rubber": mat_rubber(P + "Rubber", (0.015, 0.015, 0.015), 0.7),
        "hv": mat_plastic(P + "HighVoltage", (0.85, 0.30, 0.02), 0.45),      # Hochvolt-Leitungen (orange)
        "battery": mat_metal(P + "BatteryPack", (0.10, 0.10, 0.105), 0.6, dirt=dirt * 1.2, dirt_z=(0.0, 0.6)),
        "motor": mat_metal(P + "DriveUnit", (0.45, 0.46, 0.47), 0.45, dirt=dirt, dirt_z=(0.0, 0.7)),
        "glass": mat_glass(P + "Glass", (0.42, 0.47, 0.49)),
        "privacy": mat_glass(P + "PrivacyGlass", (0.16, 0.17, 0.18)),
        "lens": mat_glass(P + "Lens", (0.95, 0.95, 0.95)),
        "led": mat_emit(P + "LED", (0.85, 0.92, 1.0), 12.0),
        "lamp": mat_emit(P + "HeadLight", (1.0, 0.97, 0.9), 6.0),
        "tail": mat_emit(P + "TailLight", (1.0, 0.02, 0.01), 5.0, base=(0.5, 0.01, 0.01)),
        "amber": mat_emit(P + "Indicator", (1.0, 0.42, 0.02), 3.0, base=(0.6, 0.28, 0.02)),
        "reverse": mat_paint(P + "ReverseLens", (0.85, 0.85, 0.85), 0.1, coat=1.0),
        "reflector": mat_paint(P + "Reflector", (0.45, 0.02, 0.015), 0.15, coat=1.0),
        "interior": uv_weight(mat_plastic(P + "Interior", (0.05, 0.05, 0.055), 0.75, grain=0.15), 0.5),
        "leather": uv_weight(mat_plastic(P + "Leather", (0.075, 0.068, 0.062), 0.55, grain=0.3), 0.5),
        "fabric": uv_weight(mat_plastic(P + "Headliner", (0.20, 0.20, 0.19), 0.9, grain=0.3), 0.4),
        "floor": uv_weight(mat_rubber(P + "Floor", (0.03, 0.03, 0.03), 0.85), 0.5),
        "panel": uv_weight(mat_plastic(P + "Dash", (0.025, 0.025, 0.028), 0.5), 0.5),
        "trim": uv_weight(mat_metal(P + "DashTrim", (0.55, 0.56, 0.57), 0.3, brushed=True), 0.5),
        "screen": uv_weight(mat_emit(P + "Screen", (0.10, 0.24, 0.38), 1.6, base=(0.01, 0.01, 0.012)), 0.8),
        "ambient": mat_emit(P + "Ambient", (0.10, 0.55, 1.0), 2.0),
        "plate": mat_paint(P + "Plate", (0.82, 0.82, 0.80), 0.35),
        "decal": uv_weight(mat_paint(P + "Decal", (0.02, 0.02, 0.022), 0.3), 2.0),
        "mirror": mat_metal(P + "MirrorGlass", (0.85, 0.87, 0.9), 0.02),
    }


def rrect2_ring(xf, xr, hy, rf, rr, nx, ny, cs, z):
    """Abgerundetes Rechteck auf Höhe z zwischen ``xr`` (hinten) und ``xf`` (vorne), halbe Breite ``hy``, mit
    eigenem Eckradius vorne (``rf``) und hinten (``rr``). CCW von oben, Start vorne rechts; gerade Seiten in
    ``nx``/``ny`` Stücke geteilt, Ecken mit ``cs`` Segmenten – gleiche Punktzahl auf jeder Höhe (für Lofts)."""
    rf = min(rf, (xf - xr) / 2 - 1e-4, hy - 1e-4)
    rr = min(rr, (xf - xr) / 2 - 1e-4, hy - 1e-4)
    corners = [(xf - rf, hy - rf, rf, 0.0), (xr + rr, hy - rr, rr, 90.0), (xr + rr, -hy + rr, rr, 180.0),
               (xf - rf, -hy + rf, rf, 270.0)]
    sides = [((xf, -hy + rf), (xf, hy - rf), nx), ((xf - rf, hy), (xr + rr, hy), ny),
             ((xr, hy - rr), (xr, -hy + rr), nx), ((xr + rr, -hy), (xf - rf, -hy), ny)]
    pts = []
    for (a, b, n), (ccx, ccy, r, a0) in zip(sides, corners):
        pts += [V((lerp(a[0], b[0], i / n), lerp(a[1], b[1], i / n), z)) for i in range(n)]
        pts += [V((ccx + r * math.cos(math.radians(a0 + 90.0 * s / cs)),
                   ccy + r * math.sin(math.radians(a0 + 90.0 * s / cs)), z)) for s in range(cs)]
    return pts


def offset_open(pts, d):
    """Offene 2D-Polylinie um d versetzen (d > 0: nach rechts zur Laufrichtung, bei CCW-Umlauf = außen)."""
    pts = [V((p[0], p[1])) for p in pts]
    n, out = len(pts), []
    for i in range(n):
        e0, e1 = pts[i] - pts[max(i - 1, 0)], pts[min(i + 1, n - 1)] - pts[i]
        e0 = e0 if e0.length > 1e-9 else e1
        e1 = e1 if e1.length > 1e-9 else e0
        n0, n1 = V((e0.y, -e0.x)).normalized(), V((e1.y, -e1.x)).normalized()
        nm = (n0 + n1).normalized()
        out.append(pts[i] + nm * (d / max(nm.dot(n0), 0.35)))
    return out


def orient_faces(md, inside):
    """Alle Faces so drehen, dass ihre Normalen von ``inside(p)`` (Punkt → Vector zur Innenseite) wegzeigen."""
    for i, f in enumerate(md.faces):
        p = [md.verts[k] for k in f]
        n = V()
        for a in range(len(p)):
            n += p[a - 1].cross(p[a])
        c = sum(p, V()) / len(p)
        if n.dot(inside(c)) > 0:
            md.faces[i] = tuple(reversed(f))
            if md.uvs[i]:
                md.uvs[i] = tuple(reversed(md.uvs[i]))
    return md


class SuvGeo:
    """Maße eines generischen Elektro-SUV der 4,8-m-Klasse: Karosserie 4,78 m (≈ 4,87 m mit Stoßfängern und
    Unterfahrschutz), Breite 1,90 m (2,03 m über die Kotflügelverbreiterungen), Höhe 1,79 m (1,92 m mit
    Dachplattform), Radstand 2,90 m, Spur 1,66 m, Reifen 275/55 R20 (Ø 0,81 m), Bodenfreiheit ≈ 0,24 m.
    Flache Haube, stark geneigte Frontscheibe, Gürtellinie auf 1,18 m."""

    Z_BELT = 1.18
    # Unterbau (Loft entlang X): Superellipsen-Spanten mit straffer Schulter (et) und eingezogenem Schweller (eb)
    LOWER = [
        dict(x=-2.40, hw=0.875, zt=1.150, zb=0.640, zm=0.90, et=10.0, eb=6.0),
        dict(x=-2.392, hw=0.905, zt=1.165, zb=0.615, zm=0.90, et=10.0, eb=6.0),
        dict(x=-2.37, hw=0.930, zt=1.175, zb=0.595, zm=0.90, et=10.0, eb=6.0),
        dict(x=-2.32, hw=0.945, zt=1.18, zb=0.580, zm=0.90, et=10.0, eb=6.0),
        dict(x=-2.10, hw=0.95, zt=1.18, zb=0.565, zm=0.90, et=10.0, eb=6.0),
        dict(x=-1.40, hw=0.95, zt=1.18, zb=0.52, zm=0.90, et=10.0, eb=6.0),
        dict(x=-0.80, hw=0.95, zt=1.18, zb=0.50, zm=0.90, et=10.0, eb=6.0),
        dict(x=0.90, hw=0.95, zt=1.18, zb=0.50, zm=0.90, et=10.0, eb=6.0),
        dict(x=1.50, hw=0.95, zt=1.17, zb=0.52, zm=0.90, et=10.0, eb=6.0),
        dict(x=2.10, hw=0.95, zt=1.13, zb=0.55, zm=0.90, et=10.0, eb=6.0),
        dict(x=2.30, hw=0.945, zt=1.115, zb=0.565, zm=0.90, et=10.0, eb=6.0),
        dict(x=2.35, hw=0.930, zt=1.10, zb=0.575, zm=0.90, et=10.0, eb=6.0),
        dict(x=2.372, hw=0.905, zt=1.085, zb=0.590, zm=0.90, et=10.0, eb=6.0),
        dict(x=2.38, hw=0.875, zt=1.065, zb=0.610, zm=0.90, et=10.0, eb=6.0),
    ]

    def __init__(self):
        self.fa, self.ra = V((1.45, 0.0, 0.405)), V((-1.45, 0.0, 0.405))     # Achsmitten (Radmitte auf 0,405 m)
        self.track = 1.66
        self.R = 0.405                                  # Reifenradius 275/55 R20
        self.x_front, self.x_rear = 2.38, -2.40
        self.hw = 0.95
        self.z_belt, self.z_roof = self.Z_BELT, 1.79
        self.x_ws0, self.x_ws1 = 1.20, 0.42             # Frontscheibe: Unterkante / Dachkante (≈ 55° zur Senkrechten)
        self.hw_sh, self.hw_roof = 0.885, 0.775         # halbe Kabinenbreite an Gürtellinie / Dach (Tumblehome)
        self.r_roof = 0.075
        self.rf, self.rr = 0.12, 0.15                   # Eckradien der Kabine in der Draufsicht (A- / D-Säule)
        self.bow = 0.04                                 # Wölbung der Frontscheibe in der Draufsicht
        self.shell_t = 0.02
        self._p = {k: Pchip([s["x"] for s in self.LOWER], [s[k] for s in self.LOWER])
                   for k in ("hw", "zt", "zb", "zm", "et", "eb")}

    def wheel(self, key):
        a = self.fa if key[0] == "F" else self.ra
        return V((a.x, (1.0 if key[1] == "L" else -1.0) * self.track / 2, a.z))

    def zt(self, x):
        return self._p["zt"](x)

    def lower_y(self, x, z):
        """Halbe Breite des Unterbaus an (x, z) – exakt die Superellipse des Lofts."""
        p = self._p
        hw, zt, zb, zm = p["hw"](x), p["zt"](x), p["zb"](x), p["zm"](x)
        if z >= zm:
            t, e = (z - zm) / max(zt - zm, 1e-6), p["et"](x)
        else:
            t, e = (zm - z) / max(zm - zb, 1e-6), p["eb"](x)
        t = clamp(t)
        return hw * (1.0 - t ** e) ** (1.0 / e)

    def t_cab(self, z):
        return clamp((z - self.z_belt) / (self.z_roof - self.r_roof - self.z_belt))

    def ws_x(self, z):
        """X der Frontscheibe (Außenhaut) in der Mitte auf Höhe z."""
        return lerp(self.x_ws0, self.x_ws1, self.t_cab(z))

    def ws_point(self, y, z):
        """Punkt auf der (in der Draufsicht gewölbten) Frontscheibe."""
        return V((self.ws_x(z) - self.bow * (y / self.cab_hw(z)) ** 2, y, z))

    def cab_hw(self, z):
        return lerp(self.hw_sh, self.hw_roof, self.t_cab(z))

    def cab_xr(self, z):
        """Kabinen-Heck: Heckscheibe ≈ 29° nach vorn geneigt, unten 4 mm innerhalb des Unterbaus."""
        return self.x_rear + 0.004 + 0.30 * self.t_cab(z)

    def lower_xs(self):
        ends = [0.0, 0.004, 0.012, 0.025, 0.045, 0.075, 0.12]
        mid = [lerp(-2.22, 2.20, i / 44) for i in range(45)]
        return [self.x_rear + d for d in ends] + mid + [self.x_front - d for d in reversed(ends)]

    def _cab_ring(self, z, d=0.0, z_ref=None):
        zr = z if z_ref is None else z_ref
        hw, xf, xr = self.cab_hw(zr) - d, self.ws_x(zr) - d, self.cab_xr(zr) + d
        ring = rrect2_ring(xf, xr, hw, self.rf, self.rr, 18, 32, 5, z)
        out = []
        for p in ring:                       # Frontscheibe in der Draufsicht wölben (Seitenwände bleiben eben)
            w = smoothstep(xf - 0.45, xf - 0.05, p.x)
            out.append(V((p.x - self.bow * (p.y / hw) ** 2 * w, p.y, p.z)))
        return out

    def cabin_md(self, mat):
        """Kabine als Loft waagerechter Schnitte: Scheibenneigung, Tumblehome und gerundete Dachkante."""
        top = self.z_roof - self.r_roof
        zs = [self.z_belt - 0.08, self.z_belt - 0.02] + [lerp(self.z_belt + 0.02, top, i / 7) for i in range(8)]
        rings = [self._cab_ring(z) for z in zs]
        for k in range(1, 6):
            phi = math.pi / 2 * k / 5
            rings.append(self._cab_ring(top + self.r_roof * math.sin(phi), self.r_roof * (1 - math.cos(phi)), top))
        crown = V(((self.ws_x(top) + self.cab_xr(top)) / 2, 0.0, self.z_roof + 0.012))     # leicht gewölbtes Dach
        return grid_rings(rings, True, "ngon", crown, mat)

    def arch_outline(self, a):
        """Trapezförmiger Radlauf (Seitenansicht, offen nach unten) – Federweg ±8 cm und Lenkeinschlag frei."""
        pts = [(a.x + 0.50, 0.44), (a.x + 0.49, a.z + 0.20), (a.x + 0.30, a.z + 0.52), (a.x - 0.30, a.z + 0.52),
               (a.x - 0.49, a.z + 0.20), (a.x - 0.50, 0.44)]
        return [V((p.x, p.y)) for p in fillet_path([V((x, z, 0.0)) for x, z in pts], 0.12, 6)]


def suv_bumper(x_in, end, z0, z1, mat):
    """Stoßfänger als Stapel gerundeter Rechtecke: vorne (end=+1) bzw. hinten (end=−1) bauchig gerundete
    Ober- und Unterkante, in der Draufsicht große Eckradien; innen (x_in) endet er am Radlauf."""
    rings = []
    zs = [0.0, 0.03, 0.10, 0.25, 0.5, 0.75, 0.88, 0.95, 1.0]
    for t in zs:
        z = lerp(z0, z1, t)
        bulge = math.sqrt(max(0.0, 1.0 - (2.0 * t - 1.0) ** 8))           # flache Front, gerundete Kanten
        x_out = 2.392 + 0.042 * bulge if end > 0 else -2.405 - 0.042 * bulge
        hy = 0.928 + 0.04 * bulge
        xf, xr = (x_out, x_in) if end > 0 else (x_in, x_out)
        rf, rr = (0.17, 0.004) if end > 0 else (0.004, 0.17)
        rings.append(rrect2_ring(xf, xr, hy, rf, rr, 16, 6, 6, z))
    return grid_rings(rings, True, "ngon", "ngon", mat)


def suv_wheel(M):
    """Rad 275/55 R20: All-Terrain-Reifen mit Profilblöcken und Flankenstollen, zweifarbige 20"-Leichtmetallfelge
    mit fünf Doppelspeichen (glanzgedrehte Front), innenbelüftete Bremsscheibe. Achse +Y, Außenseite +Y,
    Radmitte im Ursprung. Gibt dict(Name → MeshData) zurück."""
    half = [(0.405, 0.0), (0.4047, 0.03), (0.4038, 0.06), (0.4022, 0.088), (0.399, 0.106), (0.393, 0.119),
            (0.383, 0.128), (0.367, 0.1345), (0.34, 0.1375), (0.31, 0.1355), (0.285, 0.128), (0.265, 0.117),
            (0.254, 0.110)]
    tire = lathe([(r, -h) for r, h in reversed(half[1:])] + half, 80, V((0, 0, 0)), Y_AXIS, M["tire"])
    cum = [0.0]
    for a, b in zip(half, half[1:]):
        cum.append(cum[-1] + math.dist(a, b))

    def at(s):
        s = clamp(s, 0.0, cum[-1] - 1e-6)
        i = bisect.bisect_right(cum, s) - 1
        (r0, h0), (r1, h1) = half[i], half[i + 1]
        seg = cum[i + 1] - cum[i]
        t = (s - cum[i]) / seg
        return lerp(r0, r1, t), lerp(h0, h1, t), (r1 - r0) / seg, (h1 - h0) / seg

    tread = MeshData()

    def block(s, side, theta, L, W, H, skew=0.0):
        r, h, dr, dh = at(s)
        rad = V((math.sin(theta), 0.0, math.cos(theta)))
        tan = V((math.cos(theta), 0.0, -math.sin(theta)))
        p = rad * r + Y_AXIS * (side * h)
        n = (rad * dh - Y_AXIS * (side * dr)).normalized()
        if skew:
            tan = Quaternion(n, skew * side) @ tan
        lat = n.cross(tan).normalized()
        tan = lat.cross(n).normalized()
        corners = ((-1, -1), (1, -1), (1, 1), (-1, 1))
        base = [p - n * 0.004 + tan * (sx * L / 2) + lat * (sy * W / 2) for sx, sy in corners]
        top = [p + n * H + tan * (sx * L * 0.44) + lat * (sy * W * 0.44) for sx, sy in corners]
        tread.add(base + top, [(4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)], None, M["tire"])

    pitches = 34
    hp = TAU / pitches / 2
    for k in range(pitches):
        th = TAU * k / pitches
        for side in (1, -1):
            off = 0.0 if side > 0 else hp
            block(0.022, side, th + off, 0.054, 0.040, 0.011, 0.30)           # Mittelblöcke, schräg
            block(0.064, side, th + off + hp, 0.056, 0.038, 0.011, -0.18)     # Zwischenblöcke
            block(0.104, side, th + off, 0.050, 0.040, 0.0105)                # Schulterblöcke
            if k % 2 == 0:
                block(0.134, side, th + off + hp, 0.040, 0.026, 0.006)        # Flankenstollen

    # Felgenbett (geschlossenes Profil) mit glanzgedrehtem Außenhorn
    rim_prof = [(0.256, -0.116), (0.256, -0.104), (0.247, -0.101), (0.242, -0.088), (0.238, -0.030), (0.232, -0.020),
                (0.232, 0.020), (0.238, 0.030), (0.242, 0.088), (0.247, 0.101), (0.258, 0.104), (0.262, 0.110),
                (0.258, 0.116), (0.246, 0.116), (0.236, 0.108), (0.228, 0.094), (0.225, 0.0), (0.228, -0.090),
                (0.238, -0.104), (0.248, -0.116)]
    rim = lathe(rim_prof, 72, V((0, 0, 0)), Y_AXIS, M["rim"], closed_profile=True)
    rim.merge(lathe([(0.236, 0.1155), (0.259, 0.1155), (0.259, 0.1185), (0.236, 0.1185)], 72, V((0, 0, 0)), Y_AXIS,
                    M["machined"], closed_profile=True))
    # Nabe: Schüssel, Nabendeckel mit Emblem, fünf Radschrauben (Lochkreis 127 mm)
    rim.merge(lathe([(0.0, 0.034), (0.092, 0.034), (0.092, 0.050), (0.080, 0.066), (0.050, 0.070), (0.0, 0.072)], 40,
                    V((0, 0, 0)), Y_AXIS, M["rim"]))
    rim.merge(lathe([(0.0, 0.0725), (0.030, 0.0725), (0.031, 0.076), (0.0, 0.079)], 24, V((0, 0, 0)), Y_AXIS,
                    M["gloss"]))
    for i in range(5):
        p = V((math.sin(TAU * i / 5 + 0.3), 0.0, math.cos(TAU * i / 5 + 0.3))) * 0.0635
        rim.merge(cylinder(p + Y_AXIS * 0.066, p + Y_AXIS * 0.080, 0.0105, 6, M["chrome"]))

    def spoke(theta):
        """Eine Speiche: Querschnitt mit gefasten Vorderkanten, konkave Front (Nabe tiefer als das Horn)."""
        rings = []
        for k in range(9):
            s = k / 8
            r = lerp(0.082, 0.234, s)
            w = lerp(0.050, 0.034, s)
            d = lerp(0.036, 0.024, s)
            yf = 0.066 + 0.046 * s ** 0.8
            c = 0.0075
            sec = [(-w / 2, yf - d), (w / 2, yf - d), (w / 2, yf - c), (w / 2 - c, yf), (-w / 2 + c, yf), (-w / 2, yf - c)]
            ang = theta
            rad = V((math.sin(ang), 0.0, math.cos(ang)))
            tan = V((math.cos(ang), 0.0, -math.sin(ang)))
            rings.append([rad * r + tan * t + Y_AXIS * y for t, y in sec])
        md = grid_rings(rings, True, "ngon", "ngon", M["rim"])
        md.recalc = True
        cols = 6
        for k in range(8):                                    # Frontfläche glanzgedreht
            md.mats[k * cols + 3] = M["machined"]
        return md
    for i in range(5):
        base = TAU * i / 5
        for sgn in (1, -1):
            sp = spoke(0.0)
            # Doppelspeiche: V-förmig, an der Nabe enger zusammen als am Horn
            for j, v in enumerate(sp.verts):
                r = math.hypot(v.x, v.z)
                ang = math.atan2(v.x, v.z) + base + sgn * lerp(math.radians(4.5), math.radians(9.0),
                                                               clamp((r - 0.082) / 0.152))
                sp.verts[j] = V((math.sin(ang) * r, v.y, math.cos(ang) * r))
            rim.merge(sp)
    brake = lathe([(0.112, -0.040), (0.190, -0.040), (0.190, -0.006), (0.112, -0.006)], 64, V((0, 0, 0)), Y_AXIS,
                  M["disc"], closed_profile=True)
    brake.merge(lathe([(0.0, 0.034), (0.085, 0.034), (0.088, 0.010), (0.115, -0.006), (0.115, -0.012),
                       (0.080, -0.012), (0.0, -0.012)], 40, V((0, 0, 0)), Y_AXIS, M["steel"]))
    return {"tire": tire, "tread": tread, "rim": rim, "brake": brake}


def suv_frunk(g, M):
    """Vorderer Kofferraum (Frunk) und Technik unter der Haube: Wanne, Innenkotflügel, Querträger, Wärmetauscher,
    Klimamodul, 12-V-Batterie, Ausgleichsbehälter, Bremsmodul, Waschwasser. Gibt (Hauptteile, Kleinteile) zurück."""
    fr, small = MeshData(), MeshData()
    z_shelf = g.fa.z + 0.52 + 0.02                                              # über dem Radlauf
    fr.merge(box((1.24, 0.0, 0.795), (0.03, 0.88, 0.31), M["interior"]))        # Spritzwand (zwischen den Radhäusern)
    fr.merge(box((1.24, 0.0, 1.075), (0.03, 1.70, 0.25), M["interior"]))
    fr.merge(box((1.31, 0.0, 1.085), (0.12, 1.50, 0.07), M["black"]))           # Klimamodul / Wasserkasten
    for side in (1, -1):                                                        # Innenkotflügel
        fr.merge(box((1.66, side * 0.69, z_shelf - 0.0075), (1.06, 0.50, 0.015), M["interior"]))
        fr.merge(box((1.66, side * 0.45, (0.70 + z_shelf) / 2), (1.06, 0.012, z_shelf - 0.70), M["interior"]))
        fr.merge(lathe([(0.0, 0.0), (0.085, 0.0), (0.075, 0.035), (0.0, 0.04)], 24,
                       V((g.fa.x - 0.09, side * 0.52, z_shelf)), Z_AXIS, M["black"]))           # Domlager
    # Frunk-Wanne (offen nach oben), Boden mit Rippen
    x0, x1, hy, zb, zt = 1.42, 2.16, 0.42, 0.70, 1.06
    fr.merge(box(((x0 + x1) / 2, 0.0, zb), (x1 - x0, 2 * hy, 0.012), M["black"]))
    for x in (x0, x1):
        fr.merge(box((x, 0.0, (zb + zt) / 2), (0.012, 2 * hy, zt - zb), M["black"]))
    for side in (1, -1):
        fr.merge(box(((x0 + x1) / 2, side * hy, (zb + zt) / 2), (x1 - x0, 0.012, zt - zb), M["black"]))
    for k in range(5):
        small.merge(box((lerp(x0 + 0.1, x1 - 0.1, k / 4), 0.0, zb + 0.009), (0.02, 2 * hy - 0.06, 0.006), M["rubber"]))
    for x in (x0, x1):                                                          # umlaufende Dichtung
        small.merge(box((x, 0.0, zt + 0.004), (0.02, 2 * hy + 0.02, 0.008), M["rubber"]))
    for side in (1, -1):
        small.merge(box(((x0 + x1) / 2, side * hy, zt + 0.004), (x1 - x0 + 0.02, 0.02, 0.008), M["rubber"]))
    # Querträger vorn, Wärmetauscher hinter dem Grill
    fr.merge(box((2.26, 0.0, 0.92), (0.08, 1.60, 0.10), M["chassis"]))
    fr.merge(box((2.30, 0.0, 0.72), (0.05, 1.10, 0.28), M["chassis"]))
    # 12-V-Batterie, Bremsmodul (Fahrerseite), Ausgleichsbehälter, Waschwasser, Sicherungskasten
    fr.merge(box((1.48, -0.66, z_shelf + 0.08), (0.24, 0.18, 0.15), M["black"]))
    small.merge(box((1.48, -0.66, z_shelf + 0.16), (0.24, 0.02, 0.012), M["hv"]))           # Warnmarkierung
    fr.merge(cylinder(V((1.255, 0.58, 1.035)), V((1.37, 0.58, 1.035)), 0.08, 24, M["black"]))
    fr.merge(box((1.41, 0.58, 1.07), (0.08, 0.14, 0.06), M["reverse"]))
    small.merge(cylinder(V((1.41, 0.58, 1.10)), V((1.41, 0.58, 1.112)), 0.022, 12, M["amber"]))
    for x, y, h in ((1.66, -0.68, 0.14), (1.88, -0.68, 0.11)):
        fr.merge(box((x, y, z_shelf + h / 2), (0.16, 0.12, h), M["reverse"]))
        small.merge(cylinder(V((x, y, z_shelf + h)), V((x, y, z_shelf + h + 0.016)), 0.022, 12, M["black"]))
    fr.merge(box((1.80, 0.68, z_shelf + 0.07), (0.14, 0.16, 0.14), M["black"]))
    small.merge(cylinder(V((1.80, 0.68, z_shelf + 0.14)), V((1.80, 0.68, z_shelf + 0.156)), 0.024, 12, M["chrome"]))
    fr.merge(box((1.60, 0.68, z_shelf + 0.05), (0.18, 0.14, 0.10), M["black"]))
    # Hochvolt-Leitung zum Klimakompressor/PTC-Heizer
    small.merge(tube(spline([(1.44, -0.47, 0.75), (1.52, -0.50, 0.86), (1.50, -0.55, 0.93), (1.42, -0.62, 0.96)], 5),
                     0.014, 8, M["hv"]))
    return fr, small


def suv_corner(g, M, front, side):
    """Doppelquerlenker einer Ecke. Liefert die Geometrie je Teil und die Lage der Pivots (siehe build_suv)."""
    a = g.fa if front else g.ra
    s = side
    caster = 0.02 if front else 0.0
    P = dict(
        lbj=V((a.x, s * 0.71, 0.215)), ubj=V((a.x - caster, s * 0.675, 0.60)),
        la=[V((a.x + 0.20, s * 0.32, 0.25)), V((a.x - 0.20, s * 0.32, 0.25))],
        ua=[V((a.x + 0.16, s * 0.42, 0.635)), V((a.x - 0.30, s * 0.42, 0.635))],
        co_lo=V((a.x - 0.09, s * 0.57, 0.255)), co_up=V((a.x - 0.09, s * 0.52, 0.80)),
        cv_in=V((a.x, s * 0.24, a.z)), hub=V((a.x, s * g.track / 2, a.z)),
    )
    P["la_c"] = (P["la"][0] + P["la"][1]) / 2
    parts = {}
    # Querlenker unten (geschmiedetes Alu): zwei Arme, Lagerbuchsen, Kugelgelenk-Aufnahme, Federbeinkonsole
    la = MeshData()
    for b in P["la"]:
        la.merge(tube([b, P["lbj"]], (0.017, 0.026), 10, M["forged"], up=Z_AXIS, exponent=3.0))
        la.merge(cylinder(b - X_AXIS * 0.035, b + X_AXIS * 0.035, 0.03, 14, M["rubber"]))
        la.merge(cylinder(b - X_AXIS * 0.04, b + X_AXIS * 0.04, 0.011, 8, M["steel"]))
    la.merge(cylinder(P["lbj"] - Z_AXIS * 0.02, P["lbj"] + Z_AXIS * 0.02, 0.034, 14, M["forged"]))
    la.merge(box(P["co_lo"] - V((0.0, 0.0, 0.025)), (0.07, 0.06, 0.05), M["forged"]))
    la.merge(box(((P["co_lo"].x + a.x) / 2 - 0.01, P["co_lo"].y, 0.232), (0.12, 0.05, 0.03), M["forged"]))
    parts["ArmLower"] = la
    # Querlenker oben (aufgelöst in zwei Streben, die beide auf das obere Kugelgelenk zielen – zusammen
    # verhalten sie sich wie ein starrer Dreieckslenker, der um seine Lagerachse schwenkt)
    for b, key in zip(P["ua"], ("ArmUpperFwd", "ArmUpperAft")):
        ua = tube([b, P["ubj"] + (b - P["ubj"]).normalized() * 0.02], (0.013, 0.02), 10, M["forged"], up=Z_AXIS,
                  exponent=3.0)
        ua.merge(cylinder(b - X_AXIS * 0.022, b + X_AXIS * 0.022, 0.022, 12, M["rubber"]))
        parts[key] = ua
    # Kugelgelenke: unten am Radträger (Federweg-Pivot), oben mit Manschette auf den oberen Lenker ausgerichtet
    parts["LowerJoint"] = lathe([(0.0, -0.03), (0.022, -0.025), (0.026, 0.0), (0.02, 0.022), (0.0, 0.028)], 14,
                                P["lbj"], Z_AXIS, M["chassis"])
    d_uj = (P["ua"][0] - P["ubj"]).normalized()
    parts["BallJoint"] = lathe([(0.0, -0.012), (0.024, -0.008), (0.026, 0.018), (0.016, 0.045), (0.0, 0.05)], 14,
                               P["ubj"], d_uj, M["rubber"])
    # Radträger (vorn lenkbar): Säule zwischen den Gelenken, Nabe, Spurstangenhebel, Bremssattel
    up = MeshData()
    up.merge(tube([P["lbj"] + Z_AXIS * 0.02, P["hub"] - Y_AXIS * s * 0.13 - Z_AXIS * 0.05,
                   P["hub"] - Y_AXIS * s * 0.13 + Z_AXIS * 0.06, P["ubj"] - Z_AXIS * 0.01], (0.03, 0.022), 10,
                  M["chassis"], up=X_AXIS, exponent=3.0))
    up.merge(cylinder(P["hub"] - Y_AXIS * s * 0.15, P["hub"] - Y_AXIS * s * 0.09, 0.07, 20, M["chassis"]))
    up.merge(cylinder(P["hub"] - Y_AXIS * s * 0.09, P["hub"] - Y_AXIS * s * 0.05, 0.05, 20, M["steel"]))
    sa = 1.0 if front else -1.0                     # Spurstangenhebel (vorn Lenkung, hinten Hinterachslenkung)
    up.merge(tube([P["hub"] - Y_AXIS * s * 0.13 - Z_AXIS * 0.02, P["hub"] - Y_AXIS * s * 0.15 + V((sa * 0.16, 0, -0.05))],
                  0.016, 8, M["chassis"]))
    cal_c = P["hub"] + V((-0.15, 0.0, 0.10))
    rad = (cal_c - P["hub"]).normalized()
    tng = Y_AXIS.cross(rad).normalized()
    cm = frame_matrix(cal_c - Y_AXIS * s * 0.023, tng, Y_AXIS, rad)
    up.merge(box((0, 0, 0), (0.16, 0.075, 0.07), M["caliper"], cm))
    up.merge(box((0, s * 0.039, 0.0), (0.12, 0.004, 0.04), M["chrome"], cm))           # Schriftfeld
    parts["Upright"] = up
    # Federbein (Coilover): Hälfte am Querlenker, Hälfte am Aufbau – visieren sich per Aim gegenseitig an
    d = (P["co_up"] - P["co_lo"]).normalized()
    cl, cu = MeshData(), MeshData()
    cl.merge(cylinder(P["co_lo"] - X_AXIS * 0.025, P["co_lo"] + X_AXIS * 0.025, 0.018, 12, M["steel"]))
    cl.merge(cylinder(P["co_lo"] + d * 0.02, P["co_lo"] + d * 0.26, 0.025, 16, M["chassis"]))
    cl.merge(cylinder(P["co_lo"] + d * 0.12, P["co_lo"] + d * 0.132, 0.058, 24, M["forged"]))
    cl.merge(tube(helix(P["co_lo"] + d * 0.132, d, 0.047, 0.36, 5.5, 16), 0.0085, 8, M["spring"], caps=False))
    cu.merge(cylinder(P["co_up"], P["co_up"] - d * 0.03, 0.062, 24, M["chassis"]))
    cu.merge(cylinder(P["co_up"] - d * 0.03, P["co_up"] - d * 0.30, 0.011, 10, M["chrome"]))
    cu.merge(lathe([(0.0, 0.0), (0.03, 0.0), (0.034, 0.10), (0.0, 0.10)], 16, P["co_up"] - d * 0.13, -d, M["rubber"]))
    parts["Coilover"], parts["CoiloverMount"] = cl, cu
    # Antriebswelle mit Gleichlaufgelenken und Faltenbälgen
    ds = MeshData()
    o = P["cv_in"]
    ax_ = Y_AXIS * s
    boot = [(0.0, 0.0), (0.045, 0.0), (0.048, 0.02), (0.038, 0.035), (0.045, 0.05), (0.034, 0.065), (0.04, 0.08),
            (0.024, 0.10), (0.018, 0.11), (0.0, 0.11)]
    ds.merge(lathe(boot, 16, o, ax_, M["rubber"]))
    ds.merge(cylinder(o + ax_ * 0.10, o + ax_ * 0.37, 0.019, 12, M["steel"]))
    ds.merge(lathe([(r, 0.11 - h) for r, h in reversed(boot)], 16, o + ax_ * 0.36, ax_, M["rubber"]))
    parts["Driveshaft"] = ds
    return P, parts


def build_suv(cfg):
    g = SuvGeo()
    liv = SUV_LIVERIES.get(cfg.suv_color, SUV_LIVERIES["forest"])
    name = cfg.suv_name
    M = suv_materials(name, liv, cfg.dirt)
    asm = Assembly(name)
    asm.ao_distance = 0.35
    bevel_s = ("bevel", dict(width=0.0015, segments=1, angle=40))
    bevel = ("bevel", dict(width=0.004, segments=2, angle=35))
    soft = ("bevel", dict(width=0.012, segments=3, angle=30))
    st, gap = g.shell_t, 0.004
    rng = random.Random(cfg.seed)
    reg = "EV " + "".join(rng.choice("ABCDEFGHKLMNPRSTUVWXYZ") for _ in range(2)) + f" {rng.randint(100, 999)}"

    def plate(x, z, fwd):
        md = box((x + fwd * 0.003, 0.0, z), (0.006, 0.52, 0.11), M["plate"])
        fm = frame_matrix(V((x + fwd * 0.0062, 0.0, z)), Y_AXIS * fwd, Z_AXIS, X_AXIS * fwd)
        return md.merge(text_md(reg, 0.075, fm, M["decal"], bold=0.03))
    k_ws = (g.x_ws0 - g.x_ws1) / (g.z_roof - g.r_roof - g.z_belt)        # Scheibenneigung −dx/dz
    n_ws = V((1.0, 0.0, k_ws)).normalized()

    # ------------------------------------------------------------- Karosserie: Unterbau ∪ Kabine → Hülle
    body = loft_stations(g.LOWER, g.lower_xs(), 56, "ngon", "ngon", M["paint"]).to_object(f"{name}_Body", smooth=50)
    asm.coll.objects.link(body)
    boolean(body, [g.cabin_md(M["paint"]).to_object("_cab", smooth=50)], "UNION")
    set_smoothing(body, 50)
    body.data.materials.append(M["interior"])
    add_modifier(body, "solidify", thickness=st, material_offset=1)
    apply_modifiers(body)
    src = body.data.copy()

    def pillar(z, back):
        """X-Position parallel zur A-Säule (``back`` hinter der Scheibenmitte)."""
        return g.ws_x(z) - back

    def flat(points3d, mat, flip=False):
        vs = list(reversed(points3d)) if flip else list(points3d)
        return MeshData().add(vs, [tuple(range(len(vs)))], None, mat)

    def side_glass(outline, side, mat=None):
        """Scheibe in der (geneigten) Kabinenwand, mittig in der Wandstärke; Rand verschwindet in der Wand."""
        pts = ensure_ccw(offset_poly(outline, 0.025))
        return flat([V((p.x, side * (g.cab_hw(p.y) - st * 0.5), p.y)) for p in pts], mat or M["glass"], side > 0)

    def side_seal(outline, side):
        pts = ensure_ccw(offset_poly(outline, 0.008))
        path = [V((p.x, side * (g.cab_hw(p.y) + 0.003), p.y)) for p in pts]
        return tube(path + [path[0]], 0.0065, 6, M["rubber"], caps=False)

    def cut_panel(key, inner, holes=()):
        """Anbauteil (Tür, Haube, Heckklappe, Ladeklappe) aus der Hülle schneiden – gleiche Wandstärke."""
        d = bpy.data.objects.new(f"{name}_{key}", src.copy())
        asm.coll.objects.link(d)
        boolean(d, [inner.to_object("_c")], "INTERSECT")
        if holes:
            boolean(d, [h.to_object("_c") for h in holes], "DIFFERENCE")
        transfer_normals(d, src)
        return d

    def skin(x0, x1, z0, z1, side, off, mat, nx=16, nz=4):
        """Fläche auf der Unterbau-Außenhaut (für Zierleisten und Verkleidungen), ``off`` nach außen versetzt."""
        md = surface_grid(lambda u, v: V((lerp(x0, x1, u), side * (g.lower_y(lerp(x0, x1, u), lerp(z0, z1, v)) + off),
                                          lerp(z0, z1, v))), nx, nz, mat)
        return orient_faces(md, lambda c: V((0.0, -side, 0.0)))

    def corner_path(z, x_from, x_to, n, end, off=0.003):
        """Pfad um die vordere (end=+1) bzw. hintere (end=−1) Ecke des Unterbaus auf Höhe z, von y=0 nach +Y."""
        xe = g.x_front if end > 0 else g.x_rear
        y_face = g.lower_y(xe - end * 1e-4, z)
        pts = [V((xe + end * off, y, z)) for y in (lerp(x_from, y_face - 0.012, i / 6) for i in range(7))]
        for i in range(1, n + 1):
            x = lerp(xe - end * 0.001, x_to, (i / n) ** 1.6)
            pts.append(V((x, g.lower_y(x, z) + off, z)))
        return pts

    # Konturen: Seitenansicht (x, z), Front/Heck (y, z), Draufsicht (x, y)
    zw0, zw1, zd1 = g.z_belt + 0.05, g.z_roof - 0.13, g.z_roof - 0.09     # Fensterunter-/-oberkante, Türoberkante
    door_f = rounded_poly_2d([(0.855, 0.60), (0.855, g.z_belt + 0.02), (pillar(zd1, 0.13), zd1), (-0.24, zd1),
                              (-0.24, 0.60)], [0.03, 0.02, 0.05, 0.04, 0.03], 4)
    win_f = rounded_poly_2d([(0.78, zw0), (pillar(zw1, 0.20), zw1), (-0.19, zw1), (-0.19, zw0)],
                            [0.02, 0.04, 0.04, 0.03], 4)
    # Hintertür: unten Aussparung um den hinteren Radlauf samt Verbreiterung
    arch_r_off = offset_open(g.arch_outline(g.ra), 0.095)         # vorne unten → oben → hinten unten
    x_dr = -1.17
    notch = [p for p in arch_r_off if p.x > x_dr + 0.01 and p.y > 0.605][::-1]     # von oben nach vorne unten
    z_top = next(lerp(p.y, q.y, (p.x - x_dr) / (p.x - q.x)) for p, q in zip(arch_r_off, arch_r_off[1:])
                 if p.x > x_dr >= q.x)
    door_r = rounded_poly_2d([(-0.28, 0.60), (-0.28, zd1), (x_dr, zd1), (x_dr, z_top)] +
                             [(p.x, p.y) for p in notch] + [(notch[-1].x, 0.60)],
                             [0.03, 0.04, 0.03, 0.0] + [0.0] * len(notch) + [0.02], 4)
    win_r = rounded_poly_2d([(-0.33, zw0), (-0.33, zw1), (-1.00, zw1), (-1.11, zw1 - 0.09), (-1.11, zw0)],
                            [0.03, 0.04, 0.05, 0.04, 0.03], 4)

    def x_back(z):
        return g.cab_xr(z) + 0.24
    win_q = rounded_poly_2d([(-1.25, zw0), (-1.25, zw1), (x_back(zw1), zw1), (x_back(zw0 + 0.10), zw0 + 0.10),
                             (x_back(zw0 + 0.10) + 0.12, zw0)], [0.03, 0.04, 0.04, 0.05, 0.05], 4)
    hw_tg = 0.62
    x_hinge = g.cab_xr(g.z_roof - g.r_roof) + 0.13
    tg = rounded_poly_2d([(-0.80, 0.70), (0.80, 0.70), (0.80, g.z_belt - 0.02), (hw_tg, g.z_roof - 0.10),
                          (hw_tg, g.z_roof + 0.10), (-hw_tg, g.z_roof + 0.10), (-hw_tg, g.z_roof - 0.10),
                          (-0.80, g.z_belt - 0.02)], [0.05, 0.05, 0.08, 0.06, 0.0, 0.0, 0.06, 0.08], 4)
    win_b = rounded_poly_2d([(-0.66, zw0 + 0.04), (0.66, zw0 + 0.04), (0.57, g.z_roof - 0.12), (-0.57, g.z_roof - 0.12)],
                            [0.04, 0.04, 0.05, 0.05], 4)
    ws = rounded_poly_2d([(-0.70, g.z_belt + 0.04), (0.70, g.z_belt + 0.04), (0.60, g.z_roof - 0.10),
                          (-0.60, g.z_roof - 0.10)], [0.05, 0.05, 0.06, 0.06], 4)
    hood = rounded_poly_2d([(1.255, -0.84), (2.30, -0.84), (2.30, 0.84), (1.255, 0.84)], [0.02, 0.06, 0.06, 0.02], 4)
    roof_glass = rounded_rect_2d(1.50, 1.16, 0.10, 4)
    roof_glass = [V((p.x - 0.62, p.y)) for p in roof_glass]
    flap = rounded_rect_2d(0.17, 0.12, 0.02, 3)
    flap = [V((p.x - 1.975, p.y + 1.035)) for p in flap]

    # ------------------------------------------------------------- Türen (4) mit verdeckten Scharnieren
    for side in (1, -1):
        s = "L" if side > 0 else "R"
        y0, y1 = (0.30, 1.30) if side > 0 else (-1.30, -0.30)
        for row, outline, win, hx, handle_x in (("F", door_f, win_f, 0.85, 0.06), ("R", door_r, win_r, -0.285, -0.80)):
            key = f"Door_{row}{s}"
            d = cut_panel(key, prism_y(offset_poly(outline, -gap), y0, y1), [prism_y(win, y0, y1)])
            p_lo, p_hi = V((hx, side * (g.hw + 0.004), 0.72)), V((hx, side * (g.hw + 0.004), 1.12))
            asm.pivot(key, (p_lo + p_hi) / 2, (p_hi - p_lo).normalized() * (-1.0 if side > 0 else 1.0), orient=True,
                      positive="Tür öffnet nach außen", limits=(0, 70))
            asm.add(d, key, key)
            asm.add(side_glass(win, side, M["glass"] if row == "F" else M["privacy"]), f"Glass_{key}", key, smooth=80)
            dt = MeshData()
            dt.merge(side_seal(win, side))
            # bündiger Türgriff, Verkleidung unten an der Tür
            hz = g.z_belt - 0.10
            dt.merge(box((handle_x, side * (g.lower_y(handle_x, hz) + 0.004), hz), (0.17, 0.010, 0.026), M["gloss"]))
            dt.merge(box((handle_x - 0.05, side * (g.lower_y(handle_x, hz) + 0.009), hz), (0.04, 0.002, 0.012),
                         M["chrome"]))
            xs = [p.x for p in outline]
            x_lo, x_hi = min(xs) + 0.01, max(xs) - 0.01
            if row == "R":
                x_lo = notch[-1].x + 0.01
            dt.merge(skin(x_hi, x_lo, 0.605, 0.70, side, 0.004, M["clad"], 12, 2))
            # Innenverkleidung: Armlehne, Griffschale, Lautsprecher, Ambientelicht
            yi = side * (g.hw - st - 0.035)
            xc = (x_lo + x_hi) / 2 - (0.05 if row == "F" else 0.0)
            dt.merge(box((xc, yi, 0.92), (min(x_hi - x_lo - 0.12, 0.60), 0.07, 0.05), M["leather"]))
            dt.merge(box((xc, side * (g.hw - st - 0.012), 0.86), (min(x_hi - x_lo - 0.1, 0.70), 0.024, 0.36),
                         M["interior"]))
            dt.merge(box((xc, side * (g.hw - st - 0.026), 1.05), (min(x_hi - x_lo - 0.1, 0.62), 0.004, 0.006),
                         M["ambient"]))
            dt.merge(cylinder(V((xc + 0.12, side * (g.hw - st - 0.024), 0.70)),
                              V((xc + 0.12, side * (g.hw - st - 0.03), 0.70)), 0.065, 20, M["black"]))
            asm.add(dt, f"DoorTrim_{row}{s}", key, mods=[bevel_s], detail=1)
        # Außenspiegel an der Vordertür (Kamera, Blinker)
        mm = MeshData()
        o = V((0.80, side * (g.hw + 0.06), 1.205))                    # Gehäuse als Superellipsen-Loft nach außen
        st_m = [dict(x=0.0, hw=0.034, zt=0.045, zb=-0.035, zm=0.0, et=3.0, eb=3.0),
                dict(x=0.05, hw=0.050, zt=0.068, zb=-0.058, zm=0.0, et=3.0, eb=3.0),
                dict(x=0.17, hw=0.056, zt=0.074, zb=-0.064, zm=0.0, et=3.0, eb=3.0),
                dict(x=0.215, hw=0.048, zt=0.066, zb=-0.054, zm=0.0, et=3.0, eb=3.0),
                dict(x=0.235, hw=0.022, zt=0.035, zb=-0.025, zm=0.0, et=3.0, eb=3.0)]
        hous = loft_stations(st_m, cosine_spacing(0.0, 0.235, 12, 0.5, 0.8), 24, "ngon", "ngon", M["gloss"])
        hous.transform(frame_matrix(o, Y_AXIS * side, -X_AXIS * side, Z_AXIS))
        mm.merge(hous)
        base = V((0.80, side * (g.lower_y(0.80, 1.13) - 0.005), 1.13))
        mm.merge(tube(spline([base, base + V((-0.01, side * 0.04, 0.025)), o + V((-0.012, side * 0.03, -0.022))], 4),
                      (0.016, 0.03), 10, M["gloss"], up=Z_AXIS, exponent=3.0))
        mm.merge(box(o + V((-0.057, side * 0.125, 0.004)), (0.004, 0.17, 0.105), M["mirror"]))
        mm.merge(box(o + V((0.052, side * 0.165, -0.034)), (0.008, 0.10, 0.008), M["amber"]))
        mm.merge(lathe([(0.0, -0.004), (0.011, 0.0), (0.0, 0.004)], 10, o + V((0.0, side * 0.15, -0.064)), Z_AXIS,
                       M["lens"]))
        asm.add(mm, f"Mirror_{s}", f"Door_F{s}", mods=[bevel_s])

    # ------------------------------------------------------------- Haube, Heckklappe, Ladeklappe
    hz0, hz1 = 1.0, 1.35
    hood_obj = cut_panel("Hood", prism_z(offset_poly(hood, -gap), hz0, hz1))
    asm.pivot("Hood", V((1.25, 0.0, g.zt(1.25) + 0.003)), -Y_AXIS, orient=True,
              positive="Haube öffnet (Vorderkante nach oben)", limits=(0, 60))
    asm.add(hood_obj, "Hood", "Hood")
    hf = MeshData()                                  # Versteifungsrahmen unter der Haube

    def under_hood(pts, d=0.012):
        return [V((p[0], p[1], g.zt(p[0]) - st - d)) for p in pts]
    frame = resample(fillet_path([(1.75, -0.78, 0.0), (2.24, -0.78, 0.0), (2.24, 0.78, 0.0), (1.32, 0.78, 0.0),
                                  (1.32, -0.78, 0.0), (1.75, -0.78, 0.0)], 0.07, 4), 0.05)
    hf.merge(tube(under_hood([(p.x, p.y) for p in frame]), (0.008, 0.022), 8, M["paint"], caps=False, up=Z_AXIS,
                  exponent=4.0))
    for side in (1, -1):
        hf.merge(tube(under_hood([(lerp(1.38, 2.18, i / 12), side * lerp(0.72, -0.72, i / 12)) for i in range(13)]),
                      (0.007, 0.02), 8, M["paint"], up=Z_AXIS, exponent=4.0))
    asm.add(hf, "HoodFrame", "Hood", detail=1)

    tg_obj = cut_panel("Tailgate", prism_x(offset_poly(tg, -gap), g.x_rear - 0.10, x_hinge - gap),
                       [prism_x(win_b, g.x_rear - 0.10, g.x_rear + 0.40)])
    z_h = g.z_roof + 0.004
    asm.pivot("Tailgate", V((x_hinge - 0.012, 0.0, z_h)), Y_AXIS, orient=True, limits=(0, 80),
              positive="Heckklappe öffnet nach oben (Scharnier am Dach)")
    asm.add(tg_obj, "Tailgate", "Tailgate")
    pts = ensure_ccw(offset_poly(win_b, 0.025))
    asm.add(flat([V((g.cab_xr(p.y) + st * 0.5, p.x, p.y)) for p in pts], M["privacy"], True), "Glass_Tailgate",
            "Tailgate", smooth=80)
    tt = MeshData()
    path = [V((g.cab_xr(p.y) - 0.003, p.x, p.y)) for p in ensure_ccw(offset_poly(win_b, 0.008))]
    tt.merge(tube(path + [path[0]], 0.0065, 6, M["rubber"], caps=False))
    # Dachspoiler mit dritter Bremsleuchte, Heckwischer
    xs0, xs1 = g.cab_xr(g.z_roof - g.r_roof) - 0.10, x_hinge - 0.02
    sp = rounded_poly_2d([(xs1, g.z_roof - 0.01), (xs1 - 0.05, g.z_roof + 0.018), (xs0 + 0.02, g.z_roof + 0.022),
                          (xs0, g.z_roof + 0.0), (xs0 + 0.06, g.z_roof - 0.05), (g.cab_xr(g.z_roof - 0.07), g.z_roof - 0.07)],
                         [0.0, 0.03, 0.015, 0.01, 0.02, 0.0], 3)
    tt.merge(prism_y(sp, -hw_tg + 0.01, hw_tg - 0.01, M["gloss"]))
    tt.merge(box((xs0 + 0.004, 0.0, g.z_roof + 0.009), (0.006, 0.40, 0.010), M["tail"]))
    zw = zw0 + 0.10
    wp = V((g.cab_xr(zw) - 0.012, 0.0, zw))
    tt.merge(cylinder(wp + X_AXIS * 0.01, wp - X_AXIS * 0.02, 0.016, 12, M["black"]))
    wt = V((g.cab_xr(zw + 0.10) - 0.012, 0.46, zw + 0.10))
    tt.merge(tube([wp, wt], 0.006, 6, M["black"]))
    # Wortmarke, Kennzeichen-Mulde mit Leuchte und Kamera, Griffleiste
    x_face = g.x_rear
    wm = text_md(cfg.suv_badge, 0.055, frame_matrix(V((x_face - 0.0035, 0.0, 1.035)), -Y_AXIS, Z_AXIS, -X_AXIS),
                 M["chrome"], bold=0.04)
    tt.merge(wm)
    tt.merge(box((x_face - 0.004, 0.0, 0.825), (0.008, 0.56, 0.16), M["gloss"]))
    tt.merge(box((x_face - 0.009, 0.0, 0.925), (0.012, 0.36, 0.03), M["gloss"]))
    tt.merge(box((x_face - 0.0155, 0.0, 0.925), (0.002, 0.03, 0.02), M["lens"]))
    tt.merge(plate(x_face - 0.0085, 0.825, -1.0))
    # Leuchtband über die volle Breite (Mittelteil an der Klappe)
    zl = 1.10
    tt.merge(box((x_face - 0.006, 0.0, zl), (0.012, 1.58, 0.07), M["gloss"]))
    tt.merge(box((x_face - 0.0125, 0.0, zl + 0.008), (0.002, 1.56, 0.014), M["tail"]))
    asm.add(tt, "TailgateTrim", "Tailgate", mods=[bevel_s], detail=1)

    flap_obj = cut_panel("ChargeFlap", prism_y(offset_poly(flap, -gap), 0.30, 1.30))
    fx = max(p.x for p in flap) - 0.004
    asm.pivot("ChargeFlap", V((fx, g.hw - 0.002, 1.035)), -Z_AXIS, orient=True, limits=(0, 110),
              positive="Ladeklappe öffnet nach außen (Scharnier vorne)")
    asm.add(flap_obj, "ChargeFlap", "ChargeFlap")

    # ------------------------------------------------------------- Öffnungen in der Karosserie, Scheiben
    cutters = []
    for side in (1, -1):
        y0, y1 = (0.30, 1.30) if side > 0 else (-1.30, -0.30)
        cutters += [prism_y(offset_poly(door_f, gap), y0, y1), prism_y(offset_poly(door_r, gap), y0, y1),
                    prism_y(win_q, y0, y1)]
        for a in (g.fa, g.ra):
            outline = g.arch_outline(a)
            cutters.append(prism_y([V((p.x, p.y)) for p in outline] + [V((a.x, 0.30))], side * 0.46, side * 1.30))
    cutters.append(prism_y(offset_poly(flap, gap), 0.30, 1.30))
    cutters.append(prism_z(offset_poly(hood, gap), hz0, hz1))
    cutters.append(prism_x(offset_poly(tg, gap), g.x_rear - 0.10, x_hinge))
    cutters.append(prism_x(ws, 0.20, 1.20))
    cutters.append(prism_z(roof_glass, g.z_roof - 0.12, g.z_roof + 0.10))
    boolean(body, [c.to_object("_c") for c in cutters], "DIFFERENCE")
    transfer_normals(body, src)
    asm.add(body, "Body")
    cab_src = g.cabin_md(M["paint"])
    bpy.data.meshes.remove(src)

    glass = MeshData()
    wsg = MeshData().add([V((p.x, p.y, 0.0)) for p in ensure_ccw(offset_poly(ws, 0.025))],
                         [tuple(range(len(ensure_ccw(offset_poly(ws, 0.025)))))], None, M["glass"])
    wsg = subdivide_tris(wsg, 0.12)
    wsg.verts = [g.ws_point(v.x, v.y) - n_ws * (st * 0.5) for v in wsg.verts]
    glass.merge(orient_faces(wsg, lambda c: -X_AXIS))
    for side in (1, -1):
        glass.merge(side_glass(win_q, side, M["privacy"]))
    rg = MeshData().add([V((p.x, p.y, g.z_roof + 0.5)) for p in offset_poly(roof_glass, 0.03)],
                        [tuple(range(len(roof_glass)))], None, M["privacy"])
    rg = project_onto(rg, cab_src, -Z_AXIS, offset=-st * 0.5, max_edge=0.10)
    glass.merge(orient_faces(rg, lambda c: -Z_AXIS))
    asm.add(glass, "Glass", smooth=80)
    seals = MeshData()
    path = [g.ws_point(p.x, p.y) + n_ws * 0.003 for p in ensure_ccw(offset_poly(ws, 0.008))]
    seals.merge(tube(path + [path[0]], 0.0065, 6, M["rubber"], caps=False))
    for side in (1, -1):
        seals.merge(side_seal(win_q, side))
    rp = offset_poly(roof_glass, 0.006)
    bvh = BVHTree.FromPolygons(cab_src.verts, [t for f in cab_src.faces for t in
                                               [(f[0], f[i], f[i + 1]) for i in range(1, len(f) - 1)]])
    rp = [bvh.ray_cast(V((p.x, p.y, g.z_roof + 0.5)), -Z_AXIS, 2.0)[0] + Z_AXIS * 0.002 for p in rp]
    seals.merge(tube(rp + [rp[0]], 0.006, 6, M["rubber"], caps=False))
    # Wasserkasten-Abdeckung unter der Frontscheibe, Wischer
    seals.merge(box((1.225, 0.0, g.zt(1.225) + 0.003), (0.05, 1.40, 0.006), M["black"]))
    zw_ = g.z_belt + 0.06
    for y0_, y1_ in ((0.50, -0.14), (-0.04, -0.66)):
        p0 = g.ws_point(y0_, zw_) + n_ws * 0.012
        p1 = g.ws_point(y1_, zw_ + 0.03) + n_ws * 0.012
        seals.merge(tube([p0, p1], (0.006, 0.010), 6, M["black"], up=n_ws, exponent=3.0))
        seals.merge(cylinder(p0 - n_ws * 0.012, p0 + n_ws * 0.01, 0.013, 10, M["black"]))
    asm.add(seals, "Seals", detail=2)

    # ------------------------------------------------------------- Radhäuser, kantige Kotflügelverbreiterungen
    trim = MeshData()
    for side in (1, -1):
        for a in (g.fa, g.ra):
            outline = g.arch_outline(a)
            inner = offset_open(outline, -0.004)
            rings = [[V((p.x, side * y, p.y)) for p in inner] for y in (0.46, g.hw + 0.004)]
            trim.merge(orient_faces(grid_rings(rings, closed=False, mat=M["clad"]),
                                    lambda c, a=a: c - V((a.x, c.y, a.z))))
            wall = [V((p.x, side * 0.46, p.y)) for p in inner]
            trim.merge(orient_faces(flat(wall, M["clad"]), lambda c, s=side: V((0.0, -s, 0.0))))
            fl = offset_open(outline, 0.040)
            path = [V((p.x, side * (g.hw + 0.022), p.y)) for p in fl]
            trim.merge(tube(path, (0.032, 0.045), 12, M["clad"], up=Y_AXIS, exponent=5.0))
    asm.add(trim, "Flares", mods=[bevel_s])

    # ------------------------------------------------------------- Front: Grill, LED-Scheinwerfer, Stoßfänger
    fr = MeshData()
    lens = MeshData()
    xf = g.x_front
    zg0, zg1 = 0.76, 1.03                                        # geschlossener Grill mit senkrechten Stäben
    fr.merge(box((xf + 0.004, 0.0, (zg0 + zg1) / 2), (0.008, 0.96, zg1 - zg0), M["black"]))
    gr = rounded_rect_2d(0.96, zg1 - zg0, 0.05, 4)
    fr.merge(extrude_ring([V((p.x, p.y)) for p in gr], [V((p.x * 0.9375, p.y * (zg1 - zg0 - 0.05) / (zg1 - zg0)))
                                                        for p in gr], 0.02,
                          frame_matrix(V((xf + 0.012, 0.0, (zg0 + zg1) / 2)), Y_AXIS, Z_AXIS, X_AXIS), M["gloss"]))
    n_bar = 15
    for i in range(n_bar):
        y = lerp(-0.42, 0.42, i / (n_bar - 1))
        if abs(y) < 0.07:
            continue
        fr.merge(box((xf + 0.016, y, (zg0 + zg1) / 2), (0.016, 0.022, zg1 - zg0 - 0.06), M["gloss"]))
    # Emblem: stilisierter Berggipfel, hinterleuchtet
    em = MeshData()
    em.merge(cylinder(V((xf + 0.006, 0.0, 0.895)), V((xf + 0.026, 0.0, 0.895)), 0.062, 32, M["gloss"]))
    peak = [(-0.045, -0.025), (-0.012, 0.026), (0.0, 0.010), (0.012, 0.030), (0.045, -0.025), (0.030, -0.025),
            (0.012, 0.004), (0.0, -0.012), (-0.012, 0.0), (-0.030, -0.025)]
    em.merge(extrude(peak, 0.006, frame_matrix(V((xf + 0.029, 0.0, 0.895)), Y_AXIS, Z_AXIS, X_AXIS), M["led"]))
    fr.merge(em)
    small_f = MeshData()
    small_f.merge(box((xf + 0.009, 0.0, zg1 + 0.012), (0.012, 0.05, 0.016), M["gloss"]))       # Frontkamera
    small_f.merge(box((xf + 0.016, 0.0, zg1 + 0.012), (0.002, 0.016, 0.010), M["lens"]))
    # Scheinwerferband mit gestufter LED-Signatur, drei Projektionsmodulen und Blinkerleiste (je Seite)
    zh, hh = 1.010, 0.034
    for side in (1, -1):
        band = corner_path(zh, 0.505, g.x_front - 0.16, 8, 1, 0.004)
        band = [V((p.x, side * p.y, p.z)) for p in band]
        fr.merge(tube(band, (hh, 0.007), 8, M["gloss"], up=Z_AXIS, exponent=6.0))
        drl = corner_path(zh + 0.021, 0.61, g.x_front - 0.12, 8, 1, 0.011)
        drl = [V((xf + 0.011, 0.515, zh - 0.018)), V((xf + 0.011, 0.565, zh - 0.018))] + drl
        drl = [V((p.x, side * p.y, p.z)) for p in drl]
        fr.merge(tube(drl, (0.0055, 0.007), 6, M["led"], up=Z_AXIS, exponent=6.0))
        for y in (0.69, 0.82):                                         # senkrechte "Zähne" der Lichtsignatur
            fr.merge(box((xf + 0.012, side * y, zh + 0.004), (0.010, 0.010, 0.034), M["led"]))
        for y in (0.63, 0.7525, 0.875 - 0.06):
            c = V((xf + 0.010, side * y, zh - 0.010))
            fr.merge(lathe([(0.0, -0.004), (0.021, -0.004), (0.021, 0.003), (0.016, 0.006)], 20, c, X_AXIS, M["chrome"]))
            fr.merge(lathe([(0.0, 0.002), (0.010, 0.003), (0.0, 0.006)], 12, c, X_AXIS, M["lamp"]))
            lens.merge(lathe([(0.016, 0.006), (0.010, 0.010), (0.0, 0.011)], 16, c, X_AXIS, M["lens"]))
        am = corner_path(zh - 0.021, 0.62, g.x_front - 0.10, 6, 1, 0.010)
        fr.merge(tube([V((p.x, side * p.y, p.z)) for p in am[3:]], (0.003, 0.004), 6, M["amber"], up=Z_AXIS,
                      exponent=6.0))
    # Stoßfänger (Kunststoff, genarbt) mit senkrechten LED-Leisten, Parksensoren, Kennzeichen
    fr.merge(suv_bumper(2.04, 1.0, 0.42, 0.76, M["clad"]))
    for side in (1, -1):
        y = side * 0.70
        fr.merge(box((2.433, y, 0.665), (0.012, 0.075, 0.17), M["gloss"]))
        fr.merge(box((2.440, y, 0.665), (0.004, 0.020, 0.15), M["led"]))
        for yy in (0.27, 0.55):
            small_f.merge(cylinder(V((2.430, side * yy, 0.735)), V((2.436, side * yy, 0.735)), 0.011, 12, M["clad"]))
        small_f.merge(box((2.07, side * 0.969, 0.60), (0.07, 0.004, 0.025), M["reflector"]))
    fr.merge(box((2.436, 0.0, 0.655), (0.006, 0.54, 0.12), M["gloss"]))
    small_f.merge(plate(2.439, 0.655, 1.0))
    # Unterfahrschutz (Alu) mit Bergehaken
    sk = rounded_poly_2d([(1.98, 0.37), (2.30, 0.37), (2.475, 0.43), (2.485, 0.555), (2.45, 0.585), (2.42, 0.56),
                          (2.10, 0.43)], [0.0, 0.05, 0.03, 0.02, 0.015, 0.02, 0.05], 3)
    fr.merge(prism_y(sk, -0.66, 0.66, M["skid"]))
    for side in (1, -1):
        fr.merge(box((2.486, side * 0.17, 0.50), (0.006, 0.17, 0.022), M["black"]))       # Kühlluftschlitze
        y = side * 0.42
        fr.merge(tube(fillet_path([(2.47, y, 0.46), (2.56, y, 0.46), (2.56, y, 0.545), (2.47, y, 0.545)], 0.035),
                      0.0135, 10, M["skid"]))
        fr.merge(box((2.48, y, 0.5025), (0.03, 0.07, 0.12), M["clad"]))
    asm.add(fr, "Front", mods=[bevel_s])
    asm.add(lens, "HeadlightLens", smooth=60)
    asm.add(small_f, "FrontDetails", mods=[bevel_s], detail=2)

    # ------------------------------------------------------------- Heck: Leuchten an den Ecken, Stoßfänger
    rr = MeshData()
    for side in (1, -1):
        band = corner_path(zl, 0.815, g.x_rear + 0.15, 8, -1, 0.004)
        rr.merge(tube([V((p.x, side * p.y, p.z)) for p in band], (0.035, 0.006), 8, M["gloss"], up=Z_AXIS, exponent=6.0))
        tl = corner_path(zl + 0.008, 0.815, g.x_rear + 0.13, 8, -1, 0.0105)
        rr.merge(tube([V((p.x, side * p.y, p.z)) for p in tl], (0.007, 0.005), 6, M["tail"], up=Z_AXIS, exponent=6.0))
        tl2 = corner_path(zl - 0.020, 0.83, g.x_rear + 0.06, 6, -1, 0.0105)
        rr.merge(tube([V((p.x, side * p.y, p.z)) for p in tl2], (0.004, 0.005), 6, M["amber"], up=Z_AXIS,
                      exponent=6.0))
    rr.merge(suv_bumper(-2.04, -1.0, 0.44, 0.70, M["clad"]))
    rs_ = rounded_poly_2d([(-2.00, 0.40), (-2.30, 0.40), (-2.47, 0.46), (-2.47, 0.52), (-2.43, 0.54), (-2.10, 0.45)],
                          [0.0, 0.05, 0.03, 0.015, 0.02, 0.05], 3)
    rr.merge(prism_y(rs_, -0.58, 0.58, M["skid"]))
    for side in (1, -1):
        rr.merge(box((-2.447, side * 0.84, 0.60), (0.006, 0.05, 0.13), M["reflector"]))
        rr.merge(box((-2.447, side * 0.62, 0.66), (0.006, 0.12, 0.03), M["reverse"] if side > 0 else M["tail"]))
        small = (-2.446, side * 0.35, 0.68)
        rr.merge(cylinder(V(small), V(small) - X_AXIS * 0.006, 0.011, 12, M["clad"]))
    rr.merge(tube(fillet_path([(-2.46, 0.0, 0.44), (-2.53, 0.0, 0.44), (-2.53, 0.0, 0.51), (-2.46, 0.0, 0.51)], 0.03),
                  0.012, 10, M["skid"]))
    asm.add(rr, "Rear", mods=[bevel_s])

    # ------------------------------------------------------------- Seite: Schwellerverkleidung, Ladebuchse
    sd = MeshData()
    for side in (1, -1):
        sec = [(0.86, 0.43), (0.962, 0.455), (0.975, 0.52), (0.968, 0.575), (0.945, 0.592), (0.86, 0.592)]
        sec = [(side * y, z) for y, z in sec]
        x0, x1 = g.ra.x + 0.50 + 0.07, g.fa.x - 0.50 - 0.07
        sd.merge(prism_x(sec, x0, x1, M["clad"]))
        sd.merge(box((x1 - 0.18, side * 0.973, 0.53), (0.25, 0.006, 0.012), M["skid"]))      # Zierleiste
    # Ladebuchse (CCS) hinter der Klappe, links hinten
    cp = V((-1.985, g.hw - 0.05, 1.035))
    port = MeshData()
    port.merge(box(cp + V((0.0, 0.004, 0.0)), (0.11, 0.04, 0.10), M["black"]))
    port.merge(cylinder(cp + Y_AXIS * 0.025, cp + Y_AXIS * 0.032, 0.032, 24, M["gloss"]))
    port.merge(cylinder(cp + Y_AXIS * 0.025 + V((0.0, 0.0, -0.045)), cp + Y_AXIS * 0.032 + V((0.0, 0.0, -0.045)), 0.02,
                        16, M["gloss"]))
    for dx in (-0.012, 0.012):
        port.merge(cylinder(cp + V((dx, 0.032, -0.045)), cp + V((dx, 0.035, -0.045)), 0.0065, 10, M["black"]))
    for k in range(5):
        a_ = TAU * k / 5
        port.merge(cylinder(cp + V((0.016 * math.cos(a_), 0.032, 0.016 * math.sin(a_))),
                            cp + V((0.016 * math.cos(a_), 0.035, 0.016 * math.sin(a_))), 0.004, 8, M["black"]))
    port.merge(box(cp + V((0.06, 0.026, 0.035)), (0.012, 0.004, 0.012), M["led"]))
    sd.merge(port)
    asm.add(sd, "Details", mods=[bevel_s], detail=2)
    # ------------------------------------------------------------- Dach: Reling und Dachplattform
    rack = MeshData()
    zr = g.z_roof + 0.004
    for side in (1, -1):
        y = side * 0.64
        rail = [V((x, y, zr + 0.045 + 0.012 * math.sin(math.pi * clamp((x + 1.84) / 2.12)))) for x in
                (lerp(0.26, -1.84, i / 20) for i in range(21))]
        rack.merge(tube(rail, (0.012, 0.022), 10, M["black"], up=Z_AXIS, exponent=3.0))
        for x in (0.20, -0.82, -1.78):
            rack.merge(box((x, y, zr + 0.02), (0.10, 0.045, 0.05), M["black"]))
    zp = g.z_roof + 0.115
    frame_ring = rrect2_ring(0.23, -1.81, 0.67, 0.08, 0.08, 6, 12, 3, zp)
    rack.merge(tube(frame_ring + [frame_ring[0]], (0.018, 0.022), 8, M["black"], caps=False, up=Z_AXIS, exponent=4.0))
    for i in range(11):                                       # Lattenrost
        x = lerp(0.16, -1.74, i / 10)
        rack.merge(box((x, 0.0, zp - 0.004), (0.06, 1.32, 0.014), M["black"]))
    for side in (1, -1):                                      # Reling-Galerie an den Seiten
        y = side * 0.67
        rack.merge(cylinder(V((0.12, y, zp + 0.065)), V((-1.70, y, zp + 0.065)), 0.011, 8, M["black"]))
        for k in range(9):
            x = lerp(0.12, -1.70, k / 8)
            rack.merge(cylinder(V((x, y, zp)), V((x, y, zp + 0.065)), 0.009, 8, M["black"]))
        for x in (0.20, -0.82, -1.78):                        # Klemmfüße auf der Reling
            rack.merge(box((x, side * 0.64, zp - 0.035), (0.06, 0.06, 0.06), M["black"]))
    defl = [V((0.255 + 0.04 * math.sin(math.pi * t), lerp(-0.60, 0.60, t), zp + 0.035)) for t in
            (i / 12 for i in range(13))]                      # Windabweiser
    rack.merge(tube(defl, (0.004, 0.035), 8, M["black"], up=V((0.6, 0.0, 0.8)), exponent=4.0))
    asm.add(rack, "RoofRack", mods=[bevel_s])

    # ------------------------------------------------------------- Innenraum
    it = MeshData()
    xa0, xa1 = g.ra.x - 0.52, g.ra.x + 0.52                                         # hinteres Radhaus
    x_cargo = -1.24                                                                # Stufe zum Laderaumboden
    it.merge(box(((xa1 + 0.92) / 2, 0.0, 0.56), (0.92 - xa1, 1.72, 0.03), M["floor"]))
    it.merge(box((1.06, 0.0, 0.56), (0.28, 0.88, 0.03), M["floor"]))                # Fußraum zwischen den Radhäusern
    it.merge(box(((x_cargo + xa1) / 2, 0.0, 0.56), (xa1 - x_cargo, 0.88, 0.03), M["floor"]))
    it.merge(box(((xa0 + x_cargo) / 2, 0.0, 0.73), (x_cargo - xa0, 0.88, 0.03), M["floor"]))
    it.merge(box(((g.x_rear + 0.06 + xa0) / 2, 0.0, 0.73), (xa0 - g.x_rear - 0.06, 1.72, 0.03), M["floor"]))
    it.merge(box((x_cargo, 0.0, 0.645), (0.03, 0.88, 0.17), M["floor"]))
    dash = rounded_poly_2d([(1.20, 0.97), (1.20, 1.15), (0.80, 1.165), (0.62, 1.12), (0.55, 1.02), (0.58, 0.88),
                            (0.70, 0.79), (0.90, 0.79), (0.90, 0.97)], [0.0, 0.0, 0.05, 0.05, 0.04, 0.05, 0.04, 0.0, 0.0], 3)
    it.merge(extrude(dash, 1.72, frame_matrix(V((0, 0, 0)), X_AXIS, Z_AXIS, -Y_AXIS), M["panel"]))
    it.merge(box((0.585, -0.30, 1.035), (0.01, 0.86, 0.022), M["trim"]))          # Zierleiste Beifahrerseite
    it.merge(box((0.578, -0.30, 1.018), (0.004, 0.86, 0.004), M["ambient"]))
    for y in (0.72, 0.24, -0.24, -0.72):                                            # Luftausströmer
        it.merge(box((0.565, y, 1.075), (0.02, 0.15, 0.026), M["black"]))
    # Freistehendes Display-Band (Instrumente + Zentralbildschirm)
    dm = Matrix.Translation((0.62, 0.17, 1.205)) @ Matrix.Rotation(math.radians(-12), 4, "Y")
    it.merge(box((0, 0, 0), (0.014, 0.98, 0.13), M["gloss"], dm))
    it.merge(box((-0.0075, 0.24, 0.0), (0.002, 0.34, 0.11), M["screen"], dm))
    it.merge(box((-0.0075, -0.21, 0.0), (0.002, 0.50, 0.11), M["screen"], dm))
    it.merge(box((0.018, 0.0, -0.06), (0.03, 0.10, 0.04), M["panel"], dm))
    # Mittelkonsole (Brücke) mit Wählhebel, Ladeschale, Becherhaltern, Armlehne
    it.merge(box((0.25, 0.0, 0.88), (0.72, 0.22, 0.05), M["panel"]))
    it.merge(box((-0.05, 0.0, 0.73), (0.40, 0.22, 0.30), M["panel"]))
    it.merge(box((0.25, 0.0, 0.65), (0.70, 0.20, 0.03), M["panel"]))
    it.merge(box((-0.13, 0.0, 0.91), (0.30, 0.20, 0.06), M["leather"]))
    it.merge(box((0.42, 0.0, 0.906), (0.16, 0.12, 0.003), M["gloss"]))
    it.merge(lathe([(0.0, 0.0), (0.024, 0.0), (0.024, 0.03), (0.020, 0.036), (0.0, 0.036)], 24,
                   V((0.27, 0.0, 0.905)), Z_AXIS, M["trim"]))
    for y in (-0.05, 0.05):
        it.merge(cylinder(V((0.14, y, 0.9065)), V((0.14, y, 0.9075)), 0.037, 20, M["black"]))
    for x, w in ((0.81, 0.06), (0.86, 0.05)):                                       # Pedale
        it.merge(box((x, 0.36 if w > 0.055 else 0.47, 0.67), (0.02, w, 0.10 if w > 0.055 else 0.07), M["black"]))

    def seat(x, y, w, single=True):
        sm_ = MeshData()
        sm_.merge(box((x, y, 0.86), (0.50, w, 0.11), M["leather"]))
        sm_.merge(box((x, y, 0.72), (0.42, min(w * 0.8, 0.9), 0.17), M["black"]))
        bm = Matrix.Translation((x - 0.23, y, 0.91)) @ Matrix.Rotation(math.radians(-16), 4, "Y")
        sm_.merge(box((0.0, 0.0, 0.30), (0.11, w, 0.60), M["leather"], bm))
        if single:
            for sgn in (1, -1):                                       # Seitenwangen
                sm_.merge(box((x + 0.01, y + sgn * (w / 2 - 0.035), 0.895), (0.46, 0.07, 0.06), M["leather"]))
                sm_.merge(box((0.035, sgn * (w / 2 - 0.035), 0.27), (0.08, 0.07, 0.48), M["leather"], bm))
            sm_.merge(box((0.0, 0.0, 0.70), (0.10, 0.27, 0.17), M["leather"], bm))
            sm_.merge(box((0.0, 0.0, 0.615), (0.06, 0.18, 0.04), M["chrome"], bm))
        else:
            for yy in (-0.48, 0.0, 0.48):
                sm_.merge(box((0.0, yy, 0.69), (0.09, 0.25, 0.15), M["leather"], bm))
        return sm_
    for y in (0.40, -0.40):
        it.merge(seat(-0.06, y, 0.54))
    it.merge(seat(-0.98, 0.0, 1.46, single=False))
    asm.add(it, "Interior", mods=[soft], detail=1)
    hub = V((0.37, 0.40, 1.08))
    col = V((-math.cos(math.radians(24)), 0.0, math.sin(math.radians(24))))
    asm.add(cylinder(hub - col * 0.25, hub - col * 0.03, 0.026, 12, M["panel"]), "SteeringColumn", detail=1)
    asm.pivot("SteeringWheel", hub, col, orient=True, positive="Lenkrad nach links (gegen den Uhrzeigersinn)",
              limits=(-540, 540))
    u, w = basis_from(col)
    swm = lathe([(0.185 + 0.016 * math.cos(TAU * i / 10), 0.016 * math.sin(TAU * i / 10)) for i in range(10)], 48,
                hub, col, M["leather"], closed_profile=True)
    for j, v in enumerate(swm.verts):                      # Kranz unten abflachen
        dv = v - hub
        lo = -0.15
        h = dv.dot(Z_AXIS)
        if h < lo:
            swm.verts[j] = v + Z_AXIS * (lo - h) * 0.9
    swm.merge(lathe([(0.0, -0.03), (0.065, -0.025), (0.07, 0.012), (0.0, 0.024)], 24, hub, col, M["panel"]))
    swm.merge(lathe([(0.0, 0.024), (0.018, 0.024), (0.0, 0.027)], 16, hub, col, M["chrome"]))
    for dd in (u, -u):
        dd_ = (dd * 0.9 - Z_AXIS * 0.25).normalized()
        dd_ = (dd_ - col * dd_.dot(col)).normalized()
        swm.merge(box((0, 0, 0), (0.016, 0.05, 0.12), M["panel"], frame_matrix(hub + dd_ * 0.12, col.cross(dd_), col, dd_)))
    asm.add(swm, "SteeringWheel", "SteeringWheel", mods=[bevel_s], detail=1)

    # ------------------------------------------------------------- Frunk (bei offener Haube sichtbar)
    fk, fk_small = suv_frunk(g, M)
    asm.add(fk, "Frunk", mods=[bevel_s], detail=1)
    asm.add(fk_small, "FrunkDetails", detail=2)

    # ------------------------------------------------------------- Unterboden: Batterie, Antriebseinheiten, Hilfsrahmen
    ch = MeshData()
    ch.merge(box((-0.06, 0.0, 0.33), (1.88, 1.40, 0.17), M["battery"]))               # Hochvolt-Batterie
    for k in range(9):
        ch.merge(box((lerp(-0.92, 0.80, k / 8), 0.0, 0.243), (0.035, 1.34, 0.008), M["battery"]))
    for side in (1, -1):
        ch.merge(box((-0.06, side * 0.715, 0.40), (1.84, 0.05, 0.03), M["battery"]))
        for k in range(8):
            x = lerp(-0.92, 0.80, k / 7)
            ch.merge(cylinder(V((x, side * 0.715, 0.415)), V((x, side * 0.715, 0.425)), 0.012, 6, M["steel"]))
    for a, sgn in ((g.fa, 1.0), (g.ra, -1.0)):
        # Hilfsrahmen: Längsträger und Querträger mit Lenkerkonsolen
        for side in (1, -1):
            ch.merge(box((a.x, side * 0.30, 0.30), (0.62, 0.07, 0.09), M["chassis"]))
            ch.merge(box((a.x - 0.07, side * 0.43, 0.66), (0.50, 0.04, 0.06), M["chassis"]))    # Konsole oberer Lenker
        for dx in (-0.26, 0.26):
            ch.merge(box((a.x + dx, 0.0, 0.30), (0.07, 0.60, 0.08), M["chassis"]))
        # Antriebseinheit: Motor (quer), Getriebe, Inverter
        mc = V((a.x + sgn * 0.20, 0.0, 0.40))
        ch.merge(cylinder(mc + V((0.0, -0.20, 0.0)), mc + V((0.0, 0.12, 0.0)), 0.125, 28, M["motor"]))
        for yy in (-0.15, -0.08, 0.0, 0.07):
            ch.merge(cylinder(mc + V((0.0, yy, 0.0)), mc + V((0.0, yy + 0.012, 0.0)), 0.132, 28, M["motor"]))
        ch.merge(box((a.x + sgn * 0.06, 0.0, a.z), (0.30, 0.30, 0.24), M["motor"]))
        ch.merge(cylinder(V((a.x, -0.245, a.z)), V((a.x, 0.245, a.z)), 0.04, 16, M["motor"]))     # Abtriebe
        ch.merge(box((a.x + sgn * 0.20, -0.02, 0.57), (0.28, 0.36, 0.09), M["battery"]))     # Inverter
        for yy in (-0.08, 0.04):                                                       # HV-Leitungen zur Batterie
            p0 = V((a.x + sgn * 0.20, yy, 0.615))
            p1 = V((0.86 if sgn > 0 else -0.98, yy * 2.0, 0.40))
            ch.merge(tube(spline([p0, p0 + V((-sgn * 0.20, 0.0, 0.03)), (p0 + p1) / 2 + V((0.0, 0.0, 0.05)), p1], 4),
                          0.016, 8, M["hv"]))
    ch.merge(box((g.fa.x + 0.55, 0.0, 0.40), (0.30, 0.84, 0.012), M["chassis"]))     # Unterbodenverkleidung vorn
    ch.merge(box((g.ra.x - 0.55, 0.0, 0.45), (0.30, 0.84, 0.012), M["chassis"]))
    asm.add(ch, "Chassis", mods=[bevel_s], detail=1)

    # ------------------------------------------------------------- Fahrwerk: Doppelquerlenker, Federbeine, Räder
    wheel = suv_wheel(M)
    corners = ("FL", "FR", "RL", "RR")
    for wk in corners:
        front, side = wk[0] == "F", (1.0 if wk[1] == "L" else -1.0)
        P, parts = suv_corner(g, M, front, side)
        k = lambda base: f"{base}_{wk}"
        asm.pivot(k("Susp"), P["lbj"], Z_AXIS, motion="slide", limits=(-0.08, 0.08),
                  positive="Rad federt ein (nach oben)")
        asm.add(parts["LowerJoint"], k("Susp"), k("Susp"), mods=[bevel_s], detail=1)
        kp = (P["ubj"] - P["lbj"]).normalized()              # vorne Achsschenkel, hinten Hinterachslenkung
        asm.pivot(k("Steer"), P["lbj"], kp, orient=True, parent=k("Susp"), positive="Rad lenkt nach links",
                  limits=(-32, 32) if front else (-10, 10))
        asm.add(parts["Upright"], k("Steer"), k("Steer"), mods=[bevel_s])
        wheel_parent = k("Steer")
        asm.pivot(k("ArmLower"), P["la_c"], (P["lbj"] - P["la_c"]).normalized(), orient=True, motion="aim",
                  target=k("Susp"), positive=f"lokales +Y zeigt auf Susp_{wk}")
        asm.add(parts["ArmLower"], k("ArmLower"), k("ArmLower"), mods=[bevel_s], detail=1)
        asm.pivot(k("BallJoint"), P["ubj"], (P["ua"][0] - P["ubj"]).normalized(), orient=True, parent=k("Susp"),
                  motion="aim", target=k("ArmUpperFwd"), positive=f"lokales +Y zeigt auf ArmUpperFwd_{wk}")
        asm.add(parts["BallJoint"], k("BallJoint"), k("BallJoint"), detail=1)
        for b, part in zip(P["ua"], ("ArmUpperFwd", "ArmUpperAft")):
            asm.pivot(k(part), b, (P["ubj"] - b).normalized(), orient=True, motion="aim", target=k("BallJoint"),
                      positive=f"lokales +Y zeigt auf BallJoint_{wk}")
            asm.add(parts[part], k(part), k(part), mods=[bevel_s], detail=1)
        d = (P["co_up"] - P["co_lo"]).normalized()
        asm.pivot(k("Coilover"), P["co_lo"], d, orient=True, parent=k("ArmLower"), motion="aim",
                  target=k("CoiloverMount"), positive=f"lokales +Y zeigt auf CoiloverMount_{wk}")
        asm.pivot(k("CoiloverMount"), P["co_up"], -d, orient=True, motion="aim", target=k("Coilover"),
                  positive=f"lokales +Y zeigt auf Coilover_{wk}")
        asm.add(parts["Coilover"], k("Coilover"), k("Coilover"), mods=[bevel_s], detail=1)
        asm.add(parts["CoiloverMount"], k("CoiloverMount"), k("CoiloverMount"), mods=[bevel_s], detail=1)
        c = g.wheel(wk)
        asm.pivot(k("Driveshaft"), P["cv_in"], (c - P["cv_in"]).normalized(), orient=True, motion="aim",
                  target=k("Wheel"), positive=f"lokales +Y zeigt auf Wheel_{wk}")
        asm.add(parts["Driveshaft"], k("Driveshaft"), k("Driveshaft"), detail=1)
        asm.pivot(k("Wheel"), c, Y_AXIS, parent=wheel_parent, positive="Rad rollt vorwärts", motion="spin")
        m = Matrix.Translation(c) if side > 0 else Matrix.Translation(c) @ Matrix.Rotation(math.pi, 4, "Z")
        wm = MeshData()
        for part in ("tire", "rim", "brake"):
            wm.merge(wheel[part], m)
        asm.add(wm, k("Wheel"), k("Wheel"), smooth=40)
        asm.add(wheel["tread"].copy().transform(m), f"Tread_{wk}", k("Wheel"), detail=1)

    # ------------------------------------------------------------- Sockets & Kollision
    asm.socket("Driver", (-0.06, 0.40, 0.92))
    asm.socket("Passenger", (-0.06, -0.40, 0.92))
    asm.socket("Rear_L", (-0.98, 0.45, 0.92))
    asm.socket("Rear_R", (-0.98, -0.45, 0.92))
    for side in (1, -1):
        asm.socket(f"Headlight_{'L' if side > 0 else 'R'}", (g.x_front + 0.02, side * 0.725, 1.011), forward=X_AXIS)
    asm.socket("ChargePort", (cp.x, cp.y + 0.035, cp.z), forward=Y_AXIS)
    asm.socket("TowFront", (2.56, 0.0, 0.50), forward=X_AXIS)
    asm.socket("TowRear", (-2.53, 0.0, 0.475), forward=-X_AXIS)
    asm.socket("Roof", (-0.79, 0.0, zp + 0.01))

    def by_pivot(*keys):
        return [o for o in asm.meshes() if o["pivot"] in keys]
    susp_parts = ("Susp", "Steer", "ArmLower", "ArmUpperFwd", "ArmUpperAft", "BallJoint", "Coilover", "CoiloverMount",
                  "Driveshaft")
    susp_keys = {f"{b}_{wk}" for wk in corners for b in susp_parts + ("Wheel",)}
    mirrors = [o for o in asm.meshes() if "_Mirror_" in o.name]
    chassis_objs = [o for o in asm.meshes() if o.name.endswith("_Chassis")]
    shell = [o for o in asm.meshes() if o["pivot"] not in susp_keys and o not in mirrors + chassis_objs]
    asm.collide("Lower", shell, lambda p: p.z <= g.z_belt + 0.03 or p.x > g.x_ws0 + 0.05, 64)
    asm.collide("Cabin", shell, lambda p: p.z >= g.z_belt - 0.03 and p.x <= g.x_ws0 + 0.05, 64)
    asm.collide("ChassisFront", chassis_objs, lambda p: p.x >= 0.84, 32)              # Hilfsrahmen + Antrieb vorn
    asm.collide("Battery", chassis_objs, lambda p: -1.03 <= p.x <= 0.90, 32)
    asm.collide("ChassisRear", chassis_objs, lambda p: p.x <= -0.97, 32)
    for wk in corners:
        asm.collide(f"Suspension_{wk}", by_pivot(*(f"{b}_{wk}" for b in susp_parts)), None, 48)
        asm.collide(f"Wheel_{wk}", by_pivot(f"Wheel_{wk}"), None, 48)
    for o in mirrors:
        asm.collide(o.name.split("_")[-2] + "_" + o.name.split("_")[-1], [o], None, 24)
    return asm


# =============================================================================
# 9) Game-Ready-Pipeline: UV-Atlas, Bake, LODs, Kollision, Rig, Export
# =============================================================================

TARGETS = {
    # rot: Drehung um Z, damit die Fahrzeugfront in der Engine nach "vorne" zeigt
    "unreal": dict(rot=0.0, fbx_scale="FBX_SCALE_UNITS"),            # +X vorne
    "unity": dict(rot=-math.pi / 2, fbx_scale="FBX_SCALE_ALL"),      # Blender -Y → Unity +Z
    "godot": dict(rot=-math.pi / 2, fbx_scale="FBX_SCALE_ALL"),      # glTF +Z vorne
    "generic": dict(rot=-math.pi / 2, fbx_scale="FBX_SCALE_ALL"),
}
LOD_RATIOS = [1.0, 0.5, 0.25, 0.12, 0.06]


def log(msg):
    print(f"[vehicle_generator] {msg}", flush=True)


def select_only(objs, active=None):
    vl = bpy.context.view_layer
    for o in vl.objects:
        o.select_set(False)
    for o in objs:
        o.select_set(True)
    vl.objects.active = active or (objs[0] if objs else None)


def op_kwargs(op, **kw):
    """Nur Parameter übergeben, die der Operator in dieser Blender-Version kennt."""
    props = op.get_rna_type().properties.keys()
    return {k: v for k, v in kw.items() if k in props}


def is_glass(obj):
    mats = [m for m in obj.data.materials if m]
    return bool(mats) and all(m.get("is_glass") for m in mats)


def tag_detail(obj):
    """Detailstufe als Face-Attribut speichern (überlebt das Zusammenführen der Meshes)."""
    me = obj.data
    attr = me.attributes.get("lod_detail") or me.attributes.new("lod_detail", "INT", "FACE")
    attr.data.foreach_set("value", [int(obj.get("detail", 0))] * len(me.polygons))


HULL_MAX_VERTS = 128       # ≤ 252 Dreiecke: Limit für konvexe MeshCollider (Unity) und PhysX
HULL_MIN_EDGE = 0.004      # kürzere Kanten der vereinfachten Hülle → Ebene weglassen (siehe convex_hull_obj)


def _hull_bm(points, merge=2e-5):
    """Konvexe Hülle (BMesh). Fast doppelte Punkte (< ``merge``) vorher zusammenfassen – sonst liefert
    Blenders Quickhull bei nahezu koplanaren Punkten gelegentlich nicht-konvexe Flächen."""
    bm = bmesh.new()
    seen = set()
    for p in points:
        k = (round(p[0] / merge), round(p[1] / merge), round(p[2] / merge))
        if k not in seen:
            seen.add(k)
            bm.verts.new(p)
    res = bmesh.ops.convex_hull(bm, input=bm.verts[:], use_existing_faces=False)
    junk = list({g for g in res["geom_interior"] + res["geom_unused"] if isinstance(g, bmesh.types.BMVert)})
    bmesh.ops.delete(bm, geom=junk, context="VERTS")
    bm.normal_update()
    return bm


def _hull_planes(bm):
    """Nach außen orientierte Flächenebenen (n, d) einer Hülle: innen gilt n·p ≤ d."""
    hc = sum((v.co for v in bm.verts), V()) / max(len(bm.verts), 1)
    planes = []
    for f in bm.faces:
        if f.calc_area() < 1e-9:
            continue
        n, d = f.normal.copy(), f.normal.dot(f.verts[0].co)
        if n.dot(hc) - d > 0:
            n, d = -n, -d
        planes.append((n, d))
    return planes


def _cut_polytope(planes, lo, hi):
    """Quader (lo, hi) mit den Halbräumen n·p ≤ d schneiden → konvexes Polyeder mit ebenen N-Ecken."""
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    for v in bm.verts:
        v.co = V([lerp(lo[i], hi[i], v.co[i] + 0.5) for i in range(3)])
    for n, d in planes:
        bmesh.ops.bisect_plane(bm, geom=bm.verts[:] + bm.edges[:] + bm.faces[:], dist=1e-7,
                               plane_co=n * d, plane_no=n, clear_outer=True)
        boundary = [e for e in bm.edges if e.is_boundary]
        if boundary:
            bmesh.ops.holes_fill(bm, edges=boundary, sides=0)
    bmesh.ops.dissolve_degenerate(bm, edges=bm.edges[:], dist=1e-6)
    bmesh.ops.delete(bm, geom=[v for v in bm.verts if not v.link_faces], context="VERTS")
    bm.normal_update()
    return bm


def convex_hull_obj(name, points, max_verts=48):
    """Konvexe Kollisionshülle, konservativ vereinfacht: umschließt garantiert alle Punkte.

    1. exakte Hülle, 2. Vertices per Raster ausdünnen (je Zelle der Punkt mit dem größten Abstand
    zum Schwerpunkt), 3. die Flächenebenen der vereinfachten Hülle so weit nach außen schieben,
    bis alle Punkte der exakten Hülle innen liegen (Schnitt der Halbräume).
    """
    bm = _hull_bm(points)
    exact = [v.co.copy() for v in bm.verts]
    if len(exact) > max_verts:
        bm.free()
        ctr = sum(exact, V()) / len(exact)
        target = max_verts
        while True:
            # 2. ausdünnen
            cell, simp = 0.02, _hull_bm(exact)
            while len(simp.verts) > target and cell < 4.0:
                grid = {}
                for p in exact:
                    k = (round(p.x / cell), round(p.y / cell), round(p.z / cell))
                    if k not in grid or (p - ctr).length > (grid[k] - ctr).length:
                        grid[k] = p
                simp.free()
                simp = _hull_bm(list(grid.values()))
                cell *= 1.35
            # 3. Ebenen (zuerst die 6 Achsen-Stützebenen gegen Spitzen) bis an die exakte Hülle schieben
            planes = [(V(d), max(V(d).dot(p) for p in exact))
                      for d in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))]
            for f in simp.faces:
                n = f.normal.copy()
                if f.calc_area() > 1e-9 and n.length > 0.5 and not any(n.dot(m) > 0.9995 for m, _ in planes):
                    planes.append((n, max(n.dot(p) for p in exact)))
            simp.free()
            lo = V([min(p[i] for p in exact) for i in range(3)]) - V((1, 1, 1))
            hi = V([max(p[i] for p in exact) for i in range(3)]) + V((1, 1, 1))
            # Das Schnitt-Polyeder ist konvex mit ebenen Flächen und enthält alle Punkte (jede Ebene ist
            # Stützebene der exakten Hülle) → direkt verwenden. Eine Neuberechnung per Quickhull knickt große,
            # fast ebene Flächen. Fast gleichzeitig schneidende Ebenen erzeugen aber Millimeter-Kanten, deren
            # Dreiecke über lange Hüllen messbar „abknicken“ – Punkte zusammenzuführen verschiebt sie aus ihren
            # Ebenen. Stattdessen die Ebene der kleinsten angrenzenden Fläche weglassen: die Hülle wird nur
            # größer (bleibt umschließend) und die Eckpunkte bleiben exakte Ebenenschnitte.
            for _ in range(len(planes)):
                bm = _cut_polytope(planes, lo, hi)
                edge = min(bm.edges, key=lambda e: e.calc_length())
                if edge.calc_length() >= HULL_MIN_EDGE:
                    break
                cand = [f for f in {f for v in edge.verts for f in v.link_faces} - set(edge.link_faces)
                        if max(abs(c) for c in f.normal) < 0.9999]               # Achsen-Stützebenen bleiben
                if not cand:
                    break
                f = min(cand, key=lambda f: f.calc_area())
                c = f.calc_center_median()
                drop = max(planes, key=lambda nd: nd[0].dot(f.normal) - abs(nd[0].dot(c) - nd[1]))
                planes.remove(drop)
                bm.free()
            if any(not e.is_manifold for e in bm.edges):                       # Notfall: Hülle der Schnittpunkte
                pts = [v.co.copy() for v in bm.verts]
                bm.free()
                bm = _hull_bm(pts)
            if len(bm.verts) <= HULL_MAX_VERTS or target <= 8:
                break
            bm.free()
            target -= 4
    bmesh.ops.triangulate(bm, faces=bm.faces[:])
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    obj = bpy.data.objects.new(name, me)
    obj.display_type = "WIRE"
    obj.hide_render = True
    return obj


def build_collision(asm, asset_name):
    coll = bpy.data.collections.new(f"{asm.name}_Collision")
    asm.coll.children.link(coll)
    hulls = []
    for i, (cname, objs, keep, max_v) in enumerate(asm.collision):
        pts = []
        for o in objs:
            mw = o.matrix_world
            for v in o.data.vertices:
                p = mw @ v.co
                if keep is None or keep(p):
                    pts.append(p)
        if len(pts) < 8:
            continue
        h = convex_hull_obj(f"UCX_{asset_name}_{i:02d}", pts, max_v)
        coll.objects.link(h)
        h.parent = asm.root
        h["collision_group"] = cname
        hulls.append(h)
    return hulls


def hull_exempt_regions(asm, hulls, angle):
    """Bewusst nicht eingehüllte Teile (``asm.hull_exempt``): jede Insel, die tatsächlich aus allen Hüllen
    ragt, als Bounding Box im Exportsystem (für den Report und tools/smoke_blender.py)."""
    planes = []
    for h in hulls:
        bm = bmesh.new()
        bm.from_mesh(h.data)
        bm.normal_update()
        planes.append(_hull_planes(bm))
        bm.free()
    rot = Matrix.Rotation(angle, 3, "Z")
    regions = []
    for objs, reason in asm.hull_exempt:
        for o in objs:
            bm = bmesh.new()
            bm.from_mesh(o.data)
            mw = o.matrix_world
            seen = set()
            for start in bm.verts:
                if start.index in seen:
                    continue
                island, stack = [], [start]
                seen.add(start.index)
                while stack:
                    v = stack.pop()
                    island.append(mw @ v.co)
                    for e in v.link_edges:
                        w = e.other_vert(v)
                        if w.index not in seen:
                            seen.add(w.index)
                            stack.append(w)
                if any(all(max(n.dot(p) - d for n, d in pl) > 1e-3 for pl in planes if pl) for p in island):
                    pts = [rot @ p for p in island]
                    regions.append(dict(part=o.name, reason=reason,
                                        min=[round(min(p[i] for p in pts), 4) for i in range(3)],
                                        max=[round(max(p[i] for p in pts), 4) for i in range(3)]))
            bm.free()
    return regions


def smart_uv(objs):
    if not objs:
        return
    select_only(objs)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.uv.smart_project(**op_kwargs(bpy.ops.uv.smart_project, angle_limit=math.radians(60), island_margin=0.0,
                                         scale_to_bounds=False))
    bpy.ops.object.mode_set(mode="OBJECT")


def scale_uv_islands(obj):
    """UV-Inseln nach Material gewichten (Material-Eigenschaft ``uv_scale``): z.B. Innenraum 0.5
    (weniger Texel für kaum sichtbare Flächen), Decals 2.0 (scharfe Schrift)."""
    me = obj.data
    factors = [float(m.get("uv_scale", 1.0)) if m else 1.0 for m in me.materials]
    if not me.uv_layers or all(abs(f - 1.0) < 1e-6 for f in factors):
        return
    bm = bmesh.new()
    bm.from_mesh(me)
    uvl = bm.loops.layers.uv.active
    bm.faces.ensure_lookup_table()
    parent = list(range(len(bm.faces)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for e in bm.edges:
        if len(e.link_loops) != 2:
            continue
        l0, l1 = e.link_loops
        a0, b0 = l0[uvl].uv, l0.link_loop_next[uvl].uv
        a1, b1 = l1[uvl].uv, l1.link_loop_next[uvl].uv
        if l0.vert == l1.vert:
            same = (a0 - a1).length < 1e-6 and (b0 - b1).length < 1e-6
        else:
            same = (a0 - b1).length < 1e-6 and (b0 - a1).length < 1e-6
        if same:
            parent[find(l0.face.index)] = find(l1.face.index)
    islands = {}
    for f in bm.faces:
        islands.setdefault(find(f.index), []).append(f)
    for faces in islands.values():
        fac = factors[faces[0].material_index] if faces[0].material_index < len(factors) else 1.0
        if abs(fac - 1.0) < 1e-6:
            continue
        loops = [lp for f in faces for lp in f.loops]
        c = V((0.0, 0.0))
        for lp in loops:
            c += lp[uvl].uv
        c /= len(loops)
        for lp in loops:
            lp[uvl].uv = c + (lp[uvl].uv - c) * fac
    bm.to_mesh(me)
    bm.free()


def pack_atlas(objs, margin=0.003):
    """Alle Teile eines Fahrzeugs in einen gemeinsamen, überlappungsfreien UV-Atlas packen."""
    select_only(objs)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.uv.select_all(action="SELECT")
    bpy.ops.uv.average_islands_scale()
    bpy.ops.object.mode_set(mode="OBJECT")
    for o in objs:
        scale_uv_islands(o)
    select_only(objs)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.uv.select_all(action="SELECT")
    bpy.ops.uv.pack_islands(**op_kwargs(bpy.ops.uv.pack_islands, margin=margin, rotate=True, shape_method="AABB",
                                        margin_method="FRACTION"))
    bpy.ops.object.mode_set(mode="OBJECT")


def join_objects(objs, name):
    if len(objs) > 1:
        others = [o.name for o in objs[1:]]
        select_only(objs, objs[0])
        bpy.ops.object.join()
        for n in others:
            ob = bpy.data.objects.get(n)
            if ob is not None:
                bpy.data.objects.remove(ob, do_unlink=True)
    target = objs[0]
    target.name = name
    target.data.name = name
    return target


def join_by_pivot(asm, meshes):
    groups = {}
    for o in meshes:
        groups.setdefault((o["pivot"], is_glass(o)), []).append(o)
    parts = []
    for (pivot, glass), objs in sorted(groups.items(), key=lambda kv: (kv[0][0] != "root", kv[0][0], kv[0][1])):
        label = "Body" if pivot == "root" else pivot
        obj = join_objects(objs, f"{asm.name}_{label}{'_Glass' if glass else ''}")
        obj["pivot"] = pivot
        parts.append(obj)
    return parts


def triangulate(obj):
    m = obj.modifiers.new("Tri", "TRIANGULATE")
    m.quad_method = "BEAUTY"
    m.ngon_method = "BEAUTY"
    if hasattr(m, "keep_custom_normals"):
        m.keep_custom_normals = True
    apply_modifiers(obj)


def clear_custom_normals(obj):
    me = obj.data
    if not me.has_custom_normals:
        return
    attr = me.attributes.get("custom_normal")          # Blender 5.x: normales Attribut
    if attr is not None:
        me.attributes.remove(attr)
    else:                                              # Blender 4.x
        with bpy.context.temp_override(object=obj, active_object=obj, selected_objects=[obj]):
            bpy.ops.mesh.customdata_custom_splitnormals_clear()


def _reorient_materials(objs, angle):
    """Prozedurale Materialien, die Welt-/Objektkoordinaten lesen (Lackstreifen, Fugen, Blattspitzen),
    nach dem Drehen auf die ursprünglichen Koordinaten zurückrechnen (wichtig bei --bake 0)."""
    mats = {m for o in objs if o.type == "MESH" for m in o.data.materials
            if m and m.get("generator") == "vehicle_generator" and not m.get("reoriented")}
    for m in mats:
        nt = m.node_tree
        for link in list(nt.links):
            fn, tn = link.from_node, link.to_node
            world = fn.type == "NEW_GEOMETRY" and link.from_socket.name == "Position"
            local = fn.type == "TEX_COORD" and link.from_socket.name == "Object" and tn.type == "SEPXYZ"
            if not (world or local):
                continue
            rot = nt.nodes.new("ShaderNodeVectorRotate")
            rot.rotation_type = "Z_AXIS"
            rot.inputs["Angle"].default_value = -angle
            src, dst = link.from_socket, link.to_socket
            nt.links.remove(link)
            nt.links.new(src, rot.inputs["Vector"])
            nt.links.new(rot.outputs["Vector"], dst)
        m["reoriented"] = True


def reorient(asm, extra, angle):
    """Fahrzeug für die Ziel-Engine drehen und Rotation in die Geometrie backen.

    Orientierte Pivots/Sockets drehen mit (ihre lokale Achse bleibt die Scharnierachse),
    alle anderen Empties behalten Einheitsrotation; die Meshes werden passend umgerechnet.
    """
    if abs(angle) < 1e-9:
        return
    R = Matrix.Rotation(angle, 4, "Z")
    objs = list(asm.coll.all_objects) + [o for o in extra if o.name not in asm.coll.all_objects]
    world = {o: o.matrix_world.copy() for o in objs}

    def depth(o):
        return 0 if o.parent is None else 1 + depth(o.parent)
    new = {}
    for o in sorted((o for o in objs if o.type != "MESH"), key=depth):
        new[o] = R @ world[o] if o.get("oriented") else Matrix.Translation(R @ world[o].translation)
        o.matrix_world = new[o]
    for o in objs:
        if o.type != "MESH":
            continue
        p1 = new.get(o.parent, o.parent.matrix_world.copy()) if o.parent else Matrix.Identity(4)
        o.data.transform(p1.inverted() @ R @ world[o])
        o.matrix_world = p1
    asm.axes = {k: (R.to_3x3() @ v) for k, v in asm.axes.items()}
    asm.socket_fwd = {k: (R.to_3x3() @ v) for k, v in asm.socket_fwd.items()}
    _reorient_materials(objs, angle)


@__import__("contextlib").contextmanager
def emission_override(mats, socket):
    """Beliebigen Principled-Eingang als Emission ausgeben (zum Backen von Farbe/Rauheit/Metall)."""
    saved = []
    for m in mats:
        nt = m.node_tree
        out = next((n for n in nt.nodes if n.type == "OUTPUT_MATERIAL"), None)
        bsdf = next((n for n in nt.nodes if n.type == "BSDF_PRINCIPLED"), None)
        if out is None:
            continue
        old = out.inputs["Surface"].links[0].from_socket if out.inputs["Surface"].is_linked else None
        em = nt.nodes.new("ShaderNodeEmission")
        em.inputs["Strength"].default_value = 1.0
        col = em.inputs["Color"]
        if bsdf is not None:
            src = bsdf.inputs[socket]
            if socket == "Emission Color" and bsdf.inputs["Emission Strength"].default_value <= 0.0:
                col.default_value = (0.0, 0.0, 0.0, 1.0)
            elif src.is_linked:
                nt.links.new(src.links[0].from_socket, col)
            else:
                v = src.default_value
                col.default_value = (v, v, v, 1.0) if isinstance(v, float) else tuple(v)
        nt.links.new(em.outputs[0], out.inputs["Surface"])
        saved.append((nt, em, old, out))
    try:
        yield
    finally:
        for nt, em, old, out in saved:
            nt.nodes.remove(em)
            if old is not None:
                nt.links.new(old, out.inputs["Surface"])


def _pixels(img):
    import numpy as np
    w, h = img.size
    a = np.empty(w * h * 4, dtype=np.float32)
    img.pixels.foreach_get(a)
    return a.reshape(h, w, 4)


def _save_image(name, arr, path, colorspace="Non-Color"):
    h, w = arr.shape[:2]
    img = bpy.data.images.new(name, w, h, alpha=True)
    img.colorspace_settings.name = colorspace
    img.pixels.foreach_set(arr.reshape(-1))
    img.filepath_raw = path
    img.file_format = "PNG"
    img.save()
    return img


def gltf_output_group():
    """Node-Gruppe "glTF Material Output" – damit der glTF-Exporter die AO-Map mitnimmt."""
    ng = bpy.data.node_groups.get("glTF Material Output")
    if ng is None:
        ng = bpy.data.node_groups.new("glTF Material Output", "ShaderNodeTree")
        ng.interface.new_socket("Occlusion", in_out="INPUT", socket_type="NodeSocketFloat")
        ng.nodes.new("NodeGroupInput")
    return ng


def bake_textures(asm, objs, cfg, tex_dir):
    """Prozedurale Materialien in PBR-Texturen backen und ein einziges Engine-Material erzeugen."""
    import numpy as np
    os.makedirs(tex_dir, exist_ok=True)
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    if cfg.gpu:
        sc.cycles.device = "GPU"
    sc.render.bake.margin = 16
    sc.render.bake.margin_type = "EXTEND"
    if sc.world is None:
        sc.world = bpy.data.worlds.new("BakeWorld")
    sc.world.light_settings.distance = asm.ao_distance     # lokale AO statt Szenen-Verschattung
    res = cfg.bake
    short = asm.name[3:] if asm.name.startswith("SM_") else asm.name
    channels = [("normal", "NORMAL", None), ("ao", "AO", None), ("base", "EMIT", "Base Color"),
                ("rough", "EMIT", "Roughness"), ("metal", "EMIT", "Metallic"), ("emit", "EMIT", "Emission Color")]
    mats = sorted({m for o in objs for m in o.data.materials if m and not m.get("is_glass")}, key=lambda m: m.name)
    for m in mats:
        node = m.node_tree.nodes.new("ShaderNodeTexImage")
        node.name = "_bake_target"
    # Alle Teile als EIN temporäres Objekt backen: Blender wendet den Bake-Rand pro Objekt an – bei
    # mehreren Objekten überschreibt der Rand eines Teils sonst bereits gebackene Texel des nächsten.
    # AO „auseinandergezogen“: Jede bewegliche Pivot-Gruppe liegt nur im AO-Durchgang weit entfernt – sonst
    # dreht ein Rad die Verschattung des Radhauses mit, offene Türen/Klappen zeigen die Schatten des
    # geschlossenen Zustands und Teile unter einer Klappe (Motorraum) wären schwarz.
    moving = sorted({o.get("pivot", "root") for o in objs} - {"root"})
    shift = {p: (30.0 * (1 + i % 5), 30.0 * (i // 5), 0.0) for i, p in enumerate(moving)}
    copies = []
    for o in objs:
        c = o.copy()
        c.data = o.data.copy()
        asm.coll.objects.link(c)
        if o.get("pivot", "root") in shift:
            a = c.data.attributes.new("_explode", "FLOAT_VECTOR", "POINT")
            a.data.foreach_set("vector", shift[o["pivot"]] * len(c.data.vertices))
        copies.append(c)
    tmp = join_objects(copies, "_bake_tmp")
    explode = rest = None
    attr = tmp.data.attributes.get("_explode")
    if attr is not None:
        n = len(tmp.data.vertices)
        explode, rest = np.zeros(n * 3, np.float32), np.zeros(n * 3, np.float32)
        attr.data.foreach_get("vector", explode)
        tmp.data.attributes.remove(attr)
        inv = np.array(tmp.matrix_world.inverted().to_3x3(), np.float32)        # Welt → Objektraum
        explode = (explode.reshape(-1, 3) @ inv.T).ravel()
        tmp.data.vertices.foreach_get("co", rest)
    for o in objs:
        o.hide_render = True           # Originale verdecken sonst die AO der deckungsgleichen Kopie
    select_only([tmp])
    imgs = {}
    for key, kind, socket in channels:
        t = time.time()
        img = bpy.data.images.new(f"_bake_{key}", res, res, alpha=False, float_buffer=True)
        img.colorspace_settings.name = "Non-Color"   # Rohwerte (linear); sRGB-Kodierung erfolgt beim Speichern
        for m in mats:
            n = m.node_tree.nodes["_bake_target"]
            n.image = img
            m.node_tree.nodes.active = n
        sc.cycles.samples = cfg.bake_samples if kind == "AO" else 4
        kw = dict(type=kind, margin=16, use_clear=True)
        if kind == "NORMAL":
            kw.update(normal_space="TANGENT")
        bumps = []
        if kind == "AO":
            # rein geometrische AO: Mikro-Bump (steckt in der Normal-Map) hängt von der Position ab
            bumps = [(nd_, nd_.inputs["Strength"].default_value) for m in mats for nd_ in m.node_tree.nodes
                     if nd_.type == "BUMP"]
            for nd_, _ in bumps:
                nd_.inputs["Strength"].default_value = 0.0
            if explode is not None:
                tmp.data.vertices.foreach_set("co", rest + explode)
                tmp.data.update()
        if socket:
            with emission_override(mats, socket):
                bpy.ops.object.bake(**op_kwargs(bpy.ops.object.bake, **kw))
        else:
            bpy.ops.object.bake(**op_kwargs(bpy.ops.object.bake, **kw))
        if kind == "AO":
            for nd_, strength in bumps:
                nd_.inputs["Strength"].default_value = strength
            if explode is not None:
                tmp.data.vertices.foreach_set("co", rest)
                tmp.data.update()
        imgs[key] = img
        log(f"  bake {key:6s} {res}px  {time.time() - t:5.1f}s")
    for m in mats:
        m.node_tree.nodes.remove(m.node_tree.nodes["_bake_target"])
    tmp_me = tmp.data
    bpy.data.objects.remove(tmp, do_unlink=True)
    bpy.data.meshes.remove(tmp_me)
    for o in objs:
        o.hide_render = False

    px = {k: _pixels(v) for k, v in imgs.items()}
    one = np.ones(px["ao"].shape[:2], dtype=np.float32)
    files = {}

    def save(suffix, arr, cs="Non-Color"):
        path = os.path.join(tex_dir, f"T_{short}_{suffix}.png")
        files[suffix] = _save_image(f"T_{short}_{suffix}", np.clip(arr, 0, 1), path, cs)
        return files[suffix]

    def to_srgb(c):
        return np.where(c <= 0.0031308, c * 12.92, 1.055 * np.power(np.maximum(c, 0), 1 / 2.4) - 0.055)

    base = px["base"].copy()
    base[..., :3] = to_srgb(base[..., :3])
    base[..., 3] = 1.0
    save("BaseColor", base, "sRGB")
    save("Normal", px["normal"])
    dx = px["normal"].copy()
    dx[..., 1] = 1.0 - dx[..., 1]
    if cfg.target == "unreal":
        save("Normal_DX", dx)
    orm = np.stack([px["ao"][..., 0], px["rough"][..., 0], px["metal"][..., 0], one], -1)
    save("ORM", orm)
    if cfg.target == "unity":
        save("MetallicSmoothness", np.stack([px["metal"][..., 0], px["metal"][..., 0], px["metal"][..., 0],
                                             1.0 - px["rough"][..., 0]], -1))
        save("MaskMap", np.stack([px["metal"][..., 0], px["ao"][..., 0], np.zeros_like(one),
                                  1.0 - px["rough"][..., 0]], -1))
    has_emit = float(px["emit"][..., :3].max()) > 0.01
    if has_emit:
        em = px["emit"].copy()
        em[..., :3] = to_srgb(em[..., :3])
        em[..., 3] = 1.0
        save("Emissive", em, "sRGB")
    for img in imgs.values():
        bpy.data.images.remove(img)

    # Engine-Material (ein Material + Glas); geschlossene Hüllen → einseitig rendern
    m, nd = new_mat(f"M_{short}")
    m.use_backface_culling = True
    bsdf = _principled(nd)
    uv = nd.new("ShaderNodeUVMap")
    uv.uv_map = "UVMap"

    def tex(key, cs):
        n = nd.new("ShaderNodeTexImage")
        n.image = files[key]
        n.image.colorspace_settings.name = cs
        nd.link(uv.outputs[0], n.inputs["Vector"])
        return n
    nd.link(tex("BaseColor", "sRGB").outputs["Color"], bsdf.inputs["Base Color"])
    sep = nd.new("ShaderNodeSeparateColor")
    orm_n = tex("ORM", "Non-Color")
    nd.link(orm_n.outputs["Color"], sep.inputs["Color"])
    nd.link(sep.outputs["Green"], bsdf.inputs["Roughness"])
    nd.link(sep.outputs["Blue"], bsdf.inputs["Metallic"])
    nm = nd.new("ShaderNodeNormalMap")
    nd.link(tex("Normal", "Non-Color").outputs["Color"], nm.inputs["Color"])
    nd.link(nm.outputs["Normal"], bsdf.inputs["Normal"])
    if has_emit:
        nd.link(tex("Emissive", "sRGB").outputs["Color"], bsdf.inputs["Emission Color"])
        bsdf.inputs["Emission Strength"].default_value = 4.0
    grp = nd.new("ShaderNodeGroup")
    grp.node_tree = gltf_output_group()
    nd.link(sep.outputs["Red"], grp.inputs["Occlusion"])
    for o in objs:
        me = o.data
        old = list(me.materials)
        slot_map, new_slots = {}, []
        for i, om in enumerate(old):
            tgt = om if (om and om.get("is_glass")) else m
            if tgt not in new_slots:
                new_slots.append(tgt)
            slot_map[i] = new_slots.index(tgt)
        idx = [0] * len(me.polygons)
        me.polygons.foreach_get("material_index", idx)
        me.materials.clear()
        for t in new_slots:
            me.materials.append(t)
        me.polygons.foreach_set("material_index", [slot_map.get(i, 0) for i in idx])
    return {k: os.path.relpath(bpy.path.abspath(v.filepath_raw), os.path.dirname(tex_dir)) for k, v in files.items()}


def build_lods(asm, parts, n):
    lods = {0: parts}
    for L in range(1, n + 1):
        coll = bpy.data.collections.new(f"{asm.name}_LOD{L}")
        asm.coll.children.link(coll)
        out = []
        for o in parts:
            me = o.data.copy()
            bm = bmesh.new()
            bm.from_mesh(me)
            layer = bm.faces.layers.int.get("lod_detail")
            if layer is not None:
                kill = [f for f in bm.faces if f[layer] > max(0, 3 - L)]
                bmesh.ops.delete(bm, geom=kill, context="FACES")
            bm.to_mesh(me)
            bm.free()
            if not me.polygons:
                bpy.data.meshes.remove(me)
                continue
            c = o.copy()
            c.data = me
            c.name = me.name = f"{o.name}_LOD{L}"
            coll.objects.link(c)
            c.parent = o.parent
            c.matrix_world = o.matrix_world.copy()
            ratio = LOD_RATIOS[min(L, len(LOD_RATIOS) - 1)]
            dec = c.modifiers.new("Decimate", "DECIMATE")
            dec.ratio = ratio
            dec.use_collapse_triangulate = True
            apply_modifiers(c)
            c.data.validate(clean_customdata=False)        # Decimate kann doppelte Faces erzeugen
            clear_custom_normals(c)
            set_smoothing(c, 50)
            out.append(c)
        lods[L] = out
    return lods


def build_rig(asm, meshes, short):
    """Skelett aus den Pivots + ein starr gewichtetes Skeletal Mesh (z.B. Chaos Vehicle, Unity Animator)."""
    data = bpy.data.armatures.new(f"{short}_Skeleton")
    arm = bpy.data.objects.new("Armature", data)
    rig_coll = bpy.data.collections.new(f"{asm.name}_Rig")
    asm.coll.children.link(rig_coll)
    rig_coll.objects.link(arm)
    select_only([arm])
    bpy.ops.object.mode_set(mode="EDIT")
    bones = {}

    def depth(k):
        return 0 if asm.parents[k] is None else 1 + depth(asm.parents[k])
    for key in sorted(asm.pivots, key=depth):
        b = data.edit_bones.new("root" if key == "root" else key)
        head = asm.pivots[key].matrix_world.translation.copy()
        b.head = head
        b.tail = head + asm.axes[key] * (0.4 if key == "root" else 0.18)
        if asm.parents[key] is not None:
            b.parent = bones[asm.parents[key]]
        bones[key] = b
    bpy.ops.object.mode_set(mode="OBJECT")
    copies = []
    for o in meshes:
        c = o.copy()
        c.data = o.data.copy()
        rig_coll.objects.link(c)
        c.parent = None
        c.matrix_world = o.matrix_world.copy()
        c.data.transform(c.matrix_world)
        c.matrix_world = Matrix.Identity(4)
        vg = c.vertex_groups.new(name="root" if o["pivot"] == "root" else o["pivot"])
        vg.add(range(len(c.data.vertices)), 1.0, "REPLACE")
        copies.append(c)
    sk = join_objects(copies, f"SK_{short}")
    sk.parent = arm
    mod = sk.modifiers.new("Armature", "ARMATURE")
    mod.object = arm
    return arm, sk


def export_fbx(path, objs, tgt):
    select_only(objs)
    kw = dict(filepath=path, use_selection=True, object_types={"EMPTY", "MESH", "ARMATURE"},
              apply_scale_options=tgt["fbx_scale"], mesh_smooth_type="FACE", use_tspace=True,
              use_custom_props=True, add_leaf_bones=False, path_mode="RELATIVE", embed_textures=False,
              bake_anim=False, axis_forward="-Z", axis_up="Y", use_mesh_modifiers=True,
              primary_bone_axis="Y", secondary_bone_axis="X", armature_nodetype="NULL")
    bpy.ops.export_scene.fbx(**op_kwargs(bpy.ops.export_scene.fbx, **kw))


def export_glb(path, objs):
    select_only(objs)
    kw = dict(filepath=path, export_format="GLB", use_selection=True, export_yup=True, export_apply=True,
              export_tangents=True, export_normals=True, export_texcoords=True, export_materials="EXPORT",
              export_extras=True, export_cameras=False, export_lights=False, export_skins=True,
              export_animations=False, export_image_format="AUTO")
    bpy.ops.export_scene.gltf(**op_kwargs(bpy.ops.export_scene.gltf, **kw))


def tri_count(objs):
    n = 0
    for o in objs:
        me = o.data
        me.calc_loop_triangles()
        n += len(me.loop_triangles)
    return n


REPORT_SCHEMA = 2
TEXTURE_KEYS = ("BaseColor", "Normal", "Normal_DX", "ORM", "MetallicSmoothness", "MaskMap", "Emissive")


def file_sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def remove_old_outputs(asm, short, out_dir):
    """Bekannte Ausgabedateien dieses Assets vor dem Export löschen – zuerst den Report, damit ein
    abgebrochener Lauf nicht vollständig aussieht und keine veralteten LODs/Formate liegen bleiben."""
    names = [f"{asm.name}_report.json"]
    names += [f"{asm.name}{suffix}.{ext}" for suffix in ["", *(f"_LOD{i}" for i in range(1, 5)), "_Collision"]
              for ext in ("fbx", "glb")]
    names += [f"SK_{short}.fbx", f"SK_{short}.glb", f"{asm.name}.blend"]
    names += [os.path.join("Textures", f"T_{short}_{k}.png") for k in TEXTURE_KEYS]
    for n in names:
        path = os.path.join(out_dir, n)
        if os.path.isfile(path):
            os.remove(path)


def finalize(asm, cfg, out_dir):
    """Rohmodell → engine-fertiges Asset (siehe Modul-Docstring)."""
    t0 = time.time()
    tgt = TARGETS[cfg.target]
    short = asm.name[3:] if asm.name.startswith("SM_") else asm.name
    os.makedirs(out_dir, exist_ok=True)
    remove_old_outputs(asm, short, out_dir)
    bpy.context.view_layer.update()
    for k, e in asm.sockets.items():
        e.name = f"SOCKET_{k}"
    if 0 < cfg.bake < 1024:
        log(f"Warnung: --bake {cfg.bake} ist sehr klein – Decals und Kleinteile werden im Atlas unscharf.")
    meshes = asm.meshes()
    for o in meshes:
        # Solidify (identische UVs innen/außen) und Bevel (Facetten über UV-Nähte) → neu abwickeln
        if any(m.type in ("SOLIDIFY", "BEVEL") for m in o.modifiers):
            o["uv_ready"] = False
        apply_modifiers(o)
        tag_detail(o)
    hulls = build_collision(asm, asm.name) if cfg.collision else []     # UCX_<Asset>_## (Unreal-Konvention)
    exempt = hull_exempt_regions(asm, hulls, tgt["rot"]) if hulls else []
    if cfg.target == "godot":
        for h in hulls:
            h.name += "-convcolonly"        # Godot-Importhinweis: konvexe Kollisionsform, Mesh wird verworfen
    log(f"{short}: {len(meshes)} Teile, {len(hulls)} Kollisionshüllen")
    smart_uv([o for o in meshes if not o.get("uv_ready")])
    parts = join_by_pivot(asm, meshes)
    opaque = [o for o in parts if not is_glass(o)]
    pack_atlas(opaque, 0.003 if cfg.bake >= 2048 else 0.004 if cfg.bake >= 1024 else 0.006)
    textures = {}
    if cfg.bake > 0:
        log(f"{short}: Textur-Bake ({cfg.bake}px) …")
        textures = bake_textures(asm, opaque, cfg, os.path.join(out_dir, "Textures"))
    for o in parts:
        triangulate(o)
    reorient(asm, hulls, tgt["rot"])
    lods = build_lods(asm, parts, cfg.lods)
    log(f"{short}: LODs " + ", ".join(f"LOD{k}={tri_count(v):,}".replace(",", ".") for k, v in lods.items()))
    hierarchy = [asm.root] + [p for k, p in asm.pivots.items() if k != "root"] + list(asm.sockets.values())
    files, lod_files, collision_files = [], {L: [] for L in lods}, []
    base = os.path.join(out_dir, asm.name)
    for L, objs in lods.items():
        suffix = "" if L == 0 else f"_LOD{L}"
        if "fbx" in cfg.formats:
            ucx = hulls if (L == 0 and cfg.target == "unreal") else []      # UCX ist eine Unreal-Konvention
            export_fbx(base + suffix + ".fbx", hierarchy + objs + ucx, tgt)
            lod_files[L].append(base + suffix + ".fbx")
            if ucx:
                collision_files.append(base + ".fbx")
            elif L == 0 and hulls:
                export_fbx(base + "_Collision.fbx", [asm.root] + hulls, tgt)
                collision_files.append(base + "_Collision.fbx")
        if "glb" in cfg.formats:
            export_glb(base + suffix + ".glb", hierarchy + objs)
            lod_files[L].append(base + suffix + ".glb")
            if L == 0 and hulls:                    # glTF kennt keine UCX-Konvention → Hüllen immer separat
                export_glb(base + "_Collision.glb", [asm.root] + hulls)
                collision_files.append(base + "_Collision.glb")
        files += lod_files[L]
    files += [f for f in collision_files if f not in files]
    bones = []
    if cfg.rig:
        arm, sk = build_rig(asm, lods[0], short)
        bones = [b.name for b in arm.data.bones]
        if "fbx" in cfg.formats:
            export_fbx(os.path.join(out_dir, f"SK_{short}.fbx"), [arm, sk], tgt)
            files.append(os.path.join(out_dir, f"SK_{short}.fbx"))
        if "glb" in cfg.formats:
            export_glb(os.path.join(out_dir, f"SK_{short}.glb"), [arm, sk])
            files.append(os.path.join(out_dir, f"SK_{short}.glb"))
    if "blend" in cfg.formats:
        # LODs, Kollision und Rig-Kopie liegen deckungsgleich auf LOD0 → im Viewport ausblenden
        def hide(layer):
            for child in layer.children:
                if child.name.startswith(asm.name + "_"):
                    child.hide_viewport = True
                hide(child)
        hide(bpy.context.view_layer.layer_collection)
        path = os.path.join(out_dir, f"{asm.name}.blend")
        bpy.ops.wm.save_as_mainfile(filepath=os.path.abspath(path), relative_remap=True, compress=True)
        files.append(path)
    pts = [o.matrix_world @ v.co for o in lods[0] for v in o.data.vertices]
    size = [max(p[i] for p in pts) - min(p[i] for p in pts) for i in range(3)]

    def rel(f):
        return os.path.relpath(f, out_dir).replace(os.sep, "/")
    seconds = round(time.time() - t0, 1)
    report = dict(
        schema_version=REPORT_SCHEMA, asset=asm.name,
        generator="vehicle_generator.py", generator_sha256=GENERATOR_SHA256,
        blender_version=bpy.app.version_string,
        options={k: (sorted(v) if isinstance(v, set) else v) for k, v in sorted(vars(cfg).items())},
        target=cfg.target, forward_axis="+X" if tgt["rot"] == 0 else "-Y (Blender)",
        units="Meter", dimensions_m=[round(s, 3) for s in size],
        lods={f"LOD{k}": dict(triangles=tri_count(v), meshes=[o.name for o in v], files=[rel(f) for f in lod_files[k]])
              for k, v in lods.items()},
        pivots={k: dict(location=[round(c, 4) for c in p.matrix_world.translation], axis=[round(c, 4) for c in asm.axes[k]],
                        motion=asm.motion.get(k, "hinge"), parent=asm.parents[k], oriented=bool(p.get("oriented")),
                        positive=asm.notes.get(k, ""), **({"target": asm.targets[k]} if k in asm.targets else {}),
                        **({"limits": list(asm.limits[k])} if k in asm.limits else {}))
                for k, p in asm.pivots.items()},
        sockets={k: dict(location=[round(c, 4) for c in s.matrix_world.translation],
                         forward=[round(c, 4) for c in asm.socket_fwd[k]]) for k, s in asm.sockets.items()},
        collision=[dict(name=h.name, group=h["collision_group"], verts=len(h.data.vertices)) for h in hulls],
        collision_files=[rel(f) for f in collision_files],
        collision_exempt=dict(pivots=dict(asm.hull_exempt_pivots), regions=exempt) if hulls else {},
        textures=textures, bones=bones,
        materials=sorted({m.name for o in lods[0] for m in o.data.materials if m}),
        files=[dict(path=rel(f), bytes=os.path.getsize(f), sha256=file_sha256(f))
               for f in files + [os.path.join(out_dir, t) for t in textures.values()]],
        seconds=seconds,
        provenance="Vollständig prozedural erzeugt (Geometrie, Materialien, Texturen); keine externen Modelle oder Bilder.",
    )
    with open(os.path.join(out_dir, f"{asm.name}_report.json"), "w", encoding="utf-8") as fh:
        json.dump(report, fh, indent=2, ensure_ascii=False)
    log(f"{short}: fertig in {report['seconds']}s → {out_dir}")
    return report


# =============================================================================
# 10) Kommandozeile
# =============================================================================

def parse_args(argv):
    p = argparse.ArgumentParser(
        prog="vehicle_generator.py",
        description="Prozedurale, game-ready 3D-Fahrzeugmodelle: Enduro, Hubschrauber, Flugzeug, Geländewagen, "
                    "Motorboot und Elektro-SUV (Blender/bpy).")
    p.add_argument("--vehicle", choices=["enduro", "heli", "plane", "offroad", "boat", "suv", "all"], default="all")
    p.add_argument("--out", default="export", help="Ausgabeordner (Standard: ./export)")
    p.add_argument("--target", choices=sorted(TARGETS), default="unreal",
                   help="Ziel-Engine: steuert Ausrichtung, FBX-Skalierung und Textur-Packing")
    p.add_argument("--formats", default="fbx,glb,blend", help="Kommagetrennt: fbx, glb, blend")
    p.add_argument("--bake", type=int, default=2048, help="Texturauflösung für den PBR-Bake (0 = kein Bake, sonst 256–8192)")
    p.add_argument("--bake-samples", type=int, default=32, help="Samples für den AO-Bake")
    p.add_argument("--lods", type=int, default=3, choices=range(0, 5), metavar="0-4",
                   help="Anzahl zusätzlicher LOD-Stufen")
    p.add_argument("--rig", action="store_true", help="Zusätzlich Skeletal Mesh (SK_*) mit Knochen exportieren")
    p.add_argument("--no-collision", dest="collision", action="store_false", help="Keine UCX-Kollision erzeugen")
    p.add_argument("--preview", action="store_true", help="Studio-Vorschaubilder rendern (Cycles)")
    p.add_argument("--preview-samples", type=int, default=96)
    p.add_argument("--gpu", action="store_true", help="Cycles auf der GPU (Bake/Preview)")
    p.add_argument("--seed", type=int, default=7, help="Zufallsstartwert (Stollenmuster, Variation)")
    p.add_argument("--dirt", type=float, default=0.35, help="Verschmutzungsgrad 0–1")
    p.add_argument("--enduro-color", choices=sorted(ENDURO_PALETTES), default="orange")
    p.add_argument("--heli-livery", dest="heli_color", choices=sorted(HELI_LIVERIES), default="civil")
    p.add_argument("--plane-livery", dest="plane_color", choices=sorted(PLANE_LIVERIES), default="classic")
    p.add_argument("--offroad-livery", dest="offroad_color", choices=sorted(OFFROAD_LIVERIES), default="sand")
    p.add_argument("--boat-livery", dest="boat_color", choices=sorted(BOAT_LIVERIES), default="navy")
    p.add_argument("--suv-livery", dest="suv_color", choices=sorted(SUV_LIVERIES), default="forest")
    p.add_argument("--suv-badge", default="4X4 ELECTRIC", help="Schriftzug auf der Heckklappe des SUV")
    p.add_argument("--blades", type=int, default=4, choices=range(2, 8), metavar="2-7",
                   help="Anzahl Hauptrotorblätter")
    p.add_argument("--enduro-name", default="SM_Enduro")
    p.add_argument("--heli-name", default="SM_Heli")
    p.add_argument("--plane-name", default="SM_Plane")
    p.add_argument("--offroad-name", default="SM_Offroad")
    p.add_argument("--boat-name", default="SM_Boat")
    p.add_argument("--suv-name", default="SM_SUV")
    p.add_argument("--no-export", action="store_true", help="Nur Modelle bauen (z.B. in der Blender-GUI)")
    cfg = p.parse_args(argv)
    cfg.formats = {f.strip().lower() for f in cfg.formats.split(",") if f.strip()}
    unknown = cfg.formats - {"fbx", "glb", "blend"}
    if unknown:
        p.error(f"--formats: unbekannt {', '.join(sorted(unknown))} (erlaubt: fbx, glb, blend)")
    if cfg.bake and not 256 <= cfg.bake <= 8192:
        p.error("--bake muss 0 (kein Bake) oder 256–8192 sein")
    for opt in ("bake_samples", "preview_samples"):
        if not 1 <= getattr(cfg, opt) <= 4096:
            p.error(f"--{opt.replace('_', '-')} muss 1–4096 sein")
    cfg.dirt = clamp(cfg.dirt)
    return cfg


def reset_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    sc.unit_settings.system = "METRIC"
    sc.unit_settings.scale_length = 1.0
    sc.render.engine = "CYCLES"


def main():
    if "--" in sys.argv:
        argv = sys.argv[sys.argv.index("--") + 1:]
    elif os.path.basename(sys.argv[0]).endswith(".py"):
        argv = sys.argv[1:]
    else:
        argv = []
    gui = not bpy.app.background
    cfg = parse_args(argv)
    if bpy.app.version < (4, 2, 0):
        raise SystemExit(f"Blender {bpy.app.version_string} ist zu alt – benötigt wird Blender/bpy 4.2 oder neuer.")
    if gui:
        cfg.no_export = True
    out_dir = os.path.abspath(cfg.out)
    summary = os.path.join(out_dir, "summary.json")
    if not cfg.no_export and os.path.isfile(summary):
        os.remove(summary)                   # erst nach erfolgreichem Lauf neu schreiben
    builders = {"enduro": build_enduro, "heli": build_heli, "plane": build_plane, "offroad": build_offroad,
                "boat": build_boat, "suv": build_suv}
    names = {"enduro": cfg.enduro_name, "heli": cfg.heli_name, "plane": cfg.plane_name, "offroad": cfg.offroad_name,
             "boat": cfg.boat_name, "suv": cfg.suv_name}
    gui_y = {"enduro": 0.0, "heli": 8.0, "plane": 22.0, "offroad": 31.0, "boat": 38.0, "suv": 45.0}   # GUI: nebeneinander
    keys = list(builders) if cfg.vehicle == "all" else [cfg.vehicle]
    reports = []
    for key in keys:
        t = time.time()
        if gui:
            old = bpy.data.collections.get(names[key])
            if old:
                for o in list(old.all_objects):
                    bpy.data.objects.remove(o, do_unlink=True)
                bpy.data.collections.remove(old)
        else:
            reset_scene()
        asm = builders[key](cfg)
        log(f"{key}: Modell gebaut in {time.time() - t:.1f}s ({len(asm.meshes())} Teile)")
        if cfg.preview:
            os.makedirs(os.path.join(out_dir, "previews"), exist_ok=True)
            for vname, view in (("34", (-35.0, 16.0)), ("side", (90.0, 5.0)), ("rear", (150.0, 22.0))):
                render_preview(asm.meshes(), os.path.join(out_dir, "previews", f"{asm.name}_{vname}.png"),
                               samples=cfg.preview_samples, view=view)
        if gui:
            asm.root.location.y = gui_y[key] if len(keys) > 1 else 0.0
            continue
        if not cfg.no_export:
            reports.append(finalize(asm, cfg, out_dir))
    if reports:
        with open(summary, "w", encoding="utf-8") as fh:
            json.dump(reports, fh, indent=2, ensure_ascii=False)
    return reports


if __name__ == "__main__":
    main()
