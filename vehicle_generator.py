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

Game-Ready-Pipeline:
  reale Maße (Meter) · saubere Pivots für alle beweglichen Teile · optionales Skelett
  (Skeletal Mesh) · UV-Atlas · Textur-Bake (BaseColor / ORM / Normal / Emissive) ·
  LOD-Kette · UCX-Kollision · Export als FBX / GLB / .blend · JSON-Report

Aufruf (Beispiele):
  blender --background --python vehicle_generator.py -- --vehicle all --out export
  blender -b -P vehicle_generator.py -- --vehicle heli --bake 2048 --rig --target unreal
  python vehicle_generator.py --vehicle enduro --preview          (mit ``pip install bpy``)

In der Blender-Oberfläche (Scripting-Tab → Öffnen → Ausführen) werden beide Modelle
ohne Export direkt in der aktuellen Szene erzeugt.

Konventionen beim Modellieren: +X = vorne, +Y = links, +Z = oben, Ursprung am Boden.
Die Ausrichtung für die Ziel-Engine wird erst beim Export angewendet (``--target``).
"""

import argparse
import bisect
import json
import math
import os
import random
import sys
import time

import bpy
import bmesh
from mathutils import Matrix, Quaternion, Vector
from mathutils.bvhtree import BVHTree
from mathutils.interpolate import poly_3d_calc

V = Vector
TAU = math.tau
X_AXIS, Y_AXIS, Z_AXIS = V((1, 0, 0)), V((0, 1, 0)), V((0, 0, 1))


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


def new_mat(name):
    m = bpy.data.materials.new(name)
    m["generator"] = "vehicle_generator"
    return m, Nodes(m)


def mat_paint(name, color, rough=0.35, coat=0.0, metallic=0.0, dirt=0.0, dirt_z=(0.15, 0.7), livery=None,
              seams=None):
    """Lack/glänzender Kunststoff. ``livery``: [(z_von, z_bis, farbe)] Farbstreifen über Welt-Z,
    ``seams``: gravierte Panel-Fugen (siehe _seam_mask)."""
    m, nd = new_mat(name)
    bsdf = _principled(nd)
    col = None
    if livery:
        col = tuple(color)
        z = nd.world_z()
        for z0, z1, c in livery:
            mask = nd.math("MULTIPLY", nd.map_range(z, z0 - 0.004, z0 + 0.004),
                           nd.map_range(z, z1 - 0.004, z1 + 0.004, 1.0, 0.0))
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


def mat_rotor_blade(name, color, bands, rough=0.4):
    """Rotorblatt-Lack mit Warnstreifen ``bands`` = [(r_von, r_bis, farbe)];
    Maske über den radialen Abstand zum Objekt-Ursprung (= Rotornabe)."""
    m, nd = new_mat(name)
    bsdf = _principled(nd)
    co = nd.coords("Object")
    sep = nd.new("ShaderNodeSeparateXYZ")
    nd.link(co, sep.inputs[0])
    comb = nd.new("ShaderNodeCombineXYZ")
    nd.link(sep.outputs["X"], comb.inputs["X"])
    nd.link(sep.outputs["Y"], comb.inputs["Y"])
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
        self.ao_distance = 0.3
        self.sockets = {}

    def _empty(self, name, loc, size=0.15, kind="PLAIN_AXES"):
        e = bpy.data.objects.new(name, None)
        e.empty_display_type = kind
        e.empty_display_size = size
        e.location = loc
        self.coll.objects.link(e)
        return e

    def pivot(self, key, loc, axis=Z_AXIS, parent="root"):
        e = self._empty(f"{self.name}_{key}_Pivot", V(loc), 0.12, "ARROWS")
        par = self.pivots[parent]
        e.parent = par
        e.matrix_world = Matrix.Translation(V(loc))
        self.pivots[key] = e
        self.axes[key] = V(axis).normalized()
        self.parents[key] = parent
        return e

    def socket(self, key, loc):
        """Andockpunkte für die Engine (Fahrerposition, Kamera, Licht …)."""
        e = self._empty(f"SOCKET_{key}", V(loc), 0.08, "SPHERE")
        e.parent = self.root
        e.matrix_world = Matrix.Translation(V(loc))
        self.sockets[key] = e
        return e

    def add(self, md_or_obj, name, pivot="root", smooth=35.0, mods=(), detail=0):
        obj = md_or_obj.to_object(f"{self.name}_{name}", smooth) if isinstance(md_or_obj, MeshData) else md_or_obj
        if obj.name not in self.coll.objects:
            self.coll.objects.link(obj)
        p = self.pivots[pivot]
        pw = p.matrix_world.translation.copy()
        obj.data.transform(Matrix.Translation(-pw))
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
    for o in objects:
        if o.type == "MESH":
            pts.extend(o.matrix_world @ V(c) for c in o.bound_box)
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
    cam = bpy.data.objects.new("PreviewCam", cam_d)
    sc.collection.objects.link(cam)
    temp.append(cam)
    az, el = math.radians(view[0]), math.radians(view[1])
    dirv = V((math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)))
    fov = 2 * math.atan(36.0 / 2 / lens) * min(1.0, resolution[1] / resolution[0] * 1.25)
    dist = radius / math.sin(fov / 2) * 0.92 * dist_scale
    cam.location = ctr + dirv * dist
    cam.rotation_euler = (-dirv).to_track_quat("-Z", "Y").to_euler()
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
    top, bottom = np.array([0.80, 0.82, 0.85]), np.array([0.94, 0.94, 0.95])
    bg = bottom + (top - bottom) * (yy[..., None] ** 1.3)
    bg = bg * (1.0 - 0.10 * (xx[..., None] ** 2))
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


def brake_disc(M, center, y, R, waves=18):
    m = disc_matrix(center, y)
    outer = [V((math.cos(t), math.sin(t))) * (R - 0.006 * (0.5 + 0.5 * math.cos(waves * t)))
             for t in (TAU * i / (waves * 6) for i in range(waves * 6))]
    inner = circle_2d(R - 0.032, waves * 6)
    md = extrude_ring(outer, inner, 0.004, m, M["steel"])
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
    asm.pivot("Steering", g.steer(0.76), g.up)
    asm.pivot("ForkLower", g.FA, g.up, parent="Steering")
    asm.pivot("WheelFront", g.FA, Y_AXIS, parent="ForkLower")
    asm.pivot("Swingarm", g.pivot, Y_AXIS)
    asm.pivot("WheelRear", g.RA, Y_AXIS, parent="Swingarm")
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
    asm.socket("Exhaust", s1 + sd * 0.03)
    asm.socket("Headlight", g.fork(0.80) + g.fwd * 0.13)
    wheels = [o for o in asm.meshes() if o["pivot"] in ("WheelFront", "WheelRear")]
    asm.collide("Body", [o for o in asm.meshes() if o not in wheels])
    asm.collide("WheelFront", [o for o in wheels if o["pivot"] == "WheelFront" and "Tire" in o.name], max_verts=32)
    asm.collide("WheelRear", [o for o in wheels if o["pivot"] == "WheelRear" and "Tire" in o.name], max_verts=32)
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
    for _ in range(8):
        mids, out, split = {}, [], False

        def mid(a, b):
            key = (min(a, b), max(a, b))
            if key not in mids:
                mids[key] = len(verts)
                verts.append((verts[a] + verts[b]) / 2)
            return mids[key]
        for a, b, c in faces:
            if max((verts[a] - verts[b]).length, (verts[b] - verts[c]).length, (verts[c] - verts[a]).length) > max_edge:
                ab, bc, ca = mid(a, b), mid(b, c), mid(c, a)
                out += [(a, ab, ca), (ab, b, bc), (ca, bc, c), (ab, bc, ca)]
                split = True
            else:
                out.append((a, b, c))
        faces = out
        if not split:
            break
    # T-Kreuzungen sind bei flachen Decals unkritisch (Projektion entlang derselben Richtung)
    res = MeshData()
    res.verts, res.faces = verts, faces
    res.uvs, res.mats = [None] * len(faces), [mat] * len(faces)
    return res.compact()


def project_onto(md, target, direction, offset=0.0015):
    """Mesh-Decal: alle Vertices entlang ``direction`` auf die Zielfläche (MeshData) projizieren."""
    md = subdivide_tris(md)
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


def airfoil_loft(sections, n_af=20, mat=None, cap=True):
    """Tragflächen-/Rotorblatt-Loft. Jede Sektion: dict(le, chord_dir, thick_dir, chord, t)."""
    base = airfoil_2d(n_af, 0.12)
    rings = []
    for s in sections:
        cd = V(s["chord_dir"]).normalized()
        td = V(s["thick_dir"]).normalized()
        k = s["t"] / 0.12
        rings.append([V(s["le"]) + cd * (p.x * s["chord"]) + td * (p.y * s["chord"] * k) for p in base])
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
        "interior": mat_plastic(P + "Interior", (0.05, 0.05, 0.055), 0.75, grain=0.15),
        "fabric": mat_plastic(P + "SeatFabric", (0.025, 0.027, 0.032), 0.9, grain=0.3),
        "floor": mat_rubber(P + "Floor", (0.03, 0.03, 0.03), 0.85),
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
                                   (0.80, 0.95, (0.8, 0.8, 0.8))], 0.45),
        "decal": mat_paint(P + "Decal", liv["text"], 0.3, coat=liv["coat"]),
        "dark_paint": mat_paint(P + "DarkPaint", (0.02, 0.02, 0.022), 0.35, coat=0.3),
        "panel": mat_plastic(P + "Panel", (0.018, 0.018, 0.02), 0.55),
        "screen": mat_emit(P + "Screen", (0.10, 0.22, 0.32), 1.5, base=(0.01, 0.01, 0.012)),
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
            asm.pivot(f"Door_{dk}", (p_lo + p_hi) / 2, (p_hi - p_lo).normalized())
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
    asm.pivot("MainRotor", g.hub, Z_AXIS)
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
        asm.pivot(key, g.hub + er * 0.62, er, parent="MainRotor")
        bl = blade_proto.copy().transform(Matrix.Translation(g.hub) @ rot)
        blades.append(asm.add(bl, key, key, smooth=40))
    asm.add(head, "RotorHead", "MainRotor", mods=[bevel_s])

    # ------------------------------------------------------------- Heckrotor
    asm.pivot("TailRotor", g.tr, Y_AXIS)
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
        asm.pivot("FLIR", V((2.12, 0.0, 0.44)), Z_AXIS)
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
        asm.socket("Searchlight", c0 + V((0.13, 0, -0.14)))

    # ------------------------------------------------------------- Sockets & Kollision
    asm.socket("Pilot", (1.30, -0.42, 1.05))
    asm.socket("CoPilot", (1.30, 0.42, 1.05))
    for i, y in enumerate((-0.52, 0.0, 0.52)):
        asm.socket(f"Passenger_{i + 1}", (-0.40, y, 1.05))
    asm.socket("RotorHub", g.hub)
    asm.socket("Exhaust", ex1)
    asm.socket("LandingLight", ll)
    asm.socket("Winch", (0.3, -0.95, 1.95))
    body = [o for o in asm.meshes() if o["pivot"] == "root" and o.name.split("_")[-1] not in ("Skids",)]
    skids = [o for o in asm.meshes() if o.name.endswith("Skids")]
    asm.collide("Cabin", body, lambda p: p.x > -2.3)
    asm.collide("TailBoom", body, lambda p: p.x < -2.0)
    asm.collide("Skid_L", skids, lambda p: p.y > 0.0, 24)
    asm.collide("Skid_R", skids, lambda p: p.y < 0.0, 24)
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


def convex_hull_obj(name, points, max_verts=48):
    cell = 0.02
    while True:
        grid = {}
        for p in points:
            grid.setdefault((round(p.x / cell), round(p.y / cell), round(p.z / cell)), p)
        bm = bmesh.new()
        for p in grid.values():
            bm.verts.new(p)
        res = bmesh.ops.convex_hull(bm, input=bm.verts[:], use_existing_faces=False)
        junk = list({g for g in res["geom_interior"] + res["geom_unused"] if isinstance(g, bmesh.types.BMVert)})
        bmesh.ops.delete(bm, geom=junk, context="VERTS")
        if len(bm.verts) <= max_verts or cell > 2.0:
            break
        bm.free()
        cell *= 1.35
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


def smart_uv(objs):
    if not objs:
        return
    select_only(objs)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.uv.smart_project(**op_kwargs(bpy.ops.uv.smart_project, angle_limit=math.radians(60), island_margin=0.0,
                                         scale_to_bounds=False))
    bpy.ops.object.mode_set(mode="OBJECT")


def pack_atlas(objs, margin=0.003):
    """Alle Teile eines Fahrzeugs in einen gemeinsamen, überlappungsfreien UV-Atlas packen."""
    select_only(objs)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.uv.select_all(action="SELECT")
    bpy.ops.uv.average_islands_scale()
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


def reorient(asm, extra, angle):
    """Fahrzeug für die Ziel-Engine drehen und Rotation in die Geometrie backen."""
    if abs(angle) < 1e-9:
        return
    R = Matrix.Rotation(angle, 4, "Z")
    objs = list(asm.coll.all_objects) + [o for o in extra if o.name not in asm.coll.all_objects]
    world = {o: o.matrix_world.copy() for o in objs}
    for o in objs:
        if o.type == "MESH":
            o.data.transform(R)
    depth = lambda o: 0 if o.parent is None else 1 + depth(o.parent)
    for o in sorted(objs, key=depth):
        o.matrix_world = Matrix.Translation(R @ world[o].translation)
    asm.axes = {k: (R.to_3x3() @ v) for k, v in asm.axes.items()}


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
    select_only(objs)
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
        if socket:
            with emission_override(mats, socket):
                bpy.ops.object.bake(**op_kwargs(bpy.ops.object.bake, **kw))
        else:
            bpy.ops.object.bake(**op_kwargs(bpy.ops.object.bake, **kw))
        imgs[key] = img
        log(f"  bake {key:6s} {res}px  {time.time() - t:5.1f}s")
    for m in mats:
        m.node_tree.nodes.remove(m.node_tree.nodes["_bake_target"])

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


def finalize(asm, cfg, out_dir):
    """Rohmodell → engine-fertiges Asset (siehe Modul-Docstring)."""
    t0 = time.time()
    tgt = TARGETS[cfg.target]
    short = asm.name[3:] if asm.name.startswith("SM_") else asm.name
    os.makedirs(out_dir, exist_ok=True)
    bpy.context.view_layer.update()
    meshes = asm.meshes()
    for o in meshes:
        # Solidify (identische UVs innen/außen) und Bevel (Facetten über UV-Nähte) → neu abwickeln
        if any(m.type in ("SOLIDIFY", "BEVEL") for m in o.modifiers):
            o["uv_ready"] = False
        apply_modifiers(o)
        tag_detail(o)
    hulls = build_collision(asm, asm.name) if cfg.collision else []     # UCX_<Asset>_## (Unreal-Konvention)
    log(f"{short}: {len(meshes)} Teile, {len(hulls)} Kollisionshüllen")
    smart_uv([o for o in meshes if not o.get("uv_ready")])
    parts = join_by_pivot(asm, meshes)
    opaque = [o for o in parts if not is_glass(o)]
    pack_atlas(opaque, 0.004 if cfg.bake >= 2048 else 0.006)
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
    files = []
    base = os.path.join(out_dir, asm.name)
    for L, objs in lods.items():
        suffix = "" if L == 0 else f"_LOD{L}"
        if "fbx" in cfg.formats:
            export_fbx(base + suffix + ".fbx", hierarchy + objs + (hulls if L == 0 else []), tgt)
            files.append(base + suffix + ".fbx")
        if "glb" in cfg.formats:
            export_glb(base + suffix + ".glb", hierarchy + objs)
            files.append(base + suffix + ".glb")
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
    pts = [o.matrix_world @ V(c) for o in lods[0] for c in o.bound_box]
    size = [max(p[i] for p in pts) - min(p[i] for p in pts) for i in range(3)]
    report = dict(
        asset=asm.name, target=cfg.target, forward_axis="+X" if tgt["rot"] == 0 else "-Y (Blender)",
        units="Meter", dimensions_m=[round(s, 3) for s in size],
        lods={f"LOD{k}": dict(triangles=tri_count(v), meshes=[o.name for o in v]) for k, v in lods.items()},
        pivots={k: dict(location=[round(c, 4) for c in p.matrix_world.translation], axis=[round(c, 4) for c in asm.axes[k]],
                        parent=asm.parents[k]) for k, p in asm.pivots.items()},
        sockets={k: [round(c, 4) for c in s.matrix_world.translation] for k, s in asm.sockets.items()},
        collision=[dict(name=h.name, group=h["collision_group"], verts=len(h.data.vertices)) for h in hulls],
        textures=textures, bones=bones,
        materials=sorted({m.name for o in lods[0] for m in o.data.materials if m}),
        files=[os.path.relpath(f, out_dir) for f in files], seconds=round(time.time() - t0, 1),
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
        description="Prozedurale, game-ready 3D-Modelle: Enduro und Hubschrauber (Blender/bpy).")
    p.add_argument("--vehicle", choices=["enduro", "heli", "all"], default="all")
    p.add_argument("--out", default="export", help="Ausgabeordner (Standard: ./export)")
    p.add_argument("--target", choices=sorted(TARGETS), default="unreal",
                   help="Ziel-Engine: steuert Ausrichtung, FBX-Skalierung und Textur-Packing")
    p.add_argument("--formats", default="fbx,glb,blend", help="Kommagetrennt: fbx, glb, blend")
    p.add_argument("--bake", type=int, default=2048, help="Texturauflösung für den PBR-Bake (0 = kein Bake)")
    p.add_argument("--bake-samples", type=int, default=32, help="Samples für den AO-Bake")
    p.add_argument("--lods", type=int, default=3, help="Anzahl zusätzlicher LOD-Stufen (0–4)")
    p.add_argument("--rig", action="store_true", help="Zusätzlich Skeletal Mesh (SK_*) mit Knochen exportieren")
    p.add_argument("--no-collision", dest="collision", action="store_false", help="Keine UCX-Kollision erzeugen")
    p.add_argument("--preview", action="store_true", help="Studio-Vorschaubilder rendern (Cycles)")
    p.add_argument("--preview-samples", type=int, default=96)
    p.add_argument("--gpu", action="store_true", help="Cycles auf der GPU (Bake/Preview)")
    p.add_argument("--seed", type=int, default=7, help="Zufallsstartwert (Stollenmuster, Variation)")
    p.add_argument("--dirt", type=float, default=0.35, help="Verschmutzungsgrad 0–1")
    p.add_argument("--enduro-color", choices=sorted(ENDURO_PALETTES), default="orange")
    p.add_argument("--heli-livery", dest="heli_color", choices=sorted(HELI_LIVERIES), default="civil")
    p.add_argument("--blades", type=int, default=4, choices=range(2, 8), metavar="2-7",
                   help="Anzahl Hauptrotorblätter")
    p.add_argument("--enduro-name", default="SM_Enduro")
    p.add_argument("--heli-name", default="SM_Heli")
    p.add_argument("--no-export", action="store_true", help="Nur Modelle bauen (z.B. in der Blender-GUI)")
    cfg = p.parse_args(argv)
    cfg.formats = {f.strip().lower() for f in cfg.formats.split(",") if f.strip()}
    cfg.lods = max(0, min(4, cfg.lods))
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
    if gui:
        cfg.no_export = True
    out_dir = os.path.abspath(cfg.out)
    builders = {"enduro": build_enduro, "heli": build_heli}
    keys = ["enduro", "heli"] if cfg.vehicle == "all" else [cfg.vehicle]
    reports = []
    for i, key in enumerate(keys):
        t = time.time()
        if gui:
            old = bpy.data.collections.get(cfg.enduro_name if key == "enduro" else cfg.heli_name)
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
                               samples=cfg.preview_samples, view=view, dist_scale=0.8 if key == "heli" else 0.85)
        if gui:
            asm.root.location.y = 8.0 * i
            continue
        if not cfg.no_export:
            reports.append(finalize(asm, cfg, out_dir))
    if reports:
        with open(os.path.join(out_dir, "summary.json"), "w", encoding="utf-8") as fh:
            json.dump(reports, fh, indent=2, ensure_ascii=False)
    return reports


if __name__ == "__main__":
    main()
