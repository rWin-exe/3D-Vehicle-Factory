#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
smoke_blender.py – Rundlauf-Test: die Exporte in frische Blender-Szenen zurückimportieren und gegen
den Report (``SM_*_report.json``) prüfen.

  blender --background --factory-startup --python-exit-code 1 --python tools/smoke_blender.py -- export
  python tools/smoke_blender.py export                (mit ``pip install bpy``)

Je Asset:
  * GLB und FBX aller LODs: Dreiecke wie im Report, LOD0-Maße, UV-Layer, eingebettete Texturen
  * Pivots: vorhanden, Position/Achse wie im Report, jedes Teil hängt an seinem Pivot; geometrisch:
    "spin"-Teile (Räder, Rotoren, Propeller) sitzen zentriert auf ihrer Achse und die Achse ist ihre
    Symmetrieachse (eiert nicht), "hinge"-Achsen berühren ihr Teil (Scharnier am Teil, nicht daneben),
    "slide"-Achsen liegen parallel zur Längsrichtung des Teils, "aim"-Achsen zeigen auf ihren Ziel-Pivot
  * Sockets: vorhanden, Position und Vorwärtsrichtung (+X) wie im Report
  * Kollision: Anzahl, ≤ 128 Vertices, konvex, umschließt LOD0 – außer dem, was der Report unter
    ``collision_exempt`` bewusst ausnimmt (drehende Rotor-/Propellerblätter, dünne Antennen)
  * Skelett (``--rig``): jeder Knochen bewegt genau die Vertices seiner Teile
Exit-Code 1 bei Fehlern. Bewertet weder Optik noch Fahrphysik; ersetzt keinen Test in der Ziel-Engine.

Idee (Blender-Rundlauf, Pivot-Rotation) nach ``tools/smoke_blender.py`` aus zickzackpro/3d.gen2,
für diese Pipeline neu geschrieben und um die geometrischen Achsen-Prüfungen ergänzt.
"""

import json
import math
import os
import sys

import bpy
import numpy as np
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree

HULL_MAX_VERTS = 128
TOL_POS = 0.002            # m: Positionen, Maße
TOL_INSIDE = 0.005         # m: LOD0-Vertices dürfen so weit aus den Hüllen ragen
TOL_TOUCH = 0.02           # m: Abstand Scharnierachse ↔ Oberfläche des Teils
HULL_CONVEX_TOL = 1e-3     # m: echte Dellen fallen auf, Rundungsrauschen der float32-Daten nicht


class Check:
    def __init__(self, asset):
        self.asset, self.errors, self.info = asset, [], {}

    def need(self, cond, msg):
        if not cond:
            self.errors.append(f"{self.asset}: {msg}")
        return cond


def fresh_import(paths):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    for path in paths:
        op = bpy.ops.import_scene.gltf if path.endswith(".glb") else bpy.ops.import_scene.fbx
        if "FINISHED" not in op(filepath=path):
            raise RuntimeError(f"Import fehlgeschlagen: {path}")
    bpy.context.view_layer.update()
    return list(bpy.context.scene.objects)


def tris(objs):
    n = 0
    for o in objs:
        o.data.calc_loop_triangles()
        n += len(o.data.loop_triangles)
    return n


def world_verts(objs):
    out = []
    for o in objs:
        mw = o.matrix_world
        out.extend(mw @ v.co for v in o.data.vertices)
    return out


def dims(pts):
    return [max(p[i] for p in pts) - min(p[i] for p in pts) for i in range(3)]


def descendants(obj):
    out, stack = [], list(obj.children)
    while stack:
        o = stack.pop()
        out.append(o)
        stack.extend(o.children)
    return out


def is_hull(o):
    return o.name.startswith("UCX_")


def check_lod_file(c, path, lod, rep, lod_key):
    objs = fresh_import([path])
    meshes = [o for o in objs if o.type == "MESH" and not is_hull(o)]
    rel = os.path.basename(path)
    n = tris(meshes)
    c.need(n == lod["triangles"], f"{rel}: {n} Dreiecke nach Import, Report {lod_key}: {lod['triangles']}")
    c.need(all(o.data.uv_layers for o in meshes), f"{rel}: Mesh ohne UV-Layer")
    if lod_key == "LOD0":
        d = dims(world_verts(meshes))
        c.need(all(abs(a - b) <= TOL_POS for a, b in zip(d, rep["dimensions_m"])),
               f"{rel}: Maße {[round(x, 3) for x in d]} ≠ Report {rep['dimensions_m']}")
    return objs, meshes


# ----------------------------------------------------------------------------- Pivots & Sockets

def area_cloud(objs):
    """Dreiecks-Schwerpunkte und -Flächen (Welt) als numpy-Arrays."""
    cs, ws = [], []
    for o in objs:
        me = o.data
        me.calc_loop_triangles()
        mw = o.matrix_world
        v = np.array([tuple(mw @ x.co) for x in me.vertices])
        t = np.array([tuple(lt.vertices) for lt in me.loop_triangles])
        if not len(t):
            continue
        a, b, cc = v[t[:, 0]], v[t[:, 1]], v[t[:, 2]]
        cs.append((a + b + cc) / 3)
        ws.append(np.linalg.norm(np.cross(b - a, cc - a), axis=1) / 2)
    return np.concatenate(cs), np.concatenate(ws)


def check_spin(c, key, parts, loc, axis):
    """Rad/Rotor/Propeller: Flächenschwerpunkt auf der Achse, Achse = Hauptträgheitsachse (falls eindeutig)."""
    pts, w = area_cloud(parts)
    p, a = np.array(tuple(loc)), np.array(tuple(axis))
    ctr = (pts * w[:, None]).sum(0) / w.sum()
    rel = pts - p
    radius = float(np.max(np.linalg.norm(rel - np.outer(rel @ a, a), axis=1)))
    off = ctr - p
    dist = float(np.linalg.norm(off - (off @ a) * a))
    tol = max(0.003, 0.005 * radius)
    c.need(dist <= tol, f"Pivot {key}: Teil sitzt {dist * 1000:.1f} mm neben der Drehachse (max. {tol * 1000:.1f} mm)")
    r = pts - ctr
    inertia = (w[:, None, None] * ((r * r).sum(1)[:, None, None] * np.eye(3) - r[:, :, None] * r[:, None, :])).sum(0)
    val, vec = np.linalg.eigh(inertia)
    info = {"offset_mm": round(dist * 1000, 2), "radius_m": round(radius, 3)}
    if val[2] > 1.15 * val[1]:                       # eindeutige Symmetrieachse (bei 2 Blättern nicht gegeben)
        ang = math.degrees(math.acos(min(1.0, abs(float(vec[:, 2] @ a)))))
        info["axis_error_deg"] = round(ang, 2)
        c.need(ang <= 3.0, f"Pivot {key}: Drehachse weicht {ang:.1f}° von der Symmetrieachse des Teils ab (eiert)")
    return info


def check_hinge(c, key, parts, loc, axis):
    """Scharnierachse muss ihr Teil berühren: kleinster Abstand der Achse (im Bereich des Teils) zu seiner
    Oberfläche – nicht zum nächsten Vertex, denn die Achse liegt oft mitten in einer Schnittfläche.
    Erkennt grob falsch platzierte Scharniere; ein Versatz in der Ebene des Teils fällt nicht immer auf."""
    verts, tris = [], []
    for o in parts:
        o.data.calc_loop_triangles()
        off = len(verts)
        verts += [o.matrix_world @ v.co for v in o.data.vertices]
        tris += [tuple(off + i for i in t.vertices) for t in o.data.loop_triangles]
    bvh = BVHTree.FromPolygons(verts, tris)
    t = [(v - loc).dot(axis) for v in verts]
    d = min(bvh.find_nearest(loc + axis * (min(t) + (max(t) - min(t)) * k / 200))[3] for k in range(201))
    c.need(d <= TOL_TOUCH, f"Pivot {key}: Achse liegt {d * 100:.1f} cm neben dem Teil (max. {TOL_TOUCH * 100:.0f} cm)")
    return {"touch_mm": round(d * 1000, 1)}


def check_slide(c, key, parts, axis):
    """Schiebeachse (z.B. Gabel) muss parallel zur Längsrichtung des Teils liegen (Lage egal)."""
    pts, w = area_cloud(parts)
    ctr = (pts * w[:, None]).sum(0) / w.sum()
    r = pts - ctr
    cov = (w[:, None, None] * r[:, :, None] * r[:, None, :]).sum(0)
    val, vec = np.linalg.eigh(cov)
    info = {}
    if val[2] > 2.0 * val[1]:                        # eindeutig länglich
        ang = math.degrees(math.acos(min(1.0, abs(float(vec[:, 2] @ np.array(tuple(axis)))))))
        info["axis_error_deg"] = round(ang, 2)
        c.need(ang <= 3.0, f"Pivot {key}: Schiebeachse weicht {ang:.1f}° von der Längsrichtung des Teils ab")
    return info


def check_aim(c, key, parts, loc, axis, target):
    """Aim-Pivot (Federbein-Hälfte): lokales +Y zeigt in Ruhelage auf den Ziel-Pivot – die Engine hält es
    per Look-At-/Aim-Constraint darauf; ein länglicher Teil liegt zudem längs dieser Achse."""
    if not c.need(target is not None, f"Pivot {key}: Aim-Ziel fehlt nach dem Import"):
        return {}
    d = target.matrix_world.translation - loc
    if not c.need(d.length > 1e-4, f"Pivot {key}: Aim-Ziel liegt im Pivot"):
        return {}
    ang = math.degrees(axis.angle(d.normalized()))
    c.need(ang <= 1.0, f"Pivot {key}: Achse zeigt {ang:.1f}° am Aim-Ziel vorbei")
    return {"aim_error_deg": round(ang, 2), **check_slide(c, key, parts, axis)}


def check_pivots(c, rep, objs, rel):
    asset = rep["asset"]
    by_name = {o.name: o for o in objs}
    meshes = [o for o in objs if o.type == "MESH" and not is_hull(o)]
    results = {}
    for key, info in rep["pivots"].items():
        if key == "root":
            continue
        p = by_name.get(f"{asset}_{key}_Pivot")
        if not c.need(p is not None and p.type == "EMPTY", f"{rel}: Pivot {key} fehlt nach dem Import"):
            continue
        loc = p.matrix_world.translation.copy()
        c.need((loc - Vector(info["location"])).length <= TOL_POS, f"{rel}: Pivot {key}: Position weicht vom Report ab")
        axis = Vector(info["axis"]).normalized()
        if info.get("oriented"):
            local_y = (p.matrix_world.to_3x3() @ Vector((0, 1, 0))).normalized()
            c.need((local_y - axis).length <= 0.01, f"{rel}: Pivot {key}: lokale Y-Achse ≠ Drehachse aus dem Report")
        own = [o for o in p.children if o.type == "MESH"]
        expected = {f"{asset}_{key}", f"{asset}_{key}_Glass"}
        c.need(own and {o.name for o in own} <= expected,
               f"{rel}: Pivot {key}: direkte Teile {sorted(o.name for o in own)} statt {sorted(expected)}")
        if not own:
            continue
        # Hierarchie: Drehung um die Achse bewegt genau den Teilbaum, starr, und lässt sich zurücksetzen
        subtree = {o for o in descendants(p) if o.type == "MESH"}
        before = {o: o.matrix_world.copy() for o in meshes}
        basis = p.matrix_basis.copy()
        if info.get("motion") == "slide":
            move = Matrix.Translation(axis * 0.02)
        else:
            move = Matrix.Translation(loc) @ Matrix.Rotation(math.radians(9.0), 4, axis) @ Matrix.Translation(-loc)
        p.matrix_world = move @ p.matrix_world
        bpy.context.view_layer.update()
        for o in meshes:
            want = move @ before[o] if o in subtree else before[o]
            dev = max(abs(o.matrix_world[i][j] - want[i][j]) for i in range(4) for j in range(4))
            if dev > 1e-4:
                c.need(False, f"{rel}: Pivot {key}: {o.name} " + ("folgt nicht starr" if o in subtree else "bewegt sich mit, gehört aber nicht dazu"))
                break
        p.matrix_basis = basis
        bpy.context.view_layer.update()
        c.need(all((o.matrix_world.translation - before[o].translation).length < 1e-5 for o in subtree),
               f"{rel}: Pivot {key}: Teile kehren nach dem Zurücksetzen nicht an ihre Position zurück")
        motion = info.get("motion", "hinge")
        if motion == "spin":
            results[key] = check_spin(c, f"{rel}: {key}", sorted(subtree, key=lambda o: o.name), loc, axis)
        elif motion == "slide":
            results[key] = check_slide(c, f"{rel}: {key}", own, axis)
        elif motion == "aim":
            results[key] = check_aim(c, f"{rel}: {key}", own, loc, axis, by_name.get(f"{asset}_{info.get('target')}_Pivot"))
        else:
            results[key] = check_hinge(c, f"{rel}: {key}", own, loc, axis)
        results[key]["motion"] = motion
    for key, info in rep.get("sockets", {}).items():
        s = by_name.get(f"SOCKET_{key}")
        if not c.need(s is not None, f"{rel}: Socket {key} fehlt nach dem Import"):
            continue
        c.need((s.matrix_world.translation - Vector(info["location"])).length <= TOL_POS,
               f"{rel}: Socket {key}: Position weicht vom Report ab")
        fwd = (s.matrix_world.to_3x3() @ Vector((1, 0, 0))).normalized()
        c.need((fwd - Vector(info["forward"]).normalized()).length <= 0.01, f"{rel}: Socket {key}: +X ≠ Vorwärtsrichtung aus dem Report")
    return results


def check_textures(c, rep, meshes, rel):
    if not rep.get("textures"):
        return 0
    imgs = {n.image for o in meshes for m in o.data.materials if m and m.node_tree
            for n in m.node_tree.nodes if n.type == "TEX_IMAGE" and n.image}
    c.need(imgs, f"{rel}: keine Texturen im Material nach dem Import")
    for im in imgs:
        c.need(im.size[0] > 0 and im.size[1] > 0 and im.has_data, f"{rel}: Textur {im.name} ohne Pixeldaten")
    return len(imgs)


# ----------------------------------------------------------------------------- Kollision

def hull_planes(o):
    me = o.data
    me.calc_loop_triangles()
    mw = o.matrix_world
    v = np.array([tuple(mw @ x.co) for x in me.vertices])
    t = np.array([tuple(lt.vertices) for lt in me.loop_triangles])
    a, b, c = v[t[:, 0]], v[t[:, 1]], v[t[:, 2]]
    n = np.cross(b - a, c - a)
    ln = np.linalg.norm(n, axis=1)
    longest = np.max([np.linalg.norm(b - a, axis=1), np.linalg.norm(c - b, axis=1), np.linalg.norm(a - c, axis=1)], axis=0)
    keep = (ln > 1e-12) & (ln / np.maximum(longest, 1e-12) >= 5e-4)     # Splitter: Normale unbestimmt
    n, base = n[keep] / ln[keep, None], v[t[keep, 0]]
    d = (n * base).sum(1)
    ctr = v.mean(0)
    flip = n @ ctr > d                                # nach außen orientieren
    n[flip], d[flip] = -n[flip], -d[flip]
    return v, n, d


def check_collision(c, rep, base_dir):
    files = [f for f in rep.get("collision_files", []) if f.endswith(".glb")] or rep.get("collision_files", [])
    if not rep.get("collision"):
        return {}
    if not c.need(files, "Report nennt Kollisionshüllen, aber keine Kollisionsdatei"):
        return {}
    lod0 = [f for f in rep["lods"]["LOD0"]["files"] if f.endswith(os.path.splitext(files[0])[1]) and f != files[0]]
    objs = fresh_import([os.path.join(base_dir, f) for f in [files[0]] + lod0[:1]])   # Unreal-FBX: Hüllen im LOD0
    hulls = [o for o in objs if o.type == "MESH" and is_hull(o)]
    names = {h["name"] for h in rep["collision"]}
    c.need({o.name for o in hulls} == names, f"{files[0]}: Hüllen passen nicht zum Report")
    planes, worst_convex = [], 0.0
    for h in hulls:
        v, n, d = hull_planes(h)
        uniq = len({tuple(np.round(p, 5)) for p in v})
        c.need(uniq <= HULL_MAX_VERTS, f"{h.name}: {uniq} Vertices (max. {HULL_MAX_VERTS})")
        out = float(np.max(v @ n.T - d)) if len(n) else 0.0
        worst_convex = max(worst_convex, out)
        c.need(out <= HULL_CONVEX_TOL, f"{h.name}: nicht konvex ({out * 1000:.2f} mm)")
        planes.append((n, d))
    # Umschließen: jeder LOD0-Vertex liegt in mindestens einer Hülle – außer den Teilen, die laut Report
    # bewusst keine Hülle haben (drehende Rotor-/Propellerblätter, dünne Antennen)
    asset, exempt = rep["asset"], rep.get("collision_exempt", {})
    skip = set()
    for key in exempt.get("pivots", {}):
        piv = bpy.data.objects.get(f"{asset}_{key}_Pivot")
        if piv is not None:
            skip |= {o for o in descendants(piv)}
    boxes = [(np.array(r["min"]) - TOL_INSIDE, np.array(r["max"]) + TOL_INSIDE) for r in exempt.get("regions", [])]
    parts = [o for o in objs if o.type == "MESH" and not is_hull(o) and o not in skip]
    outside = {}
    for o in parts:
        pts = np.array([tuple(p) for p in world_verts([o])])
        best = np.full(len(pts), np.inf)
        for n, d in planes:
            best = np.minimum(best, np.max(pts @ n.T - d, axis=1))
        for lo, hi in boxes:
            best[np.all((pts >= lo) & (pts <= hi), axis=1)] = -1.0
        bad = best > TOL_INSIDE
        if bad.any():
            outside[o.name] = {"verts": int(bad.sum()), "of": len(pts), "max_mm": round(float(best.max()) * 1000, 1)}
    for name, s in outside.items():
        c.need(False, f"{name}: {s['verts']} von {s['of']} Vertices außerhalb aller Hüllen (bis {s['max_mm']} mm)")
    return {"hulls": len(hulls), "max_convexity_error_mm": round(worst_convex * 1000, 3), "outside": outside}


# ----------------------------------------------------------------------------- Skelett

def check_rig(c, rep, path):
    objs = fresh_import([path])
    rel = os.path.basename(path)
    arm = next((o for o in objs if o.type == "ARMATURE"), None)
    sk = next((o for o in objs if o.type == "MESH" and any(m.type == "ARMATURE" for m in o.modifiers)), None)
    if not c.need(arm is not None and sk is not None, f"{rel}: Armature oder Skeletal Mesh fehlt"):
        return {}
    bones = {b.name for b in arm.data.bones}
    c.need(bones == set(rep["bones"]), f"{rel}: Knochen {sorted(bones ^ set(rep['bones']))} passen nicht zum Report")
    groups = {g.index: g.name for g in sk.vertex_groups}
    owner = {}
    for v in sk.data.vertices:
        ws = [(g.weight, groups[g.group]) for g in v.groups if g.weight > 0]
        owner[v.index] = max(ws)[1] if ws else None

    def rest():
        dg = bpy.context.evaluated_depsgraph_get()
        ev = sk.evaluated_get(dg)
        me = ev.to_mesh()
        out = np.array([tuple(sk.matrix_world @ v.co) for v in me.vertices])
        ev.to_mesh_clear()
        return out
    base = rest()
    moved_ok = {}
    for b in arm.data.bones:
        if b.parent is None:
            continue
        sub = {b.name} | {x.name for x in b.children_recursive}
        want = np.array([owner[i] in sub for i in range(len(base))])
        pb = arm.pose.bones[b.name]
        pb.rotation_mode = "XYZ"
        pb.rotation_euler = (0.0, math.radians(12.0), 0.0)
        bpy.context.view_layer.update()
        moved = np.linalg.norm(rest() - base, axis=1) > 1e-4
        pb.rotation_euler = (0.0, 0.0, 0.0)
        bpy.context.view_layer.update()
        stray = int((moved & ~want).sum())
        share = float(moved[want].mean()) if want.any() else 0.0
        c.need(stray == 0, f"{rel}: Knochen {b.name} bewegt {stray} fremde Vertices")
        c.need(share >= 0.9, f"{rel}: Knochen {b.name} bewegt nur {share:.0%} seiner Vertices")
        moved_ok[b.name] = round(share, 3)
    return {"bones": len(bones), "moved_share": moved_ok}


# ----------------------------------------------------------------------------- Ablauf

def check_asset(report_path):
    with open(report_path, encoding="utf-8") as fh:
        rep = json.load(fh)
    c = Check(rep.get("asset", os.path.basename(report_path)))
    if rep.get("schema_version", 1) < 2:
        c.need(False, "Report-Schema zu alt – mit aktuellem vehicle_generator.py neu exportieren")
        return c
    base = os.path.dirname(report_path)
    lods = sorted(rep["lods"].items(), key=lambda kv: int(kv[0][3:]))
    for key, lod in lods:
        for rel in lod.get("files", []):
            objs, meshes = check_lod_file(c, os.path.join(base, rel), lod, rep, key)
            if key != "LOD0":
                continue
            c.info[f"pivots_{rel[-3:]}"] = check_pivots(c, rep, objs, rel)
            if rel.endswith(".glb"):
                c.info["textures"] = check_textures(c, rep, meshes, rel)
            if rel.endswith(".fbx") and rep.get("target") == "unreal" and rep.get("collision"):
                n = len([o for o in objs if o.type == "MESH" and is_hull(o)])
                c.need(n == len(rep["collision"]), f"{rel}: {n} UCX-Hüllen eingebettet, Report: {len(rep['collision'])}")
    c.info["collision"] = check_collision(c, rep, base)
    for f in rep.get("files", []):
        name = os.path.basename(f["path"])
        if name.startswith("SK_") and name.endswith((".fbx", ".glb")):
            c.info[f"rig_{name[-3:]}"] = check_rig(c, rep, os.path.join(base, f["path"]))
    return c


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else sys.argv[1:]
    if len(argv) != 1 or argv[0].startswith("-"):
        print(__doc__)
        return 2
    target = os.path.abspath(argv[0])
    reports = sorted(os.path.join(root, n) for root, _, names in os.walk(target) for n in names if n.endswith("_report.json"))
    if not reports:
        print(f"FEHLER  keine *_report.json in {target} – zuerst vehicle_generator.py ausführen")
        return 1
    errors, summary = [], []
    for rp in reports:
        try:
            c = check_asset(rp)
        except Exception as err:                         # Import-/Lesefehler als Testfehler melden
            c = Check(os.path.basename(rp))
            c.errors.append(f"{c.asset}: {type(err).__name__}: {err}")
        errors += c.errors
        summary.append({"asset": c.asset, "ok": not c.errors, "errors": c.errors, "details": c.info})
        print(("OK      " if not c.errors else "FEHLER  ") + c.asset)
        for e in c.errors:
            print("        " + e)
    out = os.path.join(target if os.path.isdir(target) else os.path.dirname(target), "smoke_result.json")
    with open(out, "w", encoding="utf-8") as fh:
        json.dump({"ok": not errors, "blender_version": bpy.app.version_string, "assets": summary}, fh, indent=2,
                  ensure_ascii=False)
    print(f"{len(reports)} Assets: " + ("alles in Ordnung" if not errors else f"{len(errors)} Fehler") + f" (Details: {out})")
    return 0 if not errors else 1


if __name__ == "__main__":
    code = main()
    if code:
        sys.exit(code)
