#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
validate_exports.py – prüft die Exporte von ``vehicle_generator.py`` ohne Blender
(nur Python-Standardbibliothek, läuft daher auch in CI in Sekunden).

Je GLB-Datei: Container (Header, Chunks, eingebetteter Puffer), Accessoren (Grenzen, endliche Werte,
POSITION-min/max), Dreiecks-Indizes, Normalen (Einheitslänge), UVs (außer Kollision), Tangenten,
Skin-Gewichte, Bilder (eingebettet, Signatur, Größe), Knoten-Hierarchie (Zyklen, Mehrfach-Eltern,
normierte Quaternionen) und Weltmaße.

Liegt ein ``*_report.json`` im Ordner, zusätzlich gegen den Report: Dateigrößen + SHA-256, Dreiecke je
LOD (streng abnehmend), Maße, Pivots/Sockets als Knoten in jedem LOD, Kollisionshüllen (Anzahl,
≤ 128 Vertices, konvex) und Knochennamen des Skeletal Mesh.

Aufruf:
  python tools/validate_exports.py export           # Ordner (rekursiv) oder einzelne .glb
  python tools/validate_exports.py export --json    # vollständiger Bericht als JSON
Exit-Code 0 = in Ordnung, 1 = Fehler. Optik, Fahrphysik und der Import in die Ziel-Engine werden
nicht bewertet (Blender-Rundlauf: ``tools/smoke_blender.py``).

Idee nach ``tools/validate_exports.py`` aus zickzackpro/3d.gen2, für diese Pipeline neu geschrieben.
"""

import argparse
import hashlib
import json
import math
import os
import struct
import sys

HULL_MAX_VERTS = 128
REPORT_SCHEMA = 2
CHUNK_JSON, CHUNK_BIN = 0x4E4F534A, 0x004E4942
COMPONENTS = {5120: ("b", 1), 5121: ("B", 1), 5122: ("h", 2), 5123: ("H", 2), 5125: ("I", 4), 5126: ("f", 4)}
WIDTH = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT2": 4, "MAT3": 9, "MAT4": 16}
IDENTITY = ((1.0, 0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0), (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0))


class Invalid(ValueError):
    """Die Datei verletzt das glTF-Format oder die Regeln dieser Pipeline."""


def need(cond, msg):
    if not cond:
        raise Invalid(msg)


def ref(items, i, what):
    need(isinstance(i, int) and not isinstance(i, bool) and 0 <= i < len(items), f"ungültiger Index für {what}: {i!r}")
    return items[i]


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


# ----------------------------------------------------------------------------- Matrizen (4×4, Zeilen)

def matmul(a, b):
    return tuple(tuple(sum(a[r][k] * b[k][c] for k in range(4)) for c in range(4)) for r in range(4))


def node_matrix(node):
    if "matrix" in node:
        need(not any(k in node for k in ("translation", "rotation", "scale")), "Knoten mischt matrix und TRS")
        m = node["matrix"]
        need(isinstance(m, list) and len(m) == 16 and all(isinstance(x, (int, float)) and math.isfinite(x) for x in m),
             "ungültige Knoten-Matrix")
        return tuple(tuple(float(m[c * 4 + r]) for c in range(4)) for r in range(4))     # glTF: spaltenweise

    def vec(key, default):
        v = node.get(key, default)
        need(isinstance(v, list) and len(v) == len(default)
             and all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in v),
             f"ungültiges Knotenfeld {key}")
        return [float(x) for x in v]
    t = vec("translation", [0, 0, 0])
    x, y, z, w = vec("rotation", [0, 0, 0, 1])
    s = vec("scale", [1, 1, 1])
    need(abs(x * x + y * y + z * z + w * w - 1.0) < 1e-3, "Quaternion eines Knotens ist nicht normiert")
    need(all(abs(c) > 1e-12 for c in s), "Knoten mit Skalierung 0")
    rot = ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
           (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
           (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))
    return tuple(tuple(rot[r][c] * s[c] for c in range(3)) + (t[r],) for r in range(3)) + ((0.0, 0.0, 0.0, 1.0),)


# ----------------------------------------------------------------------------- GLB-Container

class Glb:
    def __init__(self, path):
        with open(path, "rb") as fh:
            raw = fh.read()
        need(len(raw) >= 20, "Datei zu kurz für einen GLB-Header")
        magic, version, length = struct.unpack_from("<4sII", raw)
        need(magic == b"glTF", "keine GLB-Datei (Magic fehlt)")
        need(version == 2, f"GLB-Version {version} statt 2")
        need(length == len(raw), f"Header-Länge {length} ≠ Dateigröße {len(raw)}")
        chunks, off = [], 12
        while off < len(raw):
            need(off + 8 <= len(raw), "abgeschnittener Chunk-Header")
            size, kind = struct.unpack_from("<II", raw, off)
            off += 8
            need(size % 4 == 0, "Chunk-Länge nicht auf 4 Byte ausgerichtet")
            need(off + size <= len(raw), "Chunk reicht über das Dateiende")
            chunks.append((kind, raw[off:off + size]))
            off += size
        need(chunks and chunks[0][0] == CHUNK_JSON, "erster Chunk ist kein JSON")
        need(sum(k == CHUNK_JSON for k, _ in chunks) == 1, "mehrere JSON-Chunks")
        bins = [d for k, d in chunks if k == CHUNK_BIN]
        need(len(bins) <= 1, "mehrere BIN-Chunks")
        try:
            self.doc = json.loads(chunks[0][1].decode("utf-8"))
        except (UnicodeDecodeError, ValueError) as err:
            raise Invalid(f"JSON-Chunk unlesbar: {err}") from None
        need(isinstance(self.doc, dict) and isinstance(self.doc.get("asset"), dict)
             and self.doc["asset"].get("version") == "2.0", "kein glTF-2.0-Dokument")
        self.bin = bins[0] if bins else b""
        self.size = len(raw)
        buffers = self.doc.get("buffers", [])
        need(len(buffers) == 1 and "uri" not in buffers[0], "erwartet genau einen eingebetteten Puffer")
        blen = buffers[0].get("byteLength")
        need(isinstance(blen, int) and 0 < blen <= len(self.bin) and len(self.bin) - blen <= 3,
             "byteLength des Puffers passt nicht zum BIN-Chunk")
        self.views = self.doc.get("bufferViews", [])
        for i, v in enumerate(self.views):
            o, n = v.get("byteOffset", 0), v.get("byteLength")
            need(v.get("buffer") == 0 and isinstance(o, int) and isinstance(n, int) and o >= 0 and n > 0
                 and o + n <= blen, f"bufferView {i} liegt außerhalb des Puffers")
            if "byteStride" in v:
                s = v["byteStride"]
                need(isinstance(s, int) and 4 <= s <= 252 and s % 4 == 0, f"bufferView {i}: ungültiger byteStride")
        self.accessors = self.doc.get("accessors", [])
        self._cache = {}

    def read(self, i):
        """Accessor als Liste von Tupeln (geprüft: Grenzen, Ausrichtung, endliche Werte)."""
        if i in self._cache:
            return self._cache[i]
        a = ref(self.accessors, i, "Accessor")
        need("sparse" not in a, f"Accessor {i}: sparse wird nicht unterstützt")
        view = ref(self.views, a.get("bufferView"), "bufferView")
        ct, typ = a.get("componentType"), a.get("type")
        need(ct in COMPONENTS and typ in WIDTH, f"Accessor {i}: unbekannter Typ {typ}/{ct}")
        fmt, size = COMPONENTS[ct]
        need(not typ.startswith("MAT") or size == 4, f"Accessor {i}: gepackte Matrix wird nicht unterstützt")
        item = struct.Struct("<" + fmt * WIDTH[typ])
        off, count = a.get("byteOffset", 0), a.get("count")
        stride = view.get("byteStride", item.size)
        need(isinstance(off, int) and off >= 0 and off % size == 0, f"Accessor {i}: ungültiger byteOffset")
        need(isinstance(count, int) and count > 0, f"Accessor {i}: leer")
        need(stride >= item.size, f"Accessor {i}: byteStride kleiner als ein Element")
        need(off + (count - 1) * stride + item.size <= view["byteLength"], f"Accessor {i} reicht über seine bufferView")
        start = view.get("byteOffset", 0) + off
        if stride == item.size:
            vals = list(item.iter_unpack(self.bin[start:start + count * item.size]))
        else:
            vals = [item.unpack_from(self.bin, start + k * stride) for k in range(count)]
        if fmt == "f":
            need(all(math.isfinite(x) for row in vals for x in row), f"Accessor {i}: NaN/Inf in den Daten")
        self._cache[i] = vals
        return vals

    def view_bytes(self, i):
        v = ref(self.views, i, "bufferView")
        o = v.get("byteOffset", 0)
        return self.bin[o:o + v["byteLength"]]


# ----------------------------------------------------------------------------- Einzeldatei

def image_info(data, mime, k):
    if mime == "image/png":
        need(data[:8] == b"\x89PNG\r\n\x1a\n" and data[12:16] == b"IHDR", f"Bild {k}: keine gültige PNG-Signatur")
        w, h = struct.unpack(">II", data[16:24])
        return {"mime": mime, "width": w, "height": h}
    if mime == "image/jpeg":
        need(data[:3] == b"\xff\xd8\xff", f"Bild {k}: keine gültige JPEG-Signatur")
        return {"mime": mime}
    if mime == "image/webp":
        need(data[:4] == b"RIFF" and data[8:12] == b"WEBP", f"Bild {k}: keine gültige WebP-Signatur")
        return {"mime": mime}
    raise Invalid(f"Bild {k}: MIME-Typ {mime!r} wird nicht unterstützt")


def check_glb(path, collision=None):
    """Eine GLB-Datei prüfen. ``collision``: Kollisionsdatei (UVs optional, Hüllen-Daten sammeln);
    None = anhand des Namens (``*_Collision.glb``) entscheiden."""
    if collision is None:
        collision = os.path.basename(path).lower().endswith("_collision.glb")
    g = Glb(path)
    doc = g.doc
    textures, images, samplers = doc.get("textures", []), doc.get("images", []), doc.get("samplers", [])
    materials = doc.get("materials", [])
    for t in textures:
        if "source" in t:
            ref(images, t["source"], "Textur-Bild")
        if "sampler" in t:
            ref(samplers, t["sampler"], "Sampler")

    def walk(v):
        if isinstance(v, dict):
            for k, c in v.items():
                if k.endswith("Texture") and isinstance(c, dict) and "index" in c:
                    ref(textures, c["index"], "Material-Textur")
                walk(c)
        elif isinstance(v, list):
            for c in v:
                walk(c)
    for m in materials:
        walk(m)
    imgs = []
    for k, im in enumerate(images):
        need("uri" not in im, f"Bild {k} ist nicht eingebettet")
        imgs.append(image_info(g.view_bytes(im.get("bufferView")), im.get("mimeType"), k))

    meshes = doc.get("meshes", [])
    need(meshes, "keine Meshes exportiert")
    mesh_data = []
    for mi, mesh in enumerate(meshes):
        prims = mesh.get("primitives", [])
        need(prims, f"Mesh {mi} ohne Primitive")
        positions, tris, degenerate, faces = [], 0, 0, []
        for p in prims:
            need(p.get("mode", 4) == 4, f"Mesh {mi}: nur Dreiecke erlaubt")
            at = p.get("attributes", {})
            missing = ({"POSITION", "NORMAL"} | (set() if collision else {"TEXCOORD_0"})) - at.keys()
            need(not missing, f"Mesh {mi}: Attribute fehlen: {', '.join(sorted(missing))}")
            pa = ref(g.accessors, at["POSITION"], "POSITION-Accessor")
            need(pa.get("type") == "VEC3" and pa.get("componentType") == 5126, f"Mesh {mi}: POSITION muss float-VEC3 sein")
            pos = g.read(at["POSITION"])
            need("min" in pa and "max" in pa, f"Mesh {mi}: POSITION ohne min/max (glTF-Pflicht)")
            for c in range(3):
                lo, hi = min(v[c] for v in pos), max(v[c] for v in pos)
                need(abs(lo - pa["min"][c]) <= 1e-4 and abs(hi - pa["max"][c]) <= 1e-4,
                     f"Mesh {mi}: POSITION-min/max passen nicht zu den Daten")
            for sem, ai in at.items():
                need(len(g.read(ai)) == len(pos), f"Mesh {mi}: {sem} hat eine andere Anzahl als POSITION")
            normals = g.read(at["NORMAL"])
            need(len(normals[0]) == 3, f"Mesh {mi}: NORMAL muss VEC3 sein")
            need(all(abs(x * x + y * y + z * z - 1.0) < 2e-3 for x, y, z in normals), f"Mesh {mi}: Normalen nicht normiert")
            if "TEXCOORD_0" in at:
                need(len(g.read(at["TEXCOORD_0"])[0]) == 2, f"Mesh {mi}: TEXCOORD_0 muss VEC2 sein")
            if "TANGENT" in at:
                tan = g.read(at["TANGENT"])
                need(len(tan[0]) == 4 and all(abs(abs(t[3]) - 1.0) < 1e-3 for t in tan), f"Mesh {mi}: Tangenten-w muss ±1 sein")
            if "WEIGHTS_0" in at:
                need("JOINTS_0" in at, f"Mesh {mi}: WEIGHTS_0 ohne JOINTS_0")
                wa = g.accessors[at["WEIGHTS_0"]]
                scale = {5126: 1.0, 5121: 255.0, 5123: 65535.0}.get(wa.get("componentType"))
                need(scale is not None, f"Mesh {mi}: ungültiger Typ für WEIGHTS_0")
                need(all(abs(sum(w) / scale - 1.0) < 1e-2 for w in g.read(at["WEIGHTS_0"])),
                     f"Mesh {mi}: Skin-Gewichte summieren nicht zu 1")
            if "material" in p:
                ref(materials, p["material"], "Material")
            if "indices" in p:
                ia = ref(g.accessors, p["indices"], "Index-Accessor")
                need(ia.get("type") == "SCALAR" and ia.get("componentType") in (5121, 5123, 5125),
                     f"Mesh {mi}: Indizes müssen vorzeichenlose Skalare sein")
                idx = [r[0] for r in g.read(p["indices"])]
                need(max(idx) < len(pos), f"Mesh {mi}: Dreiecks-Index außerhalb der Vertexliste")
            else:
                idx = list(range(len(pos)))
            need(len(idx) >= 3 and len(idx) % 3 == 0, f"Mesh {mi}: unvollständige Dreiecke")
            tris += len(idx) // 3
            degenerate += sum(1 for k in range(0, len(idx), 3)
                              if idx[k] == idx[k + 1] or idx[k + 1] == idx[k + 2] or idx[k] == idx[k + 2])
            if collision:
                faces.extend(tuple(pos[idx[k + j]] for j in range(3)) for k in range(0, len(idx), 3))
            positions.extend(pos)
        mesh_data.append({"positions": positions, "triangles": tris, "degenerate": degenerate, "faces": faces})

    nodes = doc.get("nodes", [])
    need(nodes, "keine Knoten exportiert")
    parent = {}
    for i, nd in enumerate(nodes):
        for c in nd.get("children", []):
            ref(nodes, c, "Kind-Knoten")
            need(c not in parent, f"Knoten {c} hat mehrere Eltern")
            parent[c] = i
        if "mesh" in nd:
            ref(meshes, nd["mesh"], "Knoten-Mesh")
        if "skin" in nd:
            ref(doc.get("skins", []), nd["skin"], "Skin")
    for i in range(len(nodes)):
        seen, k = set(), i
        while k in parent:
            need(k not in seen, "Zyklus in der Knoten-Hierarchie")
            seen.add(k)
            k = parent[k]
    local = [node_matrix(nd) for nd in nodes]
    skins = doc.get("skins", [])
    for s in skins:
        joints = s.get("joints", [])
        need(joints, "Skin ohne Joints")
        for j in joints:
            ref(nodes, j, "Skin-Joint")
        if "inverseBindMatrices" in s:
            acc = ref(g.accessors, s["inverseBindMatrices"], "inverseBindMatrices")
            need(acc.get("type") == "MAT4" and acc.get("count") == len(joints), "inverseBindMatrices passen nicht zu den Joints")
            g.read(s["inverseBindMatrices"])
    scenes = doc.get("scenes", [])
    scene = ref(scenes, doc.get("scene", 0), "Standard-Szene")
    roots = scene.get("nodes", [])
    for r in roots:
        ref(nodes, r, "Szenen-Wurzel")
        need(r not in parent, "Szenen-Wurzel hat einen Elternknoten")
    need(len(set(roots)) == len(roots), "Szenen-Wurzel doppelt")

    lo, hi = [math.inf] * 3, [-math.inf] * 3
    instances, triangles, degenerate = 0, 0, 0
    hulls = []
    stack = [(r, IDENTITY) for r in roots]
    while stack:
        i, pm = stack.pop()
        wm = matmul(pm, local[i])
        nd = nodes[i]
        if "mesh" in nd:
            md = mesh_data[nd["mesh"]]
            instances += 1
            triangles += md["triangles"]
            degenerate += md["degenerate"]
            rows = [wm[r] for r in range(3)]
            for p in md["positions"]:
                for c, row in enumerate(rows):
                    v = row[0] * p[0] + row[1] * p[1] + row[2] * p[2] + row[3]
                    if v < lo[c]:
                        lo[c] = v
                    if v > hi[c]:
                        hi[c] = v
            if collision:
                hulls.append({"name": nd.get("name", ""), **hull_stats(md, rows)})
        stack.extend((c, wm) for c in nd.get("children", []))
    need(triangles > 0, "Standard-Szene enthält keine Dreiecke")
    dims = [hi[c] - lo[c] for c in range(3)]
    return {
        "path": path, "ok": True, "bytes": g.size, "triangles": triangles, "degenerate_triangles": degenerate,
        "meshes": len(meshes), "mesh_instances": instances, "nodes": len(nodes), "materials": len(materials),
        "images": imgs, "skins": len(skins),
        "joint_names": sorted({nodes[j].get("name", "") for s in skins for j in s.get("joints", [])}),
        "node_names": [nd.get("name", "") for nd in nodes],
        "bounds_min": lo, "bounds_max": hi,
        # glTF ist Y-oben: Blender-Maße (X, Y, Z) = glTF (X, Z, Y)
        "dimensions_blender_m": [dims[0], dims[2], dims[1]],
        "collision": collision, "hulls": hulls,
    }


HULL_CONVEX_TOL = 1e-3     # m: echte Dellen fallen auf, Rundungsrauschen der float32-Daten nicht


def hull_stats(md, rows):
    """Vertexzahl (eindeutige Positionen) und Konvexität einer Kollisionshülle in Weltkoordinaten."""
    def w(p):
        return tuple(row[0] * p[0] + row[1] * p[1] + row[2] * p[2] + row[3] for row in rows)
    pts = {tuple(round(c, 5) for c in w(p)) for p in md["positions"]}
    verts = list(pts)
    ctr = [sum(v[c] for v in verts) / len(verts) for c in range(3)]
    worst = 0.0
    for a, b, c in md["faces"]:
        a, b, c = w(a), w(b), w(c)
        u = [b[k] - a[k] for k in range(3)]
        v = [c[k] - a[k] for k in range(3)]
        n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
        ln = math.sqrt(sum(x * x for x in n))
        longest = max(math.dist(a, b), math.dist(b, c), math.dist(c, a))
        if ln < 1e-12 or ln / longest < 5e-4:          # Splitter (Höhe < 0,5 mm): Normale in float32 unbestimmt
            continue
        n = [x / ln for x in n]
        d = sum(n[k] * a[k] for k in range(3))
        if sum(n[k] * ctr[k] for k in range(3)) > d:          # nach außen orientieren
            n, d = [-x for x in n], -d
        worst = max(worst, max(sum(n[k] * p[k] for k in range(3)) - d for p in verts))
    return {"verts": len(verts), "max_outside_m": worst, "convex": worst <= HULL_CONVEX_TOL}


# ----------------------------------------------------------------------------- Abgleich mit dem Report

def check_report(report_path, results):
    """Fehlerliste für ein Asset (leer = in Ordnung)."""
    errors = []
    base = os.path.dirname(report_path)
    try:
        with open(report_path, encoding="utf-8") as fh:
            rep = json.load(fh)
    except (OSError, ValueError) as err:
        return [f"{report_path}: unlesbar ({err})"]
    name = rep.get("asset", os.path.basename(report_path))
    if rep.get("schema_version", 1) < REPORT_SCHEMA:
        return [f"{name}: Report-Schema {rep.get('schema_version', 1)} ist zu alt – mit aktuellem vehicle_generator.py neu exportieren"]

    def res(rel):
        r = results.get(os.path.normpath(os.path.join(base, rel)))
        return r if r and r.get("ok") else None
    for f in rep.get("files", []):
        path = os.path.join(base, f["path"])
        if not os.path.isfile(path):
            errors.append(f"{name}: Datei fehlt: {f['path']}")
        elif os.path.getsize(path) != f["bytes"] or sha256(path) != f["sha256"]:
            errors.append(f"{name}: Datei verändert (Größe/SHA-256): {f['path']}")
    lods = sorted(rep.get("lods", {}).items(), key=lambda kv: int(kv[0][3:]))
    tris = [lod["triangles"] for _, lod in lods]
    if any(b >= a for a, b in zip(tris, tris[1:])):
        errors.append(f"{name}: Dreiecke nehmen über die LODs nicht ab: {tris}")
    nodes_needed = {name} | {f"{name}_{k}_Pivot" for k in rep.get("pivots", {}) if k != "root"}
    nodes_needed |= {f"SOCKET_{k}" for k in rep.get("sockets", {})}
    for key, lod in lods:
        for rel in lod.get("files", []):
            if not rel.endswith(".glb"):
                continue
            r = res(rel)
            if r is None:
                continue                                    # Formatfehler wird bei der Datei gemeldet
            if r["triangles"] != lod["triangles"]:
                errors.append(f"{name}: {rel} hat {r['triangles']} Dreiecke, Report {key}: {lod['triangles']}")
            missing = sorted(nodes_needed - set(r["node_names"]))
            if missing:
                errors.append(f"{name}: {rel}: Knoten fehlen: {', '.join(missing)}")
            if key == "LOD0":
                dims = rep.get("dimensions_m", [])
                if len(dims) == 3 and any(abs(a - b) > 0.002 for a, b in zip(r["dimensions_blender_m"], dims)):
                    errors.append(f"{name}: {rel}: Maße {[round(x, 3) for x in r['dimensions_blender_m']]} ≠ Report {dims}")
    hull_names = {h["name"] for h in rep.get("collision", [])}
    for rel in rep.get("collision_files", []):
        if not rel.endswith(".glb"):
            continue
        r = res(rel)
        if r is None:
            continue
        got = {h["name"] for h in r["hulls"]}
        if got != hull_names:
            errors.append(f"{name}: {rel}: Hüllen {sorted(got ^ hull_names)} passen nicht zum Report")
        for h in r["hulls"]:
            if h["verts"] > HULL_MAX_VERTS:
                errors.append(f"{name}: {rel}: {h['name']} hat {h['verts']} Vertices (max. {HULL_MAX_VERTS})")
            if not h["convex"]:
                errors.append(f"{name}: {rel}: {h['name']} ist nicht konvex ({h['max_outside_m'] * 1000:.2f} mm)")
    bones = set(rep.get("bones", []))
    for f in rep.get("files", []):
        rel = f["path"]
        if os.path.basename(rel).startswith("SK_") and rel.endswith(".glb"):
            r = res(rel)
            if r is not None and (r["skins"] != 1 or set(r["joint_names"]) != bones):
                errors.append(f"{name}: {rel}: Skelett passt nicht zum Report (Knochen {sorted(set(r['joint_names']) ^ bones)})")
    return errors


def validate(target):
    """Ordner (rekursiv) oder einzelne Datei prüfen → Gesamtbericht."""
    target = os.path.abspath(target)
    if os.path.isdir(target):
        glbs, reports = [], []
        for root, _, names in os.walk(target):
            glbs += [os.path.join(root, n) for n in names if n.lower().endswith(".glb")]
            reports += [os.path.join(root, n) for n in names if n.endswith("_report.json")]
    else:
        glbs, reports = [target], []
    results, errors = {}, []
    for p in sorted(glbs):
        try:
            results[os.path.normpath(p)] = check_glb(p)
        except (OSError, ValueError, KeyError, TypeError, IndexError, struct.error) as err:
            results[os.path.normpath(p)] = {"path": p, "ok": False, "error": str(err)}
            errors.append(f"{os.path.relpath(p, os.path.dirname(target))}: {err}")
    if not glbs:
        errors.append("keine .glb-Dateien gefunden")
    for rp in sorted(reports):
        errors += check_report(rp, results)
    for r in results.values():                  # Positionslisten nicht in den JSON-Bericht schreiben
        r.pop("node_names", None)
    return {"ok": not errors, "files": list(results.values()), "reports": sorted(reports), "errors": errors}


def main(argv=None):
    ap = argparse.ArgumentParser(description="GLB-Exporte von vehicle_generator.py prüfen (ohne Blender).")
    ap.add_argument("path", help="Ausgabeordner (rekursiv) oder einzelne .glb-Datei")
    ap.add_argument("--json", action="store_true", help="vollständigen Bericht als JSON ausgeben")
    args = ap.parse_args(argv)
    rep = validate(args.path)
    if args.json:
        print(json.dumps(rep, indent=2, ensure_ascii=False, allow_nan=False, default=str))
    else:
        for r in rep["files"]:
            rel = os.path.relpath(r["path"], os.path.abspath(args.path) if os.path.isdir(args.path) else os.path.dirname(r["path"]))
            if r["ok"]:
                extra = f", {len(r['hulls'])} Hüllen" if r["collision"] else f", {len(r['images'])} Bilder"
                if r["degenerate_triangles"]:
                    extra += f", Warnung: {r['degenerate_triangles']} degenerierte Dreiecke"
                print(f"OK      {rel}: {r['triangles']:,}".replace(",", ".") + f" Dreiecke{extra}")
            else:
                print(f"FEHLER  {rel}: {r['error']}")
        for e in rep["errors"]:
            if not any(e.endswith(r.get("error", "\0")) for r in rep["files"]):
                print(f"FEHLER  {e}")
        print(f"{len(rep['files'])} Dateien, {len(rep['reports'])} Reports: "
              + ("alles in Ordnung" if rep["ok"] else f"{len(rep['errors'])} Fehler"))
    return 0 if rep["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
