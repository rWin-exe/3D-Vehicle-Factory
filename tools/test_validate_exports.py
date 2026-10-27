#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit-Tests für validate_exports.py mit winzigen, absichtlich kaputten GLB-Dateien (ohne Blender).

  python -m unittest discover -s tools -v
"""

import hashlib
import json
import os
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import validate_exports as ve  # noqa: E402

TRI = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)]
QUAD = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (0.0, 1.0, 0.0)]
TET = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]
TET_TRIS = ((0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3))
PNG_4x4 = b"\x89PNG\r\n\x1a\n" + struct.pack(">I", 13) + b"IHDR" + struct.pack(">II", 4, 4) + b"\x08\x06\0\0\0" + b"\0" * 8


class Doc:
    """Minimaler GLB-Baukasten für die Testfälle."""

    def __init__(self):
        self.bin = bytearray()
        self.gltf = {"asset": {"version": "2.0"}, "scene": 0, "scenes": [{"nodes": []}], "nodes": [], "meshes": [],
                     "accessors": [], "bufferViews": [], "buffers": [{}]}

    def view(self, data, stride=None):
        self.bin += b"\0" * (-len(self.bin) % 4)
        v = {"buffer": 0, "byteOffset": len(self.bin), "byteLength": len(data)}
        if stride:
            v["byteStride"] = stride
        self.bin += data
        self.gltf["bufferViews"].append(v)
        return len(self.gltf["bufferViews"]) - 1

    def accessor(self, rows, typ, ct=5126, minmax=False, stride=None):
        fmt = {5126: "f", 5121: "B", 5123: "H", 5125: "I"}[ct] * len(rows[0])
        size = struct.calcsize("<" + fmt)
        pad = b"\0" * ((stride or size) - size)
        data = b"".join(struct.pack("<" + fmt, *r) + pad for r in rows)
        a = {"bufferView": self.view(data, stride), "componentType": ct, "type": typ, "count": len(rows)}
        if minmax:
            a["min"] = [min(r[c] for r in rows) for c in range(len(rows[0]))]
            a["max"] = [max(r[c] for r in rows) for c in range(len(rows[0]))]
        self.gltf["accessors"].append(a)
        return len(self.gltf["accessors"]) - 1

    def mesh(self, positions=TRI, tris=((0, 1, 2),), uv=True, normal=(0.0, 0.0, 1.0), stride=None, extra=None):
        at = {"POSITION": self.accessor(positions, "VEC3", minmax=True, stride=stride),
              "NORMAL": self.accessor([normal] * len(positions), "VEC3")}
        if uv:
            at["TEXCOORD_0"] = self.accessor([(p[0], p[1]) for p in positions], "VEC2")
        at.update(extra or {})
        idx = self.accessor([(i,) for t in tris for i in t], "SCALAR", 5123)
        self.gltf["meshes"].append({"primitives": [{"attributes": at, "indices": idx}]})
        return len(self.gltf["meshes"]) - 1

    def node(self, name, mesh=None, parent=None, **trs):
        n = dict(name=name, **trs)
        if mesh is not None:
            n["mesh"] = mesh
        self.gltf["nodes"].append(n)
        i = len(self.gltf["nodes"]) - 1
        if parent is None:
            self.gltf["scenes"][0]["nodes"].append(i)
        else:
            self.gltf["nodes"][parent].setdefault("children", []).append(i)
        return i

    def image(self, data):
        self.gltf["images"] = [{"bufferView": self.view(data), "mimeType": "image/png"}]
        self.gltf["textures"] = [{"source": 0}]
        self.gltf["materials"] = [{"pbrMetallicRoughness": {"baseColorTexture": {"index": 0}}}]

    def bytes(self):
        self.bin += b"\0" * (-len(self.bin) % 4)
        self.gltf["buffers"][0]["byteLength"] = len(self.bin)
        js = json.dumps(self.gltf).encode()
        js += b" " * (-len(js) % 4)
        body = (struct.pack("<II", len(js), ve.CHUNK_JSON) + js
                + struct.pack("<II", len(self.bin), ve.CHUNK_BIN) + bytes(self.bin))
        return struct.pack("<4sII", b"glTF", 2, 12 + len(body)) + body


def triangle(**kw):
    d = Doc()
    d.node("SM_Test", d.mesh(**kw))
    return d


class FileTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def write(self, data, name="SM_Test.glb"):
        path = os.path.join(self.tmp.name, name)
        with open(path, "wb") as fh:
            fh.write(data)
        return path

    def check(self, doc_or_bytes, name="SM_Test.glb"):
        data = doc_or_bytes.bytes() if isinstance(doc_or_bytes, Doc) else doc_or_bytes
        return ve.check_glb(self.write(data, name))

    def test_valid_triangle(self):
        r = self.check(triangle())
        self.assertEqual(r["triangles"], 1)
        self.assertEqual(r["dimensions_blender_m"], [1.0, 0.0, 1.0])       # glTF Y-oben → Blender Z-oben

    def test_index_out_of_range(self):
        with self.assertRaisesRegex(ve.Invalid, "Index außerhalb"):
            self.check(triangle(tris=((0, 1, 7),)))

    def test_missing_uv(self):
        with self.assertRaisesRegex(ve.Invalid, "TEXCOORD_0"):
            self.check(triangle(uv=False))

    def test_collision_needs_no_uv(self):
        r = self.check(triangle(uv=False), "SM_Test_Collision.glb")
        self.assertTrue(r["collision"])

    def test_trailing_bytes(self):
        with self.assertRaisesRegex(ve.Invalid, "Header-Länge"):
            self.check(triangle().bytes() + b"xxxx")

    def test_nan_position(self):
        with self.assertRaisesRegex(ve.Invalid, "NaN"):
            self.check(triangle(positions=[(float("nan"), 0.0, 0.0)] + TRI[1:]))

    def test_minmax_must_match(self):
        d = triangle()
        d.gltf["accessors"][0]["max"] = [5.0, 1.0, 0.0]
        with self.assertRaisesRegex(ve.Invalid, "min/max"):
            self.check(d)

    def test_normals_must_be_unit(self):
        with self.assertRaisesRegex(ve.Invalid, "Normalen"):
            self.check(triangle(normal=(0.0, 0.0, 2.0)))

    def test_interleaved_positions(self):
        self.assertEqual(self.check(triangle(stride=16))["triangles"], 1)

    def test_node_cycle(self):
        d = triangle()
        d.gltf["nodes"][0]["children"] = [0]
        with self.assertRaisesRegex(ve.Invalid, "Eltern|Zyklus"):
            self.check(d)

    def test_unnormalized_quaternion(self):
        d = triangle()
        d.gltf["nodes"][0]["rotation"] = [0.0, 0.0, 0.5, 0.5]
        with self.assertRaisesRegex(ve.Invalid, "Quaternion"):
            self.check(d)

    def test_world_bounds_follow_hierarchy(self):
        d = Doc()
        root = d.node("SM_Test", translation=[5.0, 6.0, 7.0])
        d.node("child", d.mesh(), root, scale=[2.0, 3.0, 1.0])
        r = self.check(d)
        self.assertEqual(r["bounds_min"], [5.0, 6.0, 7.0])
        self.assertEqual(r["dimensions_blender_m"], [2.0, 0.0, 3.0])

    def test_image_signature(self):
        d = triangle()
        d.image(PNG_4x4)
        self.assertEqual(self.check(d)["images"][0]["width"], 4)
        d = triangle()
        d.image(b"GIF89a" + b"\0" * 20)
        with self.assertRaisesRegex(ve.Invalid, "PNG-Signatur"):
            self.check(d)

    def test_empty_directory_fails(self):
        self.assertFalse(ve.validate(self.tmp.name)["ok"])


class ReportTests(unittest.TestCase):
    """Abgleich Exporte ↔ SM_*_report.json (Hashes, LODs, Knoten, Hüllen, Skelett)."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.dir = self.tmp.name

    def lod(self, tris, pivot=True):
        d = Doc()
        root = d.node("SM_Test", d.mesh(QUAD, ((0, 1, 2), (0, 2, 3))[:tris]))
        if pivot:
            piv = d.node("SM_Test_Wheel_Pivot", parent=root)
            d.node("SM_Test_Wheel", d.mesh(), piv)
        d.node("SOCKET_Seat", parent=root)
        return d

    def hull(self, kind):
        d = Doc()
        root = d.node("SM_Test")
        if kind == "tet":
            m = d.mesh(TET, TET_TRIS, uv=False)
        elif kind == "two_tets":                      # zwei getrennte Körper → nicht konvex
            far = [(x + 3.0, y, z) for x, y, z in TET]
            m = d.mesh(TET + far, TET_TRIS + tuple(tuple(i + 4 for i in t) for t in TET_TRIS), uv=False)
        else:                                         # 130 Vertices (126 innen) → über dem Budget
            inner = [(0.1 + 0.001 * i, 0.1, 0.1) for i in range(126)]
            m = d.mesh(TET + inner, TET_TRIS, uv=False)
        d.node("UCX_SM_Test_00", m, root)
        return d

    def skeleton(self):
        d = Doc()
        j0 = d.node("root")
        j1 = d.node("Wheel", parent=j0)
        joints = d.accessor([(0, 1, 0, 0)] * 3, "VEC4", 5121)
        weights = d.accessor([(1.0, 0.0, 0.0, 0.0)] * 3, "VEC4")
        mesh = d.mesh(extra={"JOINTS_0": joints, "WEIGHTS_0": weights})
        ibm = d.accessor([tuple(float(r == c) for c in range(4) for r in range(4))] * 2, "MAT4")
        d.gltf["skins"] = [{"joints": [j0, j1], "inverseBindMatrices": ibm}]
        n = d.node("SK_Test", mesh)
        d.gltf["nodes"][n]["skin"] = 0
        return d

    def build(self, lod1_tris=1, drop_pivot=False, hull="tet", bones=("root", "Wheel"), schema=ve.REPORT_SCHEMA,
              wheel=None):
        docs = {"SM_Test.glb": self.lod(2), "SM_Test_LOD1.glb": self.lod(lod1_tris, pivot=not drop_pivot),
                "SM_Test_Collision.glb": self.hull(hull), "SK_Test.glb": self.skeleton()}
        files = []
        for name, d in docs.items():
            path = os.path.join(self.dir, name)
            data = d.bytes()
            with open(path, "wb") as fh:
                fh.write(data)
            files.append({"path": name, "bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()})
        rep = {"schema_version": schema, "asset": "SM_Test", "dimensions_m": [1.0, 0.0, 1.0],
               "lods": {"LOD0": {"triangles": 3, "files": ["SM_Test.glb"]},
                        "LOD1": {"triangles": lod1_tris + (0 if drop_pivot else 1), "files": ["SM_Test_LOD1.glb"]}},
               "pivots": {"root": {}, "Wheel": wheel or {}}, "sockets": {"Seat": {}},
               "collision": [{"name": "UCX_SM_Test_00"}], "collision_files": ["SM_Test_Collision.glb"],
               "bones": list(bones), "files": files}
        with open(os.path.join(self.dir, "SM_Test_report.json"), "w", encoding="utf-8") as fh:
            json.dump(rep, fh)
        return ve.validate(self.dir)

    def assertError(self, result, pattern):
        self.assertFalse(result["ok"])
        self.assertTrue(any(pattern in e for e in result["errors"]), result["errors"])

    def test_consistent_asset(self):
        r = self.build()
        self.assertTrue(r["ok"], r["errors"])

    def test_changed_file(self):
        self.build()
        with open(os.path.join(self.dir, "SM_Test_LOD1.glb"), "wb") as fh:
            fh.write(triangle(positions=[(0.0, 0.0, 0.0), (2.0, 0.0, 0.0), (0.0, 2.0, 0.0)]).bytes())
        self.assertError(ve.validate(self.dir), "verändert")

    def test_lods_must_decrease(self):
        self.assertError(self.build(lod1_tris=2), "nehmen über die LODs nicht ab")

    def test_missing_pivot_node(self):
        self.assertError(self.build(drop_pivot=True), "Knoten fehlen: SM_Test_Wheel_Pivot")

    def test_old_report_schema(self):
        self.assertError(self.build(schema=1), "zu alt")

    def test_non_convex_hull(self):
        self.assertError(self.build(hull="two_tets"), "nicht konvex")

    def test_hull_vertex_budget(self):
        self.assertError(self.build(hull="big"), "130 Vertices")

    def test_skeleton_must_match_bones(self):
        self.assertError(self.build(bones=("root", "Wheel", "Steering")), "Skelett passt nicht")

    def test_pivot_metadata_ok(self):
        r = self.build(wheel={"motion": "aim", "target": "root", "limits": [-30, 30]})
        self.assertTrue(r["ok"], r["errors"])

    def test_aim_target_must_exist(self):
        self.assertError(self.build(wheel={"motion": "aim", "target": "Mount"}), "Aim-Ziel 'Mount'")

    def test_limits_must_be_ordered(self):
        self.assertError(self.build(wheel={"limits": [30, -30]}), "ungültiger Bewegungsbereich")


if __name__ == "__main__":
    unittest.main()
