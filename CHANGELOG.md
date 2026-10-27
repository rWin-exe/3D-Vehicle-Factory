# Änderungen

## 1.1.0 – 2026-10-27

### Neu
- **Motorboot** (`--vehicle boat`): Bowrider der 6,4-m-Klasse mit Tiefen-V-Rumpf, Spritzleisten, Teak-Laufdeck,
  Konsolen, gebogener Windschutzscheibe mit Durchgangstür, Sitzen, Badeplattformen mit Badeleiter, Reling,
  Positionslichtern und 200-PS-Außenborder (Pivots zum Kippen, Lenken und für den Propeller); vier Lackierungen
  (`--boat-livery navy|white|red|graphite`), Ursprung auf der Wasserlinie, Auftriebs-Sockets.

## 1.0.0 – 2026-10-27

Erste Veröffentlichung von **3D Vehicle Factory**: ein Python-Script für Blender (bpy), das vier
Fahrzeuge vollständig prozedural erzeugt und als engine-fertige Assets exportiert.

### Fahrzeuge
- **Enduro** – 300er Zweitakt-Hardenduro mit Gitterrohrrahmen, USD-Gabel, Speichenrädern mit einzeln
  modellierten Stollen, Kette mit ~115 Gliedern, Auspuffbirne, Kühler, Leitungen und Grafik-Kit.
- **Hubschrauber** – leichter Turbinenheli mit echten Fenster- und Türausschnitten, vier Türen,
  Innenraum, Hauptrotor mit NACA-Profilblättern, Heckrotor und Kufen; vier Lackierungen.
- **Flugzeug** – Hochdecker der C172-Klasse mit gewölbtem Flügel, Klappen, Quer-, Höhen- und
  Seitenruder, Propeller mit realer Steigungsverteilung, Bugradfahrwerk; vier Lackierungen.
- **Geländewagen** – fünftüriger 4x4 mit Leiterrahmen, Starrachsen mit Verschränkung, lenkbaren
  Achsschenkeln, teleskopierenden Federbeinen, Motorraum mit Reihen-Sechszylinder-Turbodiesel,
  Seilwinde, Schnorchel, Dachträger und Hecktür mit Reserverad; vier Lackierungen.

### Pipeline
- Pivots für alle beweglichen Teile (Bewegungsart, positive Richtung, Aim-Ziel, geprüfte
  Bewegungsbereiche als Custom Properties bzw. glTF-`extras`), optionales Skeletal Mesh (`--rig`).
- UV-Atlas und PBR-Bake (BaseColor, Normal OpenGL/DirectX, ORM, Emissive; Unity: MetallicSmoothness,
  MaskMap); AO mit auseinandergezogenen beweglichen Teilen.
- LOD0–LOD3, konvexe UCX-Kollisionshüllen (≤ 128 Vertices, auch als GLB mit Godot-Suffix), Sockets.
- Export als FBX, GLB und `.blend` für Unreal, Unity, Godot oder generisch, JSON-Report mit
  Polycounts, Pivots, Optionen und SHA-256 jeder Datei.

### Prüfwerkzeuge
- `tools/validate_exports.py` – GLB- und Report-Prüfung ohne Blender, mit Unit-Tests.
- `tools/smoke_blender.py` – Rundlauf in Blender: Pivots, Achsen, Sockets, Kollision und Skelett
  nachgemessen.

### Voraussetzungen
Blender bzw. bpy 5.2 (getestet mit 5.2.2; ausgelegt für 4.2+). Alle Fahrzeuge exportieren:
`blender --background --factory-startup --python-exit-code 1 --python vehicle_generator.py -- --out export`
