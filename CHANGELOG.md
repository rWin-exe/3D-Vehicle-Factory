# Änderungen

## Unveröffentlicht

### Geändert
- **Motorboot überarbeitet und deutlich detailreicher** (LOD0 40 304 → 59 570 Dreiecke):
  - *Rumpf und Deck:* Zierstreifen (Cove-Stripe + Haarlinie) entlang der Deckskante, Modellname an den
    Bordwänden, rutschfestes Noppendeck (prozedurales Normalmapping), Teakbelag auf den Badeplattformen,
    Heckdusche, Klüsen und zusätzliche Vorschiffsklampen, Ankerkasten-Luke mit Dichtung, Scharnieren und
    Griffmulde, Bug-Ankerrolle mit Pflugscharanker und Schäkel.
  - *Cockpit:* Polster neu als gerundete, gewölbte Kissen mit Steppnähten statt Quader; Fahrer-/Beifahrersitze mit
    Sitzkasten, Klappwulst, Bucket-Rückenlehne mit Seitenwangen, Kopfstütze und Armlehnen; Polsterfarbe je
    Lackierung (`BOAT_LIVERIES`: `seat`, `deck`, `liner`); Armaturenbrett mit Drehzahlmesser, Tacho, Tankanzeige,
    Plotter, Schalterleiste, Zündschloss mit Schlüssel, Kill-Schalter, Handschuhfach, Uhr, Steckdose und
    Becherhaltern; Lenkradknauf; Seitentaschen, Lautsprecher, Feuerlöscher, Bootshaken, Bodenluke und
    Stufenkante; Windschutzscheibe mit Haltegriff, Fußleiste, Fußplatten, Wischern und Türgriff.
  - *Außenborder:* Haube mit gerundetem Heck und abgesetzter Unterhaube (Trennfuge im Lack), Lufteinlass-Lamellen,
    Griffmulde, Farbstreifen, Typenschild, dickerer Schaft, Getriebe mit Wassereinlässen, Staurohr und
    Trimmflosse, Propellermutter, Spiegelhalterung als Klemmkonsolen mit Klemmschrauben.
  - Neue Baukasten-Funktionen `plate3d` und `round_corners`; Dekore am Motor als schmale, einzeln projizierte
    Gitter (statt isotrop unterteilter Flächen) – spart rund 10 000 Dreiecke.
  - Bewegungsbereiche per Mesh-Überschneidungstest erneut geprüft: Kippen 0–65°, Lenken ±35°, Durchgangstür,
    Badeleiter, Lenkrad, Gashebel und Propeller sind frei; bei Kippwinkeln über 35° bleibt nur noch ±15° Lenken
    kollisionsfrei (in der README vermerkt).
- **Geländewagen überarbeitet:** Gürtellinie 1,16 → 1,30 m und Dach 1,98 m, Karosserie 6 cm breiter – der Unterbau
  wirkt nicht mehr zu schmal unter dem hohen Glasaufbau. Größerer Kühlergrill mit Chromrahmen, größere
  Scheinwerfer, Zusatzscheinwerfer auf dem Stoßfänger, Lufthutze, Seitenschutzleisten, Seitenblinker,
  Rückleuchten-Cluster und Rückstrahler, kräftigere Kotflügelverbreiterungen; Motor sitzt 10 cm höher im
  Motorraum, Schnorchel-Einlass über der Schutzleiste. Bewegungsbereiche erneut kollisionsfrei geprüft.

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
