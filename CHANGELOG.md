# Änderungen

## Unveröffentlicht

### Neu
- **Elektro-Geländewagen** (`--vehicle ev`): fünftüriger Allrad-SUV (Karosserie 4,70 m, Breite 1,94 m, Dach 1,77 m,
  Radstand 2,92 m, Reifen 255/60 R20 All-Terrain). Karosserie aus zwei Spanten-Körpern: Unterbau mit Schulterkante,
  Lichtkante über dem Schweller, Powerdome und stark verrundeten Ecken, darauf die eingezogene Kabine mit gewölbter,
  ≈ 58° geneigter Windschutzscheibe (Profil am Seitenfoto der Vorlage vermessen); Hülle als Außenhaut − Innenhaut mit echten Ausschnitten und lackierten Falzen.
  Schwarzes Dach und schwarze Fensterrahmen, D-Säule in Wagenfarbe, Chromleiste um die ansteigende hintere
  Seitenscheibe. Projektions-Werkzeuge (`ev_tools`) legen Leuchten, Blenden und aufgesetzte Verkleidungen auf die
  gewölbte Außenhaut – auch polar um die Fahrzeugecken. Front mit vertieftem Grill (16 Lamellen, beleuchtete Raute),
  um die Ecke laufenden Scheinwerfern mit drei L-förmigen Tagfahrlicht-Segmenten, Nebelleuchten-Taschen, schwarzem
  Stoßfänger und silbernem Unterfahrschutz mit Bergeösen; Heck mit Leuchtband, das als Rückleuchte um die Ecken
  läuft, Unterfahrschutz mit drei Blöcken und zwei Endrohr-Blenden, Winkel-Rückstrahlern und Dachspoiler an der
  Klappe; trapezförmige Radläufe, Trittbretter, Reling mit Gepäckplattform, Ladeklappe mit CCS-Dose. Technik:
  Unterflur-Batterie, je ein E-Motor mit Inverter vorn und hinten, Einzelradaufhängung mit Querlenkern und
  Federbeinen (Aim-Pivots), Vorder- und Hinterachslenkung, Frunk mit Hochvolt-Leitungen. Innenraum mit geschichtetem
  Armaturenbrett (Kombiinstrument, frei stehendes Zentraldisplay, Beifahrerdisplay, satiniertes Zierband, Chromlinie
  mit Ambientelicht, Lüftungsdüsen, Klimabedienteil), Lenkrad mit Tastenfeldern, hoher Mittelkonsole mit Wählhebel
  und geteilter Armlehne, ausgeformten Ledersitzen mit Ziernähten, Türverkleidungen an den Tür-Pivots, Dachkonsole
  und Sonnenblenden sowie einem Laderaum mit gerippter Matte, Radhausverkleidungen, Ladekantenschutz und zwei
  Gasdruckfedern an der Heckklappe (Aim-Pivots). Pivots für vier Türen, Haube, Heckklappe, Ladeklappe, Lenkrad,
  Federweg, Lenkung und Querlenker; Bewegungsbereiche kollisionsfrei geprüft. Fünf Lackierungen
  (`--ev-livery forest|graphite|sand|white|blue`), Schriftzug per `--ev-brand` (Standard `VOLT`).

- **Elektro-Geländewagen „Classic“** (`--vehicle ev_classic`, `SM_EVClassic`): die frühere, höhere Karosserie
  (Radstand 2,83 m, Dach 1,84 m, Sechseck-Grill, senkrechte Heckklappe) als eigenes Modell mit dem aktuellen
  Innenraum; `ev_interior` richtet Rückbank, Laderaum, Spritzwand und Türverkleidungen jetzt nach der Geometrie
  der jeweiligen Karosserie aus.

### Geändert
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
