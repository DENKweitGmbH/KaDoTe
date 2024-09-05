# Projekt KaDoTe

## Python setup

- Install <https://github.com/astral-sh/uv>
- `uv sync`

## OPCUA Server

Nutzung via OPCUA-Bibliothek für Python siehe Scripte `Server.py` und `Client.py`.
Im Server-Script wird das Bildaufnehmen testweise durch drücken der taste "i" ausgelöst.

## IDS-Kamera

- IDS Kamera ansteuerbar über ids_peak für Python installierbar über pip
- IDS-Kamera erfordert PoE-Adapter (haben wir)

## Wenglor Sensor

- GigEVision Server über ShapeDriveGigEInterface.exe starten
- Sensor hat feste IP `192.168.100.1`
- Start über: `ShapeDriveGigEInterface.exe -s 192.168.100.3 -i 192.168.100.1 -n 0 -d`

```shell
    -i: IP des Sensors
    -s: IP des Netzwerkadapters
    -d: Debug messages to console
```

Wenglor-Software zum Testen:

- Software_VisionApp_Demo_3D_2.0.0
- Starten --> Auf Eintrag in Deviceliste doppeklicken
- Wenn Trigger auf "Software" gesetzt ist, können durch Python-Script derzeit keine Daten ausgelesen werden. Trigger muss auf "Intern" stehen. Evtl. im Script setzen.

Auslesen der Wenglor-Sensordaten mit Harvester-Bibliothek
<https://github.com/genicam/harvesters>
Man benötigt ein "producer file" --> `mvGenTLProducer.cti` (liegt mit im Verzeichnis)
