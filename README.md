# Projekt KaDoTe

## Python environment setup

Cloning repository

```shell
git clone https://github.com/DENKweitGmbH/KaDoTe.git KaDoTe
cd KaDoTe
git checkout main
```

### Dependencies

* Python 3.11
* Python Packages see: [pyproject.toml](/pyproject.toml)

### Install Python and dependencies

Via [`uv`](https://docs.astral.sh/uv/): [Install `uv`](https://github.com/astral-sh/uv?tab=readme-ov-file#installation)

```shell
uv sync --frozen --all-extras
```

Installing packages via conda instead of uv should work as well, but might not be as easy.

### Update dependencies

* See [`uv` documentation](https://docs.astral.sh/uv/)
* Add a package via `uv add <package>`
* Upgrade all packages via `uv sync --upgrade`
* Upgrade one package via `uv sync -P <package>`

## How to run Analysis Client

```shell
uv run client.py
```

## OPCUA Test Server

Test the client implementation by running a mock server:

```shell
uv run server.py
```

The mock server requests an image to be taken every x seconds or by user interaction (pressing `i`).

## IDS-Camera

* IDS Camera is controlled by `ids_peak` python library
* IDS Camera needs a PoE-adapter

## Wenglor Sensor

* Start GigEVision server by running ShapeDriveGigEInterface.exe
* Sensor has static IP `192.168.100.1`
Example: `ShapeDriveGigEInterface.exe -s 192.168.100.3 -i 192.168.100.1 -n 0 -d`

Options are:

```shell
    -i: IP des Sensors
    -s: IP des Netzwerkadapters
    -d: Debug messages to console
```

### Wenglor-Software zum Testen

* Software_VisionApp_Demo_3D_2.0.0
* Start --> Doubleclick entry in device list
* If trigger is set to `Software`, we currently cannot read data from out Python scripts. Trigger must be set to `Intern`. Maybe we can set this in script?

Data is read with `Harvesters` library: <https://github.com/genicam/harvesters>
Needs a "producer file": [mvGenTLProducer.cti](/mvGenTLProducer.cti) (included in this repository)
