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

Installing packages via `conda` or `pip` instead of `uv` should work as well, but might not be as easy.

### Update dependencies

* See [`uv` documentation](https://docs.astral.sh/uv/)
* Add a package via `uv add <package>`
* Upgrade all packages via `uv sync --upgrade`
* Upgrade one package via `uv sync -P <package>`

## How to run Analysis Client

```shell
uv run client.py --opcua-config <path/to/opcua-config.json>
```

See `uv run client.py --help` for available parameters:

```bash
```

The most important options are:

* `--opcua-config <path/to/opcua-config.json>`: See [schema](/schemas/opcua-server-config.schema.json) for format, optionally with `--opcua-password`, example OPCUA configs are found in [configs](/configs)
* `--token <path/to/token.txt>` to be able to use the image analysis

### Local Testing

For local testing the options `--allow-missing-hardware`, `--local-only` and the OPCUA test server might be useful.

#### OPCUA Local Test Server

Test the client implementation by running a local opcua test server:

```shell
uv run server.py
```

The test server requests an image to be taken every x seconds (or by user interaction (pressing `i`) on Windows).

NOTE: The test server and client currently use normal string type for the result node instead of array of string!

## IDS-Camera

* IDS Camera is controlled by `ids_peak` python library
  * Needs to be installed: <https://en.ids-imaging.com/download-peak.html>
* IDS Camera needs a Power-over-Ethernet (PoE) adapter

## Wenglor Depth Sensor

Before a connection to the Wenglor depth sensor can be established the GigEVision server needs to be started.

* On Windows the `client.py` program tries to do this automatically on startup (if it can find the executable anywhere within the current folder, or you supply the path via `--wenglor-config`)
* To do this manually you need to run `ShapeDriveGigEInterface.exe` like for example:
`ShapeDriveGigEInterface.exe -s 192.168.100.3 -i 192.168.100.1 -n 0 -d`
if the sensor has static IP of `192.168.100.1`.

Options are:

```shell
    -i: IP of sensors
    -s: IP of network (ethernet) adapter
    -n: Index of network adapter (run executable without parameters to see a list of network adapters of the system)
    -d: Print debug messages to console
```

### Wenglor-Software to test connection

* `Software_VisionApp_Demo_3D_2.0.0`
* Start --> Doubleclick entry in device list
* If trigger is set to `Software`, we currently cannot read data from out Python scripts. Trigger must be set to `Intern`.

Data is read with `Harvesters` library: <https://github.com/genicam/harvesters>
Needs a "producer file": [mvGenTLProducer.cti](/mvGenTLProducer.cti) (included in this repository)
