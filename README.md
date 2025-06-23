# Projekt KaDoTe

## TODOs

* Adapt result type to transmit multiple result objects via OPCUA array node instead of json array

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
* Extra libraries not included in this repo:
  * DENKweit image analysis library:
    * `denk.dll` (`libdenk.so` on Linux)
    * `onnxruntime.dll`
    * `results_pb2.py`
    * A model file
    * A token to load the model
  * `ShapeDriverGigEInterface.exe` including its dependencies

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
usage: client.py [-h] [--debug] [--logfile LOGFILE] [--token TOKEN] [--allow-missing-hardware] [--setup-mode] [--dont-show-image] [--wenglor-config WENGLOR_CONFIG] [--local-only]
                 [--check-interval CHECK_INTERVAL] [--opcua-config OPCUA_CONFIG] [--opcua-username OPCUA_USERNAME] [--opcua-password OPCUA_PASSWORD] [--save-dir SAVE_DIR]

options:
  -h, --help            show this help message and exit
  --debug               Verbose (debug) logging (default: False)
  --logfile LOGFILE     Set logging output to file (default: None)
  --token TOKEN         File containing model token (default: token.txt)
  --allow-missing-hardware
                        Allow the script to continue without any hardware connected for local testing. (Uses testimage.jpg). (default: False)
  --setup-mode          Enter special mode for testing hardware connections and camera setup. Implies --debug, --allow-missing-harware and --local-only (default: False)
  --dont-show-image     Do not popup the image after evaluation (default: False)
  --wenglor-config WENGLOR_CONFIG
                        Wenglor depth sensor config file (default: None)
  --save-dir SAVE_DIR   Save directory (default ./results)

Image Analysis:
  --camera-calibration CAMERA_CALIBRATION
                        Camera calibration data file. (default: ./configs/camera_calibration.json)
  --camera-position CAMERA_POSITION
                        Camera position data file. (default: ./configs/camera_position.json)

OPCUA:
  --local-only          Disable periodic pulling from OPCUA server, limiting functionality. (default: False)
  --check-interval CHECK_INTERVAL
                        Interval to check OPCUA server for updates (in sec). (default: 1.0)
  --opcua-config OPCUA_CONFIG
                        OPCUA server config file (default: None)
  --opcua-username OPCUA_USERNAME
                        OPCUA server username (default: None)
  --opcua-password OPCUA_PASSWORD
                        OPCUA server password (default: None)
```

The most important options are:

* `--opcua-config <path/to/opcua-config.json>`: See [schema](/schemas/opcua-server-config.schema.json) for format, optionally with `--opcua-password`, example OPCUA configs are found in [configs](/configs)
* `--token <path/to/token.txt>` to be able to use the image analysis

### Local Testing

For local testing the options `--setup-mode` (`--allow-missing-hardware` & `--local-only`) and the OPCUA test server might be useful.

#### OPCUA Local Test Server

Test the client implementation by running a local opcua test server:

```shell
uv run server.py
```

The test server requests an image to be taken every x seconds (or by user interaction (pressing `i`) on Windows).

NOTE: The test server and client currently use normal string type for the result node instead of array of string!

## IDS-Camera

* IDS Camera is controlled by `ids_peak` python library
  * Library needs to be installed: <https://en.ids-imaging.com/download-peak.html>

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

Data is read with `Harvesters` library: <https://github.com/genicam/harvesters>
Needs a "producer file": [mvGenTLProducer.cti](/mvGenTLProducer.cti) (included in this repository)
