# Copyright (c) 2024 DENKweit GmbH <denkweit.com> All rights reserved.
"""DENKweit image analysis for KaDoTe project."""

from __future__ import annotations

import ctypes
import json
import logging
import os
import struct
import sys
import time
import tkinter as tk
import warnings
from argparse import ArgumentDefaultsHelpFormatter, ArgumentParser, Namespace
from datetime import datetime
from enum import IntEnum
from pathlib import Path
from tkinter import ttk
from typing import TYPE_CHECKING, Any, TypeAlias, TypedDict, cast

import ids_peak_ipl.ids_peak_ipl as ids_ipl
import numpy as np
import opcua
import open3d as o3d
from harvesters.core import Harvester
from ids_peak import ids_peak
from matplotlib import pyplot as plt
from PIL import Image

if TYPE_CHECKING:
    from collections.abc import Callable

# ruff: noqa: T201, D101, D102, D103, D107, DOC201

# IDS library has some of these
warnings.filterwarnings("ignore", category=DeprecationWarning)


class ServerParameters(TypedDict, total=True):
    url: str


class CameraParameters(TypedDict, total=True):
    auto_exposure: bool
    exposure_time: int
    image_size: int


class EvalParameters(TypedDict, total=True):
    probability_threshold: float


EvalResult: TypeAlias = tuple[bytes, int, int, int]


class ObjectType(IntEnum):
    """Classification object type."""

    Unknown = -1


class AnalysisResult(TypedDict, total=True):
    """Python representation of results json object.

    See `schemas/result.schema.json.
    """

    x: int
    y: int
    z: int
    r: int
    p: int
    yaw: int
    t: ObjectType
    pr: int


class Gui:
    def __init__(  # noqa: PLR0913, PLR0915, PLR0917
        self,
        connect_to_server: Callable[[ServerParameters], None],
        server_parameters: ServerParameters,
        acquire_image: Callable[[CameraParameters], Path],
        camera_parameters: CameraParameters,
        acquire_point_cloud: Callable[[], object] | None,
        evaluate_image: Callable[[Path, EvalParameters], EvalResult],
        eval_parameters: EvalParameters,
    ) -> None:
        """Initialize GUI."""
        self.connect_to_server = connect_to_server
        self.server_parameters = server_parameters

        self.acquire_image = acquire_image
        self.camera_parameters = camera_parameters
        self.last_image_file: Path | None = None

        self.acquire_point_cloud = acquire_point_cloud if acquire_point_cloud else lambda: None

        self.evaluate_image = evaluate_image
        self.eval_parameters = eval_parameters
        self.point_cloud_data: Any = None
        # Build Gui
        self.root = tk.Tk()
        self.root.title("DENKweit KaDoTe")
        self.root.geometry("1000x700")

        self.tab_control = ttk.Notebook(self.root)
        self.server_tab_frame = ttk.Frame(self.tab_control)
        self.tab_control.add(self.server_tab_frame, text="Server", sticky="nsew")
        self.camera_tab_frame = ttk.Frame(self.tab_control)
        self.tab_control.add(self.camera_tab_frame, text="Camera")
        self.wenglor_tab_frame = ttk.Frame(self.tab_control)
        self.tab_control.add(self.wenglor_tab_frame, text="3D")
        self.ai_tab_frame = ttk.Frame(self.tab_control)
        self.tab_control.add(self.ai_tab_frame, text="AI")
        # camera tab stuff
        self.camera_tab_frame.rowconfigure(3, weight=1)
        self.camera_tab_frame.columnconfigure(2, weight=1)
        self.acquire_ids_image_button = tk.Button(
            self.camera_tab_frame,
            command=self.acquire_and_display,
            text="Acquire camera image",
        )
        self.acquire_ids_image_button.grid(row=0, column=0, columnspan=5)
        self.exposure_label = tk.Label(self.camera_tab_frame, text="ExposureMode:")
        self.exposure_label.grid(row=1, column=0, columnspan=2)
        self.checkbox_auto_exposure_var = tk.BooleanVar()
        self.checkbox_auto_exposure_var.set(camera_parameters["auto_exposure"])
        self.checkbox_auto_exposure = tk.Checkbutton(
            self.camera_tab_frame,
            text="AutoExposure",
            command=self.update_camera_parameters,
            variable=self.checkbox_auto_exposure_var,
        )
        self.checkbox_auto_exposure.grid(row=1, column=2, columnspan=1)
        self.entry_exposure_time = tk.Entry(
            self.camera_tab_frame,
            width=20,
            validate="key",
            validatecommand=(self.root.register(self._validate_int), "%d", "%P"),
        )
        self.entry_exposure_time.insert(0, str(camera_parameters["exposure_time"]))
        self.entry_exposure_time.grid(row=1, column=3, columnspan=1)
        self.button_set_autoexposure = tk.Button(
            self.camera_tab_frame, text="Set", command=self.update_camera_parameters
        )
        self.button_set_autoexposure.grid(row=1, column=4, columnspan=1)

        self.image_size_label = tk.Label(self.camera_tab_frame, text="Image size [%]:")
        self.image_size_label.grid(row=2, column=0, columnspan=2)
        self.entry_image_size_percent = tk.Entry(
            self.camera_tab_frame,
            width=20,
            validate="key",
            validatecommand=(self.root.register(self._validate_percent), "%d", "%P"),
        )
        self.entry_image_size_percent.insert(0, str(camera_parameters["image_size"]))
        self.entry_image_size_percent.grid(row=2, column=2, columnspan=1)
        self.entry_image_size_percent.bind("<Return>", self.update_camera_parameters)
        self.textbox_camera_tab = tk.Text(self.camera_tab_frame, height=10, width=90)
        self.textbox_camera_tab.grid(
            sticky=tk.E + tk.N + tk.W + tk.S, row=3, column=0, columnspan=5
        )
        self.textbox_camera_tab.config(state=tk.DISABLED)

        # Ai Tab stuff
        self.ai_tab_frame.rowconfigure(2, weight=1)
        self.ai_tab_frame.columnconfigure(2, weight=1)
        self.evaluate_image_button = tk.Button(
            self.ai_tab_frame,
            command=self.eval_and_display,
            text="Evaluate image",
        )
        self.evaluate_image_button.grid(row=0, column=0, columnspan=5)

        self.detection_threshold_label = tk.Label(self.ai_tab_frame, text="Min probability [%]:")
        self.detection_threshold_label.grid(row=1, column=0, columnspan=2)
        self.entry_detection_threshold = tk.Entry(
            self.ai_tab_frame,
            width=20,
            validate="key",
            validatecommand=(self.root.register(self._validate_percent), "%d", "%P"),
        )
        self.entry_detection_threshold.insert(0, "100")
        self.entry_detection_threshold.grid(row=1, column=2, columnspan=1)
        self.entry_detection_threshold.bind("<Return>", self.update_eval_parameters)

        self.textbox_ai_tab = tk.Text(self.ai_tab_frame, height=13, width=90)
        self.textbox_ai_tab.grid(sticky=tk.E + tk.N + tk.W + tk.S, row=2, column=0, columnspan=5)
        self.textbox_ai_tab.config(state=tk.DISABLED)

        # Wenglor tab stuff
        self.wenglor_tab_frame.rowconfigure(3, weight=1)
        for col in range(4):
            self.wenglor_tab_frame.columnconfigure(col, weight=1)
        self.acquire_pointcloud_button = tk.Button(
            self.wenglor_tab_frame, command=self.acquire_point_cloud, text="Acquire point cloud"
        )
        self.acquire_pointcloud_button.grid(
            sticky=tk.E + tk.N + tk.W + tk.S, row=0, column=0, columnspan=1
        )
        self.show_last_intensity_map_button = tk.Button(
            self.wenglor_tab_frame,
            command=self.show_last_intensity_map,
            text="Show last intensity map",
        )
        self.show_last_intensity_map_button.grid(
            sticky=tk.E + tk.N + tk.W + tk.S, row=0, column=1, columnspan=1
        )
        self.show_last_height_map_button = tk.Button(
            self.wenglor_tab_frame, command=self.show_last_height_map, text="Show last height map"
        )
        self.show_last_height_map_button.grid(
            sticky=tk.E + tk.N + tk.W + tk.S, row=0, column=2, columnspan=1
        )
        self.show_last_point_cloud_button = tk.Button(
            self.wenglor_tab_frame, command=self.show_last_point_cloud, text="Show last point cloud"
        )
        self.show_last_point_cloud_button.grid(
            sticky=tk.E + tk.N + tk.W + tk.S, row=0, column=3, columnspan=1
        )

        self.textbox_wenglor_tab = tk.Text(self.wenglor_tab_frame, height=10, width=90)
        self.textbox_wenglor_tab.grid(
            sticky=tk.E + tk.N + tk.W + tk.S, row=3, column=0, columnspan=4
        )
        self.textbox_wenglor_tab.config(state=tk.DISABLED)
        # Server tab stuff
        self.server_tab_frame.rowconfigure(1, weight=1)
        self.server_tab_frame.columnconfigure(1, weight=1)
        self.server_label = tk.Label(self.server_tab_frame, text="OPCUA server URL:")
        self.server_label.grid(row=0, column=0, columnspan=1)
        self.entry_server_url = tk.Entry(self.server_tab_frame, width=50)
        self.entry_server_url.insert(0, "opc.tcp://localhost:4840/freeopcua/server/")
        self.entry_server_url.bind("<Return>", self.update_server_parameters)
        self.entry_server_url.grid(sticky=tk.E + tk.N + tk.W + tk.S, row=0, column=1, columnspan=3)
        self.button_connect_to_server = tk.Button(
            self.server_tab_frame,
            text="Connect",
            command=lambda: self.connect_to_server(self.server_parameters),
        )
        self.button_connect_to_server.grid(row=0, column=4, columnspan=1)

        self.textbox_server_tab = tk.Text(self.server_tab_frame, height=13, width=90)
        self.textbox_server_tab.grid(
            sticky=tk.E + tk.N + tk.W + tk.S, row=1, column=0, columnspan=5
        )
        self.textbox_server_tab.config(state=tk.DISABLED)

        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)
        self.tab_control.grid(sticky=tk.E + tk.N + tk.W + tk.S, padx=5, pady=5)

    @staticmethod
    def _validate_int(action: str, value_if_allowed: str) -> bool:
        if action != "1":
            return True
        if value_if_allowed:
            try:
                int(value_if_allowed)
            except ValueError:
                return False
            else:
                return True
        else:
            return False

    @staticmethod
    def _validate_percent(action: str, value_if_allowed: str) -> bool:
        if not Gui._validate_int(action, value_if_allowed):
            return False
        if action != "1":
            return True
        return 0 <= int(value_if_allowed) <= 100  # noqa: PLR2004

    def acquire_and_display(self) -> None:
        """Acquire new image and display."""
        try:
            self.last_image_file = self.acquire_image(self.camera_parameters)
        except (ids_peak.Exception, ids_ipl.Exception):
            return
        self.display_image(self.last_image_file)

    def eval_and_display(self) -> None:
        """Evaluate image and display result."""
        if self.last_image_file is None:
            return
        res = self.evaluate_image(self.last_image_file, self.eval_parameters)
        self.save_and_display_image(*res, self.last_image_file)

    def update_camera_textbox(self, new_info: str) -> None:
        self.textbox_camera_tab.config(state=tk.NORMAL)  # Set state to normal to allow editing
        self.textbox_camera_tab.insert(
            tk.END, new_info + "\n"
        )  # Append new information with a newline
        self.textbox_camera_tab.see(tk.END)
        self.textbox_camera_tab.config(
            state=tk.DISABLED
        )  # Set state back to disabled to prevent editing

    def update_ai_textbox(self, new_info: str) -> None:
        self.textbox_ai_tab.config(state=tk.NORMAL)  # Set state to normal to allow editing
        self.textbox_ai_tab.insert(tk.END, new_info + "\n")  # Append new information with a newline
        self.textbox_ai_tab.see(tk.END)
        self.textbox_ai_tab.config(
            state=tk.DISABLED
        )  # Set state back to disabled to prevent editing

    def update_server_textbox(self, new_info: str) -> None:
        self.textbox_server_tab.config(state=tk.NORMAL)  # Set state to normal to allow editing
        self.textbox_server_tab.insert(
            tk.END, new_info + "\n"
        )  # Append new information with a newline
        self.textbox_server_tab.see(tk.END)
        self.textbox_server_tab.config(
            state=tk.DISABLED
        )  # Set state back to disabled to prevent editing

    def update_wenglor_textbox(self, new_info: str) -> None:
        self.textbox_wenglor_tab.config(state=tk.NORMAL)  # Set state to normal to allow editing
        self.textbox_wenglor_tab.insert(
            tk.END, new_info + "\n"
        )  # Append new information with a newline
        self.textbox_wenglor_tab.see(tk.END)
        self.textbox_wenglor_tab.config(
            state=tk.DISABLED
        )  # Set state back to disabled to prevent editing

    def update_camera_parameters(self, _evt: tk.Event[Any] | None = None) -> None:
        self.camera_parameters["auto_exposure"] = self.checkbox_auto_exposure_var.get()
        try:
            var = int(self.entry_exposure_time.get())
        except ValueError:
            self.update_camera_textbox("Enter an INT value for exposure time.")
        else:
            self.camera_parameters["exposure_time"] = var

        try:
            var = int(self.entry_image_size_percent.get())
            if var >= 1 and var <= 100:  # noqa: PLR2004
                self.camera_parameters["image_size"] = var
            else:
                self.update_camera_textbox("Enter a value between 1 and 100 for image size.")
        except ValueError:
            self.update_camera_textbox("Enter a value between 1 and 100 for image size.")

        self.update_camera_textbox("AutoExposure:" + str(self.camera_parameters["auto_exposure"]))
        self.update_camera_textbox("ExposureTime:" + str(self.camera_parameters["exposure_time"]))
        self.update_camera_textbox("ImageSize:" + str(self.camera_parameters["image_size"]))

    def update_server_parameters(self, _evt: tk.Event[Any] | None = None) -> None:
        try:
            var = self.entry_server_url.get()
            self.server_parameters["url"] = var
            self.update_server_textbox(f"Server url set to: {var}")
        except ValueError:
            self.update_server_textbox("Enter a valid server url.")

    def update_eval_parameters(self, _evt: tk.Event[Any] | None = None) -> None:
        try:
            var = int(self.entry_detection_threshold.get())
            if var >= 1 and var <= 100:  # noqa: PLR2004
                self.eval_parameters["probability_threshold"] = var / 100
                self.update_ai_textbox(f"Probability_threshold set to {var}%.")
            else:
                self.update_ai_textbox("Enter a value between 1 and 100.")
        except ValueError:
            self.update_ai_textbox("Enter a value between 1 and 100.")

    @staticmethod
    def display_image(img: Image.Image | Path) -> None:
        """Display the image using matplotlib."""
        plt.figure(figsize=(15, 15))
        if isinstance(img, Path):
            img = Image.open(img)
        plt.imshow(img)
        plt.show()

    @staticmethod
    def save_and_display_image(
        img_buffer: bytes, img_w: int, img_h: int, img_c: int, image_file: str | Path
    ) -> None:
        """Display the output image."""
        array = np.frombuffer(img_buffer, dtype=np.uint8, count=(img_w * img_h * img_c), offset=0)
        array.shape = (img_h, img_w, img_c)  # h, w, c
        img = Image.fromarray(array)
        image_file = Path(image_file)
        name = image_file.stem
        ext = image_file.suffix
        insertion = "_evaluated"
        new_filename = f"{name}{insertion}{ext}"
        img.save(new_filename, "JPEG")
        Gui.display_image(img)

    def show_last_height_map(self) -> None:
        if self.point_cloud_data is None:
            return
        image_component = self.point_cloud_data.payload.components[1]
        _3d = image_component.data.reshape(
            image_component.height,
            image_component.width,
            int(image_component.num_components_per_pixel),
        )
        array = _3d[:, :, 2]

        vmin = np.min(array)
        vmax = np.max(array)

        # Plot the array
        plt.imshow(array, vmin=vmin, vmax=vmax)
        plt.colorbar()  # Optionally add a colorbar
        plt.title("Plot of the array")
        plt.show()
        plt.close()

    def show_last_intensity_map(self) -> None:
        if self.point_cloud_data is None:
            return
        image_component = self.point_cloud_data.payload.components[0]
        _2d = image_component.data.reshape(
            image_component.height,
            image_component.width,
            int(image_component.num_components_per_pixel),
        )
        array = np.squeeze(_2d)
        vmin = np.min(array)
        vmax = np.max(array)
        # Plot the array
        plt.imshow(array, cmap="gray", vmin=vmin, vmax=vmax)
        plt.colorbar()  # Optionally add a colorbar
        plt.title("Plot of the array")
        plt.show()
        plt.close()

    def show_last_point_cloud(self) -> None:
        if self.point_cloud_data is None:
            return
        pointcloud_height_component = self.point_cloud_data.payload.components[1]
        _3d = pointcloud_height_component.data.reshape(
            pointcloud_height_component.height,
            pointcloud_height_component.width,
            int(pointcloud_height_component.num_components_per_pixel),
        )

        x = _3d[:, :, 0].flatten()
        y = _3d[:, :, 1].flatten()
        z = _3d[:, :, 2].flatten()

        pointcloud_plot_data = np.vstack((x, y, z)).T
        z_min = 0.0  # Minimum Z value
        z_max = 500.0  # Maximum Z value
        mask = (pointcloud_plot_data[:, 2] >= z_min) & (pointcloud_plot_data[:, 2] <= z_max)

        # Use the mask to filter the xyz array
        filtered_xyz = pointcloud_plot_data[mask]
        point_cloud_plot = o3d.geometry.PointCloud()
        point_cloud_plot.points = o3d.utility.Vector3dVector(filtered_xyz)
        o3d.visualization.draw_geometries([point_cloud_plot])


class OpcuaClient:
    def __init__(self) -> None:
        self.console: Callable[[str], None] = lambda _s: None
        self.client: opcua.Client | None = None

    def connect_to_server(self, params: ServerParameters) -> None:
        log = logging.getLogger().getChild("opcua_client")
        log.info("Trying to establish connection with server %s", params["url"])
        try:
            self.client = opcua.Client(params["url"])
            self.client.connect()
            self.console("Connected to server.")
        except OSError:
            self.client = None
            log.info("Server connection failed.")
            self.console("Server connection failed.")

    @property
    def is_connected(self) -> bool:
        return self.client is not None

    @property
    def acquire_image_node(self) -> opcua.Node:
        if self.client is None:
            msg = "Client not connected to server"
            raise ValueError(msg)
        return self.client.get_node("ns = 2; i = 2")

    @property
    def acquire_image(self) -> int:
        if self.client is None:
            return 0
        return cast(int, self.acquire_image_node.get_value())

    @acquire_image.setter
    def acquire_image(self, value: int) -> None:
        if self.client is None:
            return
        self.acquire_image_node.set_value(value)

    @property
    def image_acquired_node(self) -> opcua.Node:
        if self.client is None:
            msg = "Client not connected to server"
            raise ValueError(msg)
        return self.client.get_node("ns = 2; i = 3")

    @property
    def image_acquired(self) -> int:
        if self.client is None:
            return 0
        return cast(int, self.image_acquired_node.get_value())

    @image_acquired.setter
    def image_acquired(self, value: int) -> None:
        if self.client is None:
            return
        self.image_acquired_node.set_value(value)

    @property
    def results_node(self) -> opcua.Node:
        if self.client is None:
            msg = "Client not connected to server"
            raise ValueError(msg)
        return self.client.get_node("ns = 2; i = 4")

    @property
    def results(self) -> str:
        if self.client is None:
            return ""
        return cast(str, self.image_acquired_node.get_value())

    @results.setter
    def results(self, value: str) -> None:
        if self.client is None:
            return
        self.results_node.set_value(value)


class Wenglor:
    def __init__(self, producer_file: str | Path = Path("mvGenTLProducer.cti")) -> None:
        """Initialize Wenglor Sensor.

        Raises:
            ValueError: If no sensor is found.
        """
        self.console: Callable[[str], None] = lambda _s: None
        self.log = logging.getLogger().getChild("wenglor")
        # TODO: Start GigE software automatically?
        # script_path = "start_GigE_server.bat"  # noqa: ERA001
        # process = subprocess.Popen(['bash', '-c', script_path], shell = True)  # noqa: ERA001
        h = Harvester()
        h.add_file(str(producer_file))
        h.update()
        self.log.debug("Devices: %s", h.device_info_list)
        self.ia = h.create()
        self.point_cloud_data: None = None

    def acquire_point_cloud(self) -> None:
        self.console("Acquiring point cloud.")
        try:
            self.ia.start()
            self.point_cloud_data = self.ia.fetch(timeout=20)
            self.ia.stop()
            self.console("Acquired point cloud.")
        except:
            self.log.exception("Acquiring point cloud failed.")
            self.console("Acquiring point cloud failed.")

    def __del__(self) -> None:
        if hasattr(self, "ia"):
            self.ia.destroy()


class IdsCamera:
    def __init__(self) -> None:
        """Initialize IDS camera."""
        self.console: Callable[[str], None] = lambda _s: None
        self.log = logging.getLogger().getChild("ids")
        self.last_image_file: Path | None = None
        ids_peak.Library.Initialize()
        device_manager = ids_peak.DeviceManager.Instance()
        device_manager.Update()
        device_descriptors = device_manager.Devices()

        self.log.debug("Found Devices: %s", len(device_descriptors))
        for device_descriptor in device_descriptors:
            self.log.debug(device_descriptor.DisplayName())

        try:
            device = device_descriptors[0].OpenDevice(ids_peak.DeviceAccessType_Exclusive)
            self.log.info("Opened Device: %s", device.DisplayName())
            self.remote_device_nodemap = device.RemoteDevice().NodeMaps()[0]
            self.remote_device_nodemap.FindNode("TriggerSelector").SetCurrentEntry("ExposureStart")
            self.remote_device_nodemap.FindNode("TriggerSource").SetCurrentEntry("Software")
            self.remote_device_nodemap.FindNode("TriggerMode").SetCurrentEntry("On")

            self.datastream = device.DataStreams()[0].OpenDataStream()
            payload_size = self.remote_device_nodemap.FindNode("PayloadSize").Value()
            for _ in range(self.datastream.NumBuffersAnnouncedMinRequired()):
                buffer = self.datastream.AllocAndAnnounceBuffer(payload_size)
                self.datastream.QueueBuffer(buffer)
            self.datastream.StartAcquisition()
        except Exception:
            self.log.warning("No camera detected.")
            raise

    # IDS image acquisition
    def acquire_image(self, params: CameraParameters) -> Path:
        self.console("Acquiring image...")
        self.log.info("Acquiring image...")
        try:
            self.remote_device_nodemap.FindNode("AcquisitionStart").Execute()
            self.remote_device_nodemap.FindNode("AcquisitionStart").WaitUntilDone()
            # Enable Auto Exposure
            exposure_auto_node = self.remote_device_nodemap.FindNode("ExposureAuto")
            if params["auto_exposure"]:
                exposure_auto_node.SetCurrentEntry("Continuous")
                self.log.debug("AutoExposure set to True")
            else:
                exposure_auto_node.SetCurrentEntry("Off")
                self.log.debug("AutoExposure set to False")
                self.remote_device_nodemap.FindNode("ExposureTime").SetValue(
                    params["exposure_time"]
                )
            # Trigger image
            self.remote_device_nodemap.FindNode("TriggerSoftware").Execute()
            buffer = self.datastream.WaitForFinishedBuffer(1000)

            # Convert to RGB
            raw_image = ids_ipl.Image.CreateFromSizeAndBuffer(
                buffer.PixelFormat(),
                buffer.BasePtr(),
                buffer.Size(),
                buffer.Width(),
                buffer.Height(),
            )
            color_image = raw_image.ConvertTo(ids_ipl.PixelFormatName_RGB8)
            self.datastream.QueueBuffer(buffer)

            picture = color_image.get_numpy_3D()
            # Convert numpy array to PIL Image
            pil_image = Image.fromarray(picture)
            # Resize the image
            resize_percent = params["image_size"] / 100
            resized_image = pil_image.resize((
                int(pil_image.width * resize_percent),
                int(pil_image.height * resize_percent),
            ))

            # Save the image as JPEG
            current_time = datetime.now()
            file_name = Path(current_time.strftime("%Y-%m-%d_%H-%M-%S") + ".jpg")
            resized_image.save(str(file_name), "JPEG")
            self.console("Acquired camera image.")
            self.log.info("Acquired camera image.")
        except (ids_peak.Exception, ids_ipl.Exception):
            self.log.exception("Image acquisition failed.")
            self.console("Image acquisition failed.")
            raise
        self.last_image_file = file_name
        return file_name


class MockCamera:
    def __init__(self) -> None:
        self.console: Callable[[str], None] = lambda _s: None
        self.log = logging.getLogger().getChild("ids.mock")
        self.last_image_file: Path = Path("testimage.jpg")

    def acquire_image(self, _params: CameraParameters) -> Path:
        self.log.info("Acquiring dummy image.")
        self.console("Acquiring dummy image.")
        return self.last_image_file


class Libdenk:
    def __init__(
        self, token: str, library_dir: Path | None = None, model_dir: Path = Path("models")
    ) -> None:
        self.console: Callable[[str], None] = lambda _s: None
        if library_dir is None:
            library_dir = Path.cwd()
        # Initialize networks
        # Load the DLL
        if sys.platform == "win32":
            # Add the current working directory to the DLL search path
            os.add_dll_directory(str(library_dir))
            self.libdll = ctypes.cdll.LoadLibrary("denk.dll")
        elif sys.platform == "linux":
            # TODO: Might need to add library_dir to LD_LIBRARY_PATH env var?
            self.libdll = ctypes.cdll.LoadLibrary("libdenk.so")

        self.libdll.BuildInfo()
        retval = self.libdll.TokenLogin(token.encode("utf-8"), b"\x00")
        self.print_formatted_return("TokenLogin", retval)
        # Allocate a buffer for the model information
        modelinfo = b"\x00" * 10000
        modelinfo_size = ctypes.c_int(len(modelinfo))
        modelinfo_size_pnt = ctypes.pointer(modelinfo_size)
        # Read all model files in the model_dir directory, write the model info into "buffer" (will be ignored in this example), select the CPU (-1) as the evaluation device
        retval = self.libdll.ReadAllModels(
            str(model_dir).encode("utf-8"), modelinfo, modelinfo_size_pnt, -1
        )
        self.print_formatted_return("ReadAllModels", retval)

        # Get the default JSON
        buffer1_size = ctypes.c_int(1000000)
        buffer1 = b"\x00" * buffer1_size.value
        retval = self.libdll.GetDefaultJson(buffer1, ctypes.byref(buffer1_size))
        self.print_formatted_return("GetDefaultJson", retval)

        default_json = json.loads(buffer1[: buffer1_size.value].decode("utf-8"))

        with Path("networkconfig_default.json").open("w", encoding="utf-8") as file:
            json.dump(default_json, file, indent=2)

        # Add entries for the loaded networks
        buffer2_size = ctypes.c_int(1000000)
        buffer2 = b"\x00" * buffer2_size.value
        retval = self.libdll.CreateJsonEntries(
            buffer1, buffer1_size.value, buffer2, ctypes.byref(buffer2_size)
        )
        self.print_formatted_return("CreateJsonEntries", retval)

        default_json_with_models = json.loads(buffer2[: buffer2_size.value].decode("utf-8"))

        with Path("networkconfig_default_with_models.json").open("w", encoding="utf-8") as file:
            json.dump(default_json_with_models, file, indent=2)

    @staticmethod
    def print_formatted_return(
        function_name: str, retval: int, t: float | None = None, *, raise_on_error: bool = True
    ) -> bool:
        """Prints the returned integer value as hexadecimal.

        Returns:
            Whether the status is okay.

        Raises:
            RuntimeError: If libdenk status is not ok and raise_on_error is True.
        """
        log = logging.getLogger().getChild("libdenk")
        code = struct.pack(">i", retval).hex().upper()
        if t is None:
            log.info("%s returned: %s", function_name, code)
        else:
            log.info("%s returned: %s ({%s} s)", function_name, code, t)
        ok = code == "DE000000"
        if raise_on_error and not ok:
            msg = f"Libdenk function {function_name} returned code {code}!"
            raise RuntimeError(msg)
        return ok

    @staticmethod
    def c_str_to_p_str(c_str: bytes) -> str:
        pos = c_str.find(b"\x00")
        if pos == -1:
            return c_str.decode("utf-8")
        return c_str[:pos].decode("utf-8")

    def evaluate_image(  # noqa: PLR0914
        self,
        image_file: str | Path,
        eval_parameters: EvalParameters,
    ) -> EvalResult:
        import results_pb2  # type: ignore[import-not-found] # noqa: PLC0415

        log = logging.getLogger().getChild("libdenk")
        self.console("Evaluating image...")
        log.info("Evaluating image...")
        probability_threshold = eval_parameters["probability_threshold"]
        # Open the image file in the "read bytes" mode and read the data
        img_data = Path(image_file).read_bytes()

        # Allocate the variable and the pointer for the index
        index = ctypes.c_int(0)
        index_pnt = ctypes.pointer(index)

        # Load the image data
        retval = self.libdll.LoadImageData(index_pnt, img_data, len(img_data))
        self.print_formatted_return("LoadImageData", retval)

        # Evaluate the image
        t1 = time.time()
        retval = self.libdll.EvaluateImage(index)
        t2 = time.time()
        self.print_formatted_return("EvaluateImage", retval, t2 - t1)

        # Allocate a buffer for the results of the evaluation
        results = b"\x00" * 100000
        results_size = ctypes.c_int(len(results))
        results_size_pnt = ctypes.pointer(results_size)

        # Get the results of the evaluation
        retval = self.libdll.ProcessFeatures(index)
        retval = self.libdll.GetResults(index, results, results_size_pnt)
        self.print_formatted_return("GetResults", retval)

        # Parse the results
        results_proto = results_pb2.Results()
        results_proto.ParseFromString(results[: results_size.value])

        # Print some results
        for otpt in results_proto.output:
            for ftr in otpt.feature:
                if ftr.probability > probability_threshold:
                    log.debug(
                        f"Found {ftr.label} at\tx = {ftr.rect_x}\tand y = {ftr.rect_y}\twith probabilty p = {ftr.probability}"  # noqa: G004
                    )
                    self.console(
                        f"Found {ftr.label} at\tx = {ftr.rect_x}\tand y = {ftr.rect_y}\twith probabilty p = {ftr.probability}"
                    )
        # Classification results
        for otpt in results_proto.output:
            if otpt.classifier > 0:
                print(f"Class {otpt.model_label}:\t{round(100 * otpt.classifier, 2)} %")

        # To allocate the correct buffer size, the image dimensions will be taken from the original image
        w = ctypes.c_int(0)
        h = ctypes.c_int(0)
        c = ctypes.c_int(0)
        w_pnt = ctypes.pointer(w)
        h_pnt = ctypes.pointer(h)
        c_pnt = ctypes.pointer(c)
        retval = self.libdll.GetOriginalImageDimensions(index, w_pnt, h_pnt, c_pnt)
        self.print_formatted_return("GetOriginalImageDimensions", retval)
        c.value = 3
        image_buffer_size = w.value * h.value * c.value
        # Allocate a buffer for the resulting image data
        image = b"\x00" * image_buffer_size
        image_size = ctypes.c_int(image_buffer_size)
        image_size_pnt = ctypes.pointer(image_size)

        # Get the image with drawn in boxes and segmentations
        overlap_threshold = 1.0
        alpha_boxes = 0.5
        alpha_segmentations = 0.5

        retval = self.libdll.DrawBoxes(
            index,
            ctypes.c_double(overlap_threshold),
            ctypes.c_double(alpha_boxes),
            ctypes.c_double(alpha_segmentations),
            image,
            image_size_pnt,
        )
        self.print_formatted_return("DrawBoxes", retval)
        self.console("Evaluating image done.")
        log.info("Evaluating image done.")
        return image, w.value, h.value, c.value


class MockEvaluation:
    def __init__(self) -> None:
        self.console: Callable[[str], None] = lambda _s: None

    def evaluate_image(
        self,
        image_file: str | Path,
        _eval_parameters: EvalParameters,
    ) -> EvalResult:
        log = logging.getLogger().getChild("lidenk.mock")
        self.console("Evaluating image...")
        # Do nothing, return as-is
        with Image.open(image_file) as img:
            ret = img.tobytes(), img.width, img.height, 3
        self.console("Evaluating image done.")
        log.info("Evaluating image done.")
        return ret


class AcquireStages(IntEnum):
    Camera = 1
    PointCloud = 2


def check_acquire_and_evaluate(
    gui: Gui,
    client: OpcuaClient,
    libdenk: Libdenk | MockEvaluation,
    camera: IdsCamera | MockCamera,
    wenglor: Wenglor | None,
) -> None:
    """Poll the OPCUA server and do new analysis if requested."""
    log = logging.getLogger().getChild("opcua")
    if not client.is_connected:
        log.info("Not connected to OPCUA server.")
        return
    log.info("Checking OPCUA server...")
    # Read the value of the variable
    acquire_image = client.acquire_image
    image_acquired = client.image_acquired
    log.debug("'acquireImage' = %s", acquire_image)
    log.debug("'imageAcquired' = %s", image_acquired)
    # Check which stage we need to execute
    match acquire_image:
        case AcquireStages.Camera:
            try:
                camera.acquire_image(gui.camera_parameters)
            except (ids_peak.Exception, ids_ipl.Exception):
                log.exception("Could not acquire camera image:")
        case AcquireStages.PointCloud if wenglor is not None:
            wenglor.acquire_point_cloud()
    # Evaluate the images if we executed the last stage
    if acquire_image == max(AcquireStages):
        if camera.last_image_file is None:
            log.warning("Cannot evaluate, camera image is missing!")
        else:
            # TODO: Do something with the point cloud?
            image, w, h, c = libdenk.evaluate_image(camera.last_image_file, gui.eval_parameters)
            gui.save_and_display_image(image, w, h, c, camera.last_image_file)
            # TODO: Create actual results here
            client.results = json.dumps(
                [AnalysisResult(x=0, y=0, z=0, r=0, p=0, yaw=0, t=ObjectType.Unknown, pr=100)],
                separators=(",", ":"),
            )
    # Signal completion to server
    client.image_acquired = 1


def _setup_logging() -> None:
    log = logging.getLogger()
    stream_h = logging.StreamHandler()
    stream_fmter = logging.Formatter(
        fmt="%(asctime)s.%(msecs)03d %(name)-15s %(levelname)-8s %(message)s",
        style="%",
        datefmt="%H:%M:%S",
    )
    stream_h.setFormatter(stream_fmter)
    log.addHandler(stream_h)
    # Matplotlib and PIL logs are very verbose, set them to warning
    log.getChild("opcua").setLevel(logging.WARNING)
    log.getChild("matplotlib").setLevel(logging.WARNING)
    log.getChild("PIL").setLevel(logging.WARNING)


def _parse_args(args: list[str]) -> Namespace:
    """CLI argument parsing."""
    parser = ArgumentParser(formatter_class=ArgumentDefaultsHelpFormatter)
    parser.add_argument("--debug", action="store_true", help="Verbose (debug) logging")
    parser.add_argument(
        "--token", type=Path, default=Path("token.txt"), help="File containing model token"
    )
    parser.add_argument(
        "--local-only",
        action="store_true",
        help="Disable periodic pulling from OPCUA server, limiting functionality.",
    )
    parser.add_argument(
        "--allow-missing-hardware",
        action="store_true",
        help="Allow the script to continue without any hardware connected for local testing. (Uses testimage.jpg).",
    )
    parser.add_argument(
        "--check-interval",
        type=float,
        default=1.0,
        help="Interval to check OPCUA server for updates (in sec).",
    )
    return parser.parse_args(args)


def main(args_: list[str]) -> None:
    log = logging.getLogger()
    _setup_logging()
    args = _parse_args(args_)
    if args.debug:
        log.setLevel(logging.DEBUG)

    server_parameters = ServerParameters(url="opc.tcp://localhost:4840/freeopcua/server/")
    camera_parameters = CameraParameters(auto_exposure=True, exposure_time=1000, image_size=50)
    eval_parameters = EvalParameters(probability_threshold=0.75)

    client = OpcuaClient()
    wenglor: Wenglor | None = None
    try:
        wenglor = Wenglor()
    except ValueError:
        log.exception("Cannot connect Wenglor device:")
        if not args.allow_missing_hardware:
            raise

    try:
        libdenk: Libdenk | MockEvaluation = Libdenk(token=args.token.read_text().strip())
    except OSError:
        log.exception("Cannot init libdenk:")
        if args.allow_missing_hardware:
            libdenk = MockEvaluation()
        else:
            raise

    try:
        camera: IdsCamera | MockCamera = IdsCamera()
    except (ids_peak.Exception, ids_ipl.Exception):
        log.exception("Cannot connect IDS camera:")
        if args.allow_missing_hardware:
            camera = MockCamera()
        else:
            raise

    gui = Gui(
        client.connect_to_server,
        server_parameters,
        camera.acquire_image,
        camera_parameters,
        wenglor.acquire_point_cloud if wenglor else None,
        libdenk.evaluate_image,
        eval_parameters,
    )
    # Need to wire up the "console" (outputs) after creating the gui
    client.console = gui.update_server_textbox
    camera.console = gui.update_camera_textbox
    if wenglor:
        wenglor.console = gui.update_wenglor_textbox
    libdenk.console = gui.update_ai_textbox
    # Only enable server polling if not disabled
    if not args.local_only:
        client.connect_to_server(server_parameters)

        def mainloop() -> None:
            check_acquire_and_evaluate(gui, client, libdenk, camera, wenglor)
            gui.root.after(int(args.check_interval * 1000), mainloop)

        gui.root.after(int(args.check_interval * 1000), mainloop)

    gui.root.mainloop()


if __name__ == "__main__":
    main(sys.argv[1:])
