# existing imports
import ctypes
import json
import logging
import os
import struct
import sys
import time
import numpy as np
import cv2
from pathlib import Path
from typing import TYPE_CHECKING, List
if TYPE_CHECKING:
    from collections.abc import Callable

import eval_types
import results_pb2


class PointCloudMapper:
    def __init__(self, camera_calibration_data_path: str, camera_position_data_path: str):
        self.load_calibration_data(camera_calibration_data_path)
        self.load_camera_position(camera_position_data_path)
        self.point_cloud = None
        self.projected_points = None
        self.z_depths = None

    def load_calibration_data(self, filename):
        with open(filename, 'r') as file:
            data = json.load(file)
        fx = data["fx"]
        fy = data["fy"]
        cx = data["cx"]
        cy = data["cy"]
        self.camera_matrix = np.array([
            [fx,  0, cx],
            [ 0, fy, cy],
            [ 0,  0,  1]
        ], dtype=np.float64)
        self.distortion_coefficients = np.zeros(4) # assume all zeros for now

    def load_camera_position(self, filename):
        with open(filename, 'r') as file:
            data = json.load(file)
        r_vec = data["rotation_vector"]
        t_vec = data["translation_vector"]
        self.rotation_vector = np.array([
            [r_vec[0]], [r_vec[1]], [r_vec[2]],
        ], dtype=np.float64)
        self.translation_vector = np.array([
            [t_vec[0]], [t_vec[1]], [t_vec[2]],
        ], dtype=np.float64)

    def load_point_cloud(self, point_cloud: np.ndarray):
        self.point_cloud = point_cloud
        # Project all 3D points to image plane
        projected_points, _ = cv2.projectPoints(point_cloud, self.rotation_vector, self.translation_vector, self.camera_matrix, self.distortion_coefficients)
        self.projected_points = projected_points.reshape(-1, 2)
        # Get camera-space coordinates and find closest point
        R, _ = cv2.Rodrigues(self.rotation_vector)
        points_cam = (R @ self.point_cloud.T).T + self.translation_vector.ravel()
        self.z_depths = points_cam[:, 2]

    def find_3d_point_from_pixel(self, pixel_x: int, pixel_y: int, distance_threshold: float = 5.0):
        if self.point_cloud is None or \
            self.projected_points is None or \
            self.z_depths is None:
            raise Exception("load_point_cloud(...) must be called before find_3d_point_from_pixel(...)")

        # Find nearest points in 2D space
        distances = np.linalg.norm(self.projected_points - [pixel_x, pixel_y], axis=1)
        mask = distances < distance_threshold

        if not np.any(mask):
            return None

        # Return closest 3D point in camera Z-axis
        valid_indices = np.where(mask)[0]
        closest_idx = valid_indices[np.argmin(self.z_depths[valid_indices])]
        closest_point = self.point_cloud[closest_idx]

        return closest_point

class Libdenk:
    # common error values
    DE_NO_ERROR: int = struct.unpack('>i', bytes.fromhex("DE000000"))[0]
    DE_BUFFER_TOO_SMALL: int = struct.unpack('>i', bytes.fromhex("DE000010"))[0]

    def __init__(
        self, token: str | None = None, library_dir: Path | None = None, model_dir: Path = Path("models"), device_id: int = -1,
        camera_calibration_data_path: str = "configs/camera_calibration.json", camera_position_data_path: str = "configs/camera_calibration.json"
    ) -> None:
        self.console: Callable[[str], None] = lambda _s: None

        self.log = logging.getLogger().getChild("libdenk")

        if library_dir is None:
            library_dir = Path.cwd()

        # Initialize networks
        # Load the DLL
        if sys.platform == "win32":
            # Add the current working directory to the DLL search path
            os.add_dll_directory(str(library_dir))
            self.libdll = ctypes.cdll.LoadLibrary("denk.dll")
        elif sys.platform == "linux":
            os.environ["LD_LIBRARY_PATH"] = str(library_dir) # TODO: Test if loading libdenk.so works
            self.libdll = ctypes.cdll.LoadLibrary("libdenk.so")

        self.mapper = PointCloudMapper(camera_calibration_data_path, camera_position_data_path)

        self.libdll.BuildInfo()

        if token is None:
            retval = self.libdll.FindDongle()
            self.print_formatted_return("FindDongle", retval)
        else:
            retval = self.libdll.TokenLogin(self.p_str_to_c_str(token), b'\x00')
            self.print_formatted_return("TokenLogin", retval)

        # Allocate a buffer for the model information
        modelinfo_size = ctypes.c_int(10000)
        modelinfo = b'\x00' * modelinfo_size.value

        # Read all model files in the model_dir directory, write the model info into "buffer" (will be ignored in this example), select the CPU (-1) as the evaluation device
        retval = self.libdll.ReadAllModels(str(model_dir).encode("utf-8"), modelinfo, ctypes.byref(modelinfo_size), device_id)
        if retval == self.DE_BUFFER_TOO_SMALL:
            modelinfo = b'\x00' * modelinfo_size.value
            retval = self.libdll.ReadAllModels(str(model_dir).encode("utf-8"), modelinfo, ctypes.byref(modelinfo_size), device_id)
        self.print_formatted_return("ReadAllModels", retval)

        # Get the default JSON
        buffer1_size = ctypes.c_int(1000000)
        buffer1 = b'\x00' * buffer1_size.value
        retval = self.libdll.GetDefaultJson(buffer1, ctypes.byref(buffer1_size))
        if retval == self.DE_BUFFER_TOO_SMALL:
            buffer1 = b'\x00' * buffer1_size.value
            retval = self.libdll.GetDefaultJson(buffer1, ctypes.byref(buffer1_size))
        self.print_formatted_return("GetDefaultJson", retval)

        default_json = json.loads(buffer1[: buffer1_size.value].decode("utf-8"))

        with Path("networkconfig_default.json").open("w", encoding="utf-8") as file:
            json.dump(default_json, file, indent=2)

        # Add entries for the loaded networks
        buffer2_size = ctypes.c_int(1000000)
        buffer2 = b'\x00' * buffer2_size.value
        retval = self.libdll.CreateJsonEntries(
            buffer1, buffer1_size.value, buffer2, ctypes.byref(buffer2_size)
        )
        if retval == self.DE_BUFFER_TOO_SMALL:
            buffer2 = b'\x00' * buffer2_size.value
            retval = self.libdll.CreateJsonEntries(
                buffer1, buffer1_size.value, buffer2, ctypes.byref(buffer2_size)
            )
        self.print_formatted_return("CreateJsonEntries", retval)

        default_json_with_models = json.loads(buffer2[: buffer2_size.value].decode("utf-8"))

        with Path("networkconfig_default_with_models.json").open("w", encoding="utf-8") as file:
            json.dump(default_json_with_models, file, indent=2)

    def evaluate_image_2d(
        self,
        image_file: str | Path,
        eval_parameters: eval_types.EvalParameters,
    ) -> eval_types.EvalResult2D:
        self.console("Evaluating image...")
        self.log.info("Evaluating image...")

        probability_threshold = eval_parameters["probability_threshold"]
        # Open the image file in the "read bytes" mode and read the data
        img_data = Path(image_file).read_bytes()

        # Allocate the variable and the pointer for the index
        index = ctypes.c_int(0)

        # Load the image data
        retval = self.libdll.LoadImageData(ctypes.byref(index), img_data, len(img_data))
        self.print_formatted_return("LoadImageData", retval)

        # Evaluate the image
        t1 = time.time()
        retval = self.libdll.EvaluateImage(index)
        t2 = time.time()
        self.print_formatted_return("EvaluateImage", retval, t2 - t1)

        # Get the results of the evaluation
        retval = self.libdll.ProcessFeatures(index)
        self.print_formatted_return("ProcessFeatures", retval)

        # Allocate a buffer for the results of the evaluation
        results_size = ctypes.c_int(1000000)
        results = b'\x00' * results_size.value

        retval = self.libdll.GetResults(index, results, ctypes.byref(results_size))
        if retval == self.DE_BUFFER_TOO_SMALL:
            results = b'\x00' * results_size.value
            retval = self.libdll.GetResults(index, results, ctypes.byref(results_size))
        self.print_formatted_return("GetResults", retval)

        # Parse the results
        results_proto = results_pb2.Results()
        results_proto.ParseFromString(results[:results_size.value])

        # Print some results
        objects: List[eval_types.ImageObject] = []
        for otpt in results_proto.output:
            if otpt.result_field_type != results_pb2.RFT_REGULAR:
                continue
            for ftr in otpt.feature:
                if ftr.probability > probability_threshold:
                    print_str = f"Found {ftr.label} at\tx = {ftr.rect_x}\tand y = {ftr.rect_y}\twith probabilty p = {ftr.probability}"
                    self.log.debug(print_str)
                    self.console(print_str)
                    objects.append({
                        "x1": ftr.rect_x,
                        "y1": ftr.rect_y,
                        "x2": ftr.rect_x + ftr.rect_w,
                        "y2": ftr.rect_y + ftr.rect_h,
                        "confidence": ftr.probability,
                        "t": eval_types.ObjectType.Unknown
                    })

        # To allocate the correct buffer size, the image dimensions will be taken from the original image
        w = ctypes.c_int(0)
        h = ctypes.c_int(0)
        c = ctypes.c_int(0)

        retval = self.libdll.GetOriginalImageDimensions(index, ctypes.byref(w), ctypes.byref(h), ctypes.byref(c))
        self.print_formatted_return("GetOriginalImageDimensions", retval)

        c.value = 3
        image_buffer_size = w.value * h.value * c.value

        # Allocate a buffer for the resulting image data
        image = b'\x00' * image_buffer_size
        image_size = ctypes.c_int(image_buffer_size)

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
            ctypes.byref(image_size),
        )

        self.print_formatted_return("DrawBoxes", retval)
        self.console("Evaluating image done.")
        self.log.info("Evaluating image done.")

        img_array = np.frombuffer(image, dtype=np.uint8, count=(w.value * h.value * c.value), offset=0)
        img_array.shape = (h.value, w.value, c.value)  # h, w, c

        return img_array, objects

    def evaluate_image_3d(
        self,
        image_file: str | Path,
        eval_parameters: eval_types.EvalParameters,
        point_cloud: np.ndarray,
        distance_threshold: float = 5.0
    ) -> eval_types.EvalResult3D:
        img_array, objects = self.evaluate_image_2d(image_file, eval_parameters)

        self.mapper.load_point_cloud(point_cloud)

        analysis_results: List[eval_types.AnalysisResult] = []

        for obj in objects:
            middle_x = (obj["x1"] + obj["x2"]) / 2
            middle_y = (obj["y1"] + obj["y2"]) / 2

            point_in_space = self.mapper.find_3d_point_from_pixel(middle_x, middle_y, distance_threshold)

            if point_in_space is None:
                self.log.warning(f"No 3D point found for object at x: {middle_x} and y: {middle_y}")
                continue

            analysis_results.append({
                "x": float(point_in_space[0]),
                "y": float(point_in_space[1]),
                "z": float(point_in_space[2]),
                "p": 0.0,
                "r": 0.0,
                "yaw": 0.0,
                "t": obj["t"],
                "c": 100.0 * obj["confidence"]
            })

        return img_array, analysis_results


    @staticmethod
    def c_str_to_p_str(c_str: bytes) -> str:
        pos = c_str.find(b'\x00')
        if pos == -1:
            return c_str.decode("utf-8")
        return c_str[:pos].decode("utf-8")

    @staticmethod
    def p_str_to_c_str(p_str: str) -> bytes:
        return b''.join(( p_str.encode('utf-8'), b'\x00' ))

    def print_formatted_return(
        self, function_name: str, retval: int, t: float | None = None, *, raise_on_error: bool = True
    ) -> bool:
        """Prints the returned integer value as hexadecimal.

        Returns:
            Whether the status is okay.

        Raises:
            RuntimeError: If libdenk status is not ok and raise_on_error is True.
        """

        code = struct.pack(">i", retval).hex().upper()
        ok = retval == self.DE_NO_ERROR
        if t is None:
            self.log.info("%s returned: %s", function_name, code)
        else:
            self.log.info("%s returned: %s ({%s} s)", function_name, code, t)
        if raise_on_error and not ok:
            msg = f"Libdenk function {function_name} returned code {code}!"
            raise RuntimeError(msg)
        return ok