# Copyright (c) 2025 DENKweit GmbH <denkweit.com> All rights reserved.
"""Creates camera calibration and position configurations."""

import json

import cv2
import numpy as np
import numpy.typing as npt
import open3d as o3d
from pathlib import Path
from typing import Any

def make_heatmap():
    hm = []
    color = [255, 0, 0]
    hm.append(color.copy())
    for _ in range(255):
        color[1] += 1
        hm.append(color.copy())
    for _ in range(255):
        color[0] -= 1
        hm.append(color.copy())
    for _ in range(255):
        color[2] += 1
        hm.append(color.copy())
    for _ in range(255):
        color[1] -= 1
        hm.append(color.copy())
    return hm

class PointCloudMapper:
    def __init__(self, camera_calibration_data_path: Path, camera_position_data_path: Path) -> None:
        """Init."""
        self._load_calibration_data(camera_calibration_data_path)
        self._load_camera_position(camera_position_data_path)
        self.point_cloud: npt.NDArray[np.float64] | None = None
        self.projected_points: npt.NDArray[Any] | None = None
        self.z_depths: npt.NDArray[Any] | None = None
        self.heatmap = make_heatmap()

    def _load_calibration_data(self, filename: Path) -> None:
        with filename.open(encoding="utf-8") as file:
            data = json.load(file)
        fx = data["fx"]
        fy = data["fy"]
        cx = data["cx"]
        cy = data["cy"]
        self.camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        self.distortion_coefficients = np.zeros(4)  # assume all zeros for now

    def _load_camera_position(self, filename: Path) -> None:
        with filename.open(encoding="utf-8") as file:
            data = json.load(file)
        r_vec = data["rotation_vector"]
        t_vec = data["translation_vector"]
        self.rotation_vector = np.array(
            [
                [r_vec[0]],
                [r_vec[1]],
                [r_vec[2]],
            ],
            dtype=np.float64,
        )
        self.translation_vector = np.array(
            [
                [t_vec[0]],
                [t_vec[1]],
                [t_vec[2]],
            ],
            dtype=np.float64,
        )

    def load_point_cloud(self, point_cloud: npt.NDArray[np.float64]) -> None:
        """Load point cloud and update projections."""
        self.point_cloud = point_cloud
        # Project all 3D points to image plane
        projected_points, _ = cv2.projectPoints(
            point_cloud,
            self.rotation_vector,
            self.translation_vector,
            self.camera_matrix,
            self.distortion_coefficients,
        )
        self.projected_points = projected_points.reshape(-1, 2)
        # Get camera-space coordinates and find closest point
        R, _ = cv2.Rodrigues(self.rotation_vector)
        points_cam = (R @ self.point_cloud.T).T + self.translation_vector.ravel()
        self.z_depths = points_cam[:, 2]

    def get_filter(self, p, z_min, z_max):
        mask = np.random.random(self.point_cloud.shape[0]) < p
        mask &= (self.point_cloud[:, 2] >= z_min) & (self.point_cloud[:, 2] <= z_max)
        return mask

    def render_projected_points(self, canvas: np.ndarray, z_filter=[0, 500], point_size=1, density=1.0) -> np.ndarray:
        filter_mask = self.get_filter(density, z_filter[0], z_filter[1])
        filtered_point_cloud = self.point_cloud[filter_mask]
        filtered_projected_points = self.projected_points[filter_mask]
        min_z_3d = filtered_point_cloud[np.argmin(filtered_point_cloud[:, 2]), 2]
        max_z_3d = filtered_point_cloud[np.argmax(filtered_point_cloud[:, 2]), 2]

        range_z = (max_z_3d - min_z_3d)
        heatmap_max_idx = len(self.heatmap) - 1

        image = canvas.copy()

        x_coords = np.round(filtered_projected_points[:, 0]).astype(int)
        y_coords = np.round(filtered_projected_points[:, 1]).astype(int)
        
        valid_mask = (
            (x_coords >= 0) & (x_coords < image.shape[1]) &
            (y_coords >= 0) & (y_coords < image.shape[0])
        )
        
        valid_x = x_coords[valid_mask]
        valid_y = y_coords[valid_mask]
        valid_points_3d = filtered_point_cloud[valid_mask]

        if range_z == 0:
            colors = np.full((len(valid_x), 3), [255, 0, 0], dtype=np.uint8)
        else:
            relative_z = (valid_points_3d[:, 2] - min_z_3d) / range_z
            color_indices = (relative_z * heatmap_max_idx).astype(int)
            # Convert heatmap to NumPy array for array indexing
            heatmap_array = np.array(self.heatmap, dtype=np.uint8)
            colors = heatmap_array[color_indices]

        for x, y, color in zip(valid_x, valid_y, colors):
            cv2.circle(image, (x, y), point_size, color.tolist(), -1)
        return image

def get_origin():
    # X Y Z
    # R G B
    coord_length = 20.0
    points = np.array([
        [0.0, 0.0, 0.0],
        [coord_length, 0.0, 0.0],
        [0.0, coord_length, 0.0],
        [0.0, 0.0, coord_length],
    ])
    lines = np.array([
        [0, 1],
        [0, 2],
        [0, 3],
    ])
    colors = np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
    ])
    coordinate_system = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines),
    )
    coordinate_system.colors = o3d.utility.Vector3dVector(colors)

    return coordinate_system


def save_camera_position(
    image_points: np.ndarray,
    object_points: np.ndarray,
    calibration_data_path: str,
    camera_position_data_output_path: str,
) -> bool:
    with open(calibration_data_path, encoding="utf-8") as file:
        data = json.load(file)

    fx = data["fx"]
    fy = data["fy"]
    cx = data["cx"]
    cy = data["cy"]
    camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    distortion_coefficients = np.zeros(4)

    success, r_vec, t_vec = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        distortion_coefficients,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )

    if not success:
        print("Could not calculate camera position")
        return False

    if t_vec[2] > 100:
        R, _ = cv2.Rodrigues(r_vec)
        camera_position = (-R.T @ t_vec).flatten()
        print(f"Camera position at {camera_position} seems implausible, retrying with mirrored depth axis")
        new_object_points = object_points
        new_object_points[:, [0]] = -object_points[:, [0]]

        success, r_vec, t_vec = cv2.solvePnP(
            new_object_points,
            image_points,
            camera_matrix,
            distortion_coefficients,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )

        if not success:
            print("Could not calculate camera position")
            return False

    r_vec, t_vec = cv2.solvePnPRefineLM(
        object_points, image_points, camera_matrix, distortion_coefficients, r_vec, t_vec
    )

    projected_points, _ = cv2.projectPoints(
        object_points, r_vec, t_vec, camera_matrix, distortion_coefficients
    )

    error = np.linalg.norm(image_points - projected_points.squeeze(), axis=1).mean()
    print(f"Mean reprojection error: {error:.2f} pixels")

    data = {
        "rotation_vector": [r_vec[0][0], r_vec[1][0], r_vec[2][0]],
        "translation_vector": [t_vec[0][0], t_vec[1][0], t_vec[2][0]],
    }
    with open(camera_position_data_output_path, "w", encoding="utf-8") as file:
        json.dump(data, file, indent=2)

    print(f"Saved camera position data to {camera_position_data_output_path}")

    R, _ = cv2.Rodrigues(r_vec)
    camera_position = (-R.T @ t_vec).flatten()
    print(f"Camera position: {camera_position}")

    return True

def save_position_test_image(
    image: np.ndarray,
    point_cloud: np.ndarray,
    calibration_data_path: str,
    camera_position_data_path: str,
    depth_min: float,
    depth_max: float,
    test_image_path: str
):
    pcm = PointCloudMapper(Path(calibration_data_path), Path(camera_position_data_path))
    pcm.load_point_cloud(point_cloud)
    test_image = pcm.render_projected_points(image, [depth_min, depth_max])
    cv2.imwrite(test_image_path, test_image)