import open3d as o3d
import numpy as np
import cv2
import json

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


def save_camera_position(image_points: np.ndarray, object_points: np.ndarray, calibration_data_path: str, camera_position_data_output_path: str) -> bool:
    with open(calibration_data_path, 'r') as file:
        data = json.load(file)
    fx = data["fx"]
    fy = data["fy"]
    cx = data["cx"]
    cy = data["cy"]
    camera_matrix = np.array([
        [fx,  0, cx],
        [ 0, fy, cy],
        [ 0,  0,  1]
    ], dtype=np.float64)
    distortion_coefficients = np.zeros(4)

    success, r_vec, t_vec = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        distortion_coefficients,
        flags=cv2.SOLVEPNP_ITERATIVE
    )

    if not success:
        print("Could not calculate camera position")
        return None

    r_vec, t_vec = cv2.solvePnPRefineLM(
        object_points,
        image_points,
        camera_matrix,
        distortion_coefficients,
        r_vec,
        t_vec
    )

    projected_points, _ = cv2.projectPoints(
        object_points,
        r_vec,
        t_vec,
        camera_matrix,
        distortion_coefficients
    )

    error = np.linalg.norm(image_points - projected_points.squeeze(), axis=1).mean()
    print(f"Mean reprojection error: {error:.2f} pixels")

    data = {
        "rotation_vector": [r_vec[0][0], r_vec[1][0], r_vec[2][0]],
        "translation_vector": [t_vec[0][0], t_vec[1][0], t_vec[2][0]]
    }
    with open(camera_position_data_output_path, 'w') as file:
        json.dump(data, file, indent=2)

    print(f"Saved camera position data to {camera_position_data_output_path}")