"""
Implementation of Rotation operator SO(3) without Library
Author : Taeyoung Kim (https://github.com/Taeyoung96)
"""

import argparse
import open3d as o3d
import numpy as np
import os
from PIL import Image, ImageDraw, ImageFont


def exp_map_matrix(w, theta):
    """Exponential map function to create rotation matrix
    from 3x1 vector and angle
    """
    wx = np.array([[0, -w[2], w[1]],
                   [w[2], 0, -w[0]],
                   [-w[1], w[0], 0]])

    R = np.eye(3) + wx * np.sin(theta) + (1 - np.cos(theta)) * (wx @ wx)
    return R


def exp_map_to_quaternion(w, theta):
    """
    Exponential map function to create unit quaternion
    from 3x1 vector and angle.
    """
    qw = np.cos(theta / 2)
    qx, qy, qz = w * np.sin(theta / 2)
    return np.array([qw, qx, qy, qz])


"""
For calculate quaternion operation
"""
def points_to_pure_quaternions(points):
    n_points = points.shape[0]
    quaternions = np.zeros((n_points, 4))
    quaternions[:, 1:] = points
    return quaternions

def quaternions_to_points(quaternions):
    return quaternions[:, 1:]

def conjugate(q):
    q_conj = np.zeros_like(q)
    q_conj[0] = q[0]
    q_conj[1:] = -q[1:]
    return q_conj

def quaternion_product(q1, q2):
    q_product = np.zeros(4)
    q_product[0] = q1[0] * q2[0] - np.dot(q1[1:], q2[1:])
    q_product[1:] = q1[0] * q2[1:] + q2[0] * q1[1:] + np.cross(q1[1:], q2[1:])
    return q_product


def main(args):
    # Default setting
    os.environ["OPEN3D_ML_DEVICE"] = "CPU"
    WINDOW_WIDTH = 1920
    WINDOW_HEIGHT = 1080

    # Load point cloud data from .ply file
    pcd = o3d.io.read_point_cloud(args.input_path)
    pcd_quat = o3d.io.read_point_cloud(args.input_path)
    pcd_origin = o3d.io.read_point_cloud(args.input_path)

    # Get point cloud coordinates as a numpy array
    coords = np.asarray(pcd.points)
    coord_quat = np.asarray(pcd_quat.points)
    coords_origin = np.asarray(pcd_origin.points)

    # Calculate centroid of point cloud coordinates
    centroid = np.mean(coords, axis=0)
    centroid_quat = np.mean(coord_quat, axis=0)
    centroid_origin = np.mean(coords_origin, axis=0)

    # Translate point cloud coordinates to origin
    coords -= centroid
    coord_quat -= centroid_quat
    coords_origin -= centroid_origin

    # Generate a random rotation matrix
    rot_vec = np.random.rand(3)  # random rotation vector
    rot_vec /= np.sum(rot_vec)  # normalize random vector
    angle = np.random.uniform(0, np.pi / 3)  # random angle
    angle_deg = np.rad2deg(angle)  # For print

    # Calculate rotation matrix with exponential
    rot_matrix = exp_map_matrix(rot_vec, angle)  # For rotation matrix
    rotated_coords = np.dot(coords, rot_matrix)  # Apply rotation matrix to point cloud coordinates

    # Calculate quaternion with exponential map
    quatuarnion = exp_map_to_quaternion(rot_vec, angle)
    coord_pure_quat = points_to_pure_quaternions(coord_quat)  # convert point cloud to pure quatuarnion

    # Apply quaternions to point cloud coordinates
    rotated_quaternions = np.zeros_like(coord_pure_quat)
    conjugate_quaternion = conjugate(quatuarnion)
    for i, q in enumerate(coord_pure_quat):
        rotated_quaternions[i] = quaternion_product(quaternion_product(conjugate_quaternion, q), quatuarnion)

    rotated_points = quaternions_to_points(rotated_quaternions)

    # Update point cloud with rotated coordinates
    pcd.points = o3d.utility.Vector3dVector(rotated_coords)
    pcd_quat.points = o3d.utility.Vector3dVector(rotated_points)
    pcd_origin.points = o3d.utility.Vector3dVector(coords_origin)  # Maintain original point cloud

    # Visualize rotation matrix in a separate window
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='Rotate Point cloud with Rotation matrix ', width=WINDOW_WIDTH, height=WINDOW_HEIGHT)

    vis1 = o3d.visualization.Visualizer()
    vis1.create_window(window_name='Rotate Point cloud with Quaternion ', width=WINDOW_WIDTH, height=WINDOW_HEIGHT)

    # Convert make vector to string
    rot_vec_str = np.array2string(np.round(rot_vec, decimals=4), separator=', ')[1:-1]  # Remove brackets
    angle_str = str(np.round(angle_deg, decimals=4))  # Remove brackets
    rot_vec_text = "Rotation vector: " + rot_vec_str
    angle_text = "Rotation angle : " + angle_str + "°"

    # Tricks to do annotation : ref - https://github.com/isl-org/Open3D/issues/2
    img = Image.new('RGB', (WINDOW_WIDTH, WINDOW_HEIGHT), color=(255, 255, 255))
    fnt = ImageFont.truetype('/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf', 32)
    d = ImageDraw.Draw(img)
    d.text((1200, 100), "Red : Input point cloud", font=fnt, fill=(0, 0, 0))
    d.text((1200, 150), "Blue : Output point cloud", font=fnt, fill=(0, 0, 0))
    d.text((1200, 200), rot_vec_text, font=fnt, fill=(0, 0, 0))
    d.text((1200, 250), angle_text, font=fnt, fill=(0, 0, 0))

    img.save('pil_text.png')
    text_img = o3d.io.read_image("./pil_text.png")

    # Set the color of the points
    pcd.paint_uniform_color([0, 0, 1])  # output : blue
    pcd_quat.paint_uniform_color([0, 0, 1])  # output : blue
    pcd_origin.paint_uniform_color([1, 0, 0])  # input : red

    # Add to the visualizer
    vis.add_geometry(pcd_origin)
    vis.add_geometry(pcd)
    vis.add_geometry(text_img)

    vis1.add_geometry(pcd_origin)
    vis1.add_geometry(pcd_quat)
    vis1.add_geometry(text_img)

    # Run visualizer
    vis.run()
    vis1.run()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_path", type=str, help="path to input .ply file")
    args = parser.parse_args()
    main(args)
