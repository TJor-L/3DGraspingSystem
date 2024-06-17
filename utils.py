# -*- coding: utf-8 -*-
# File : utils.py
# Time : 6/16/2024 9:51 PM 
# Author : Dijkstra Liu
# Email : l.tingjun@wustl.edu
# 
# 　　　    /＞ —— フ
# 　　　　　| `_　 _ l
# 　 　　　ノ  ミ＿xノ
# 　　 　 /　　　 　|
# 　　　 /　 ヽ　　ﾉ
# 　 　 │　　|　|　\
# 　／￣|　　 |　|　|
#  | (￣ヽ＿_ヽ_)__)
# 　＼_つ
#
# Description:
# utils.py
#
# This file contains utility functions used throughout the project. These include vector operations,
# rotation matrix calculations, plane alignment functions, and point cloud manipulation functions.
#
# Functions:
# - create_arrow: Creates an arrow mesh for visualization purposes.
# - get_rotation_matrix_from_two_vectors: Computes the rotation matrix to rotate one vector to another.
# - is_plane_parallel_to_z: Checks if a given plane is parallel to the z-axis within a specified angle threshold.
# - remove_points: Removes points from a point cloud based on given indices.
# - align_with_ground: Aligns the point cloud so that the ground plane is parallel to the z=0 plane.

import numpy as np
import open3d as o3d


def create_arrow(vector, origin=[0, 0, 0], color=[1, 0, 0]):
    arrow = o3d.geometry.TriangleMesh.create_arrow(cylinder_radius=0.02,
                                                   cone_radius=0.04,
                                                   cylinder_height=1.2,
                                                   cone_height=0.2)
    rotation_matrix = get_rotation_matrix_from_two_vectors([0, 0, 1], vector)
    arrow.rotate(rotation_matrix, center=(0, 0, 0))
    arrow.translate(origin)
    arrow.paint_uniform_color(color)
    return arrow


def get_rotation_matrix_from_two_vectors(vec1, vec2):
    vec1 = vec1 / np.linalg.norm(vec1)
    vec2 = vec2 / np.linalg.norm(vec2)
    v = np.cross(vec1, vec2)
    s = np.linalg.norm(v)
    c = np.dot(vec1, vec2)
    v_x = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + v_x + np.dot(v_x, v_x) * ((1 - c) / (s ** 2))
    return rotation_matrix


def is_plane_parallel_to_z(plane_model, threshold_angle=5):
    [a, b, c, d] = plane_model
    plane_normal = np.array([a, b, c])
    z_axis = np.array([0, 0, 1])
    cos_theta = np.dot(plane_normal, z_axis) / (np.linalg.norm(plane_normal) * np.linalg.norm(z_axis))
    angle = np.arccos(cos_theta) * 180 / np.pi
    return angle <= threshold_angle


def remove_points(pcd, indices):
    return pcd.select_by_index(indices, invert=True)


def align_with_ground(pcd):
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                             ransac_n=3,
                                             num_iterations=1000)
    [a, b, c, d] = plane_model
    plane_normal = np.array([a, b, c])
    z_axis = np.array([0, 0, 1])
    rotation_matrix = get_rotation_matrix_from_two_vectors(plane_normal, z_axis)
    pcd.rotate(rotation_matrix, center=(0, 0, 0))

    points = np.asarray(pcd.points)
    plane_point = points[inliers[0]]
    translation_vector = np.array([0, 0, -plane_point[2]])
    pcd.translate(translation_vector)

    return pcd, plane_model, inliers
