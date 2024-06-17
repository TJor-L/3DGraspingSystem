# -*- coding: utf-8 -*-
# File : plane_detection.py
# Time : 6/17/2024 8:40 AM 
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
# plane_detection.py
#
# This file contains the main logic for plane detection using the RANSAC algorithm.
# It iteratively detects planes in a point cloud, checks if they are parallel to the z-axis,
# and stores the indices of inliers for each detected plane.
#
# Functions:
# - detect_planes: Detects planes in a given point cloud, checking for parallelism to the z-axis,
#   and returns the aligned point cloud, detected planes, and inlier indices.

import open3d as o3d
import numpy as np
from utils import is_plane_parallel_to_z, remove_points

def detect_planes(pcd, threshold_angle=5, max_iterations=10):
    original_indices = np.arange(len(pcd.points))
    aligned_pcd = pcd
    detected_planes = []
    all_inliers = []
    inliers_in_plane = []

    current_iterations = 0
    while current_iterations < max_iterations and len(aligned_pcd.points) > 0:
        plane_model, inliers = aligned_pcd.segment_plane(distance_threshold=0.01,
                                                         ransac_n=3,
                                                         num_iterations=1000)
        inliers_original_indices = original_indices[inliers]

        if is_plane_parallel_to_z(plane_model, threshold_angle):
            detected_planes.append((plane_model, inliers_original_indices))
            inliers_in_plane.extend(inliers_original_indices)
        else:
            all_inliers.extend(inliers_original_indices)

        aligned_pcd = remove_points(aligned_pcd, inliers)
        points = np.asarray(aligned_pcd.points)
        original_indices = np.delete(original_indices, inliers)
        current_iterations += 1

    return aligned_pcd, detected_planes, all_inliers, inliers_in_plane
