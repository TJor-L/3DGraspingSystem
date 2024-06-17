# -*- coding: utf-8 -*-
# File : main.py
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
# main.py
#
# This is the main script for running the 3D object grasping system. It reads the point cloud data,
# aligns it with the ground, detects planes, and visualizes the results.
#
# Functions:
# - main: The main function that orchestrates the reading, processing, and visualization of point cloud data.


import open3d as o3d
import numpy as np
from utils import align_with_ground
from plane_detection import detect_planes
from visualization import visualize


def main():
    pcd = o3d.io.read_point_cloud("data/stair.ply")

    points = np.asarray(pcd.points)
    points[:, 2] = -points[:, 2]  # TODO: 根据实际情况调整z的正负值
    pcd.points = o3d.utility.Vector3dVector(points)

    pcd, plane_model, inliers = align_with_ground(pcd)

    plane_normal = np.array([plane_model[0], plane_model[1], plane_model[2]])

    aligned_pcd, detected_planes, all_inliers, inliers_in_plane = detect_planes(pcd)

    visualize(aligned_pcd, pcd, detected_planes, all_inliers, inliers_in_plane)

    o3d.io.write_point_cloud("output/final_detected_planes.pcd", aligned_pcd)


if __name__ == "__main__":
    main()
