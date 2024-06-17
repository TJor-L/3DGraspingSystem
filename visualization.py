# -*- coding: utf-8 -*-
# File : visualization.py.py
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
# visualization.py
#
# This file contains the logic for visualizing the point cloud and the detected planes.
# It handles the coloring of inliers and the creation of geometry objects for visualization.
#
# Functions:
# - visualize: Visualizes the point cloud along with the detected planes and inliers.

import open3d as o3d
import numpy as np


def visualize(aligned_pcd, pcd, detected_planes, all_inliers, inliers_in_plane):

    all_inliers_cloud = pcd.select_by_index(all_inliers)
    inliers_in_plane_cloud = pcd.select_by_index(inliers_in_plane)
    inliers_in_plane_cloud.paint_uniform_color([1, 0, 0])
    new_coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])

    plane = o3d.geometry.TriangleMesh.create_box(width=5, height=5, depth=0.01)
    plane.translate([-2.5, -2.5, 0])
    plane.paint_uniform_color([0.5, 0.5, 0.5])

    geometries = [all_inliers_cloud,inliers_in_plane_cloud, aligned_pcd, new_coordinate_frame, plane]

    for i, (plane_model, inliers) in enumerate(detected_planes):
        try:
            [a, b, c, d] = plane_model
            detected_plane = o3d.geometry.TriangleMesh.create_box(width=5, height=5, depth=0.01)
            detected_plane.translate([-2.5, -2.5, -d / c])  # 平面平移到相应的高度
            edges = o3d.geometry.LineSet.create_from_triangle_mesh(detected_plane)
            edges.paint_uniform_color([1.0, 1.0, 1.0])  # 白色线框
            geometries.append(edges)

        except Exception as e:
            print(f"Error processing plane {i}: {e}")

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    for geom in geometries:
        vis.add_geometry(geom)
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0, 0, 0])
    opt.mesh_show_back_face = True
    opt.point_size = 2
    opt.line_width = 2.0

    vis.run()
    vis.destroy_window()
