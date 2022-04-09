# -*- coding: utf-8 -*-
"""
Created on Thu Jan 13 00:32:07 2022

@author: James Kibii, 296096 EIB
"""

import numpy as np
from ground_detection import cartesian_plot
# from parameters import mirror_parameters as mp
from ground_detection import grounddetection


ground_detection = grounddetection.GroundDetection()

# distances_right = np.empty([1,0])
# distances_left = np.empty([1,0])


def distribute_values(distances, angles, start_angle, end_angle):
    target_angles = []
    target_distances = []
    
    for i in range(len(angles)):
        if start_angle <= angles[i] <= end_angle:
            target_angles.append(angles[i])
    
    temp = set(target_angles)
    index_list = [i for i, val in enumerate(angles) if val in temp]
    
    for i in range(len(index_list)):
        target_distances.append(distances[index_list[i]])
    
    return target_distances, target_angles

def remove_mid_points(distances, angles, target_angle):
    shortened_target_angles = []
    shortened_target_distances = []
    
    for i in range(len(angles)):
        if angles[i] != target_angle:
            shortened_target_angles.append(angles[i])
    
    temp = set(shortened_target_angles)
    index_list = [i for i, val in enumerate(angles) if val in temp]
    
    for i in range(len(index_list)):
        shortened_target_distances.append(distances[index_list[i]])
        
    return shortened_target_distances, shortened_target_angles


def find_points(distances, angles, target_angle):
    target_angles = []
    target_distances = []
    
    for i in range(len(angles)):
        if angles[i] == target_angle:
            target_angles.append(angles[i])
    
    temp = set(target_angles)
    index_list = [i for i, val in enumerate(angles) if val in temp]
    
    for i in range(len(index_list)):
        target_distances.append(distances[index_list[i]])
        
    return target_distances, target_angles
    
    
def start_mirror_mode(support_vector_right, normal_vector_right,
                      support_vector_left, normal_vector_left, 
                      distances, angles, plot_graph):
    
    # global distances_right, distances_left
    unit_normal_right = normal_vector_right/np.linalg.norm(normal_vector_right)
    unit_normal_left = normal_vector_left/np.linalg.norm(normal_vector_left)
    
    distances_right_raw, angles_right_raw = distribute_values(
        distances, angles, start_angle=65, end_angle=95)
    distances_left_raw, angles_left_raw = distribute_values(
        distances, angles, start_angle=-95, end_angle=-65)
    
    distances_front, angles_front = distribute_values(
        distances, angles, start_angle=-20, end_angle=20)
    
    # Crossing Angles
    distances_right_mid, angles_right_mid = find_points(distances_right_raw, angles_right_raw,
                                                            target_angle=67)
    distances_right, angles_right = remove_mid_points(distances_right_raw,
                                                      angles_right_raw, target_angle=-66)
    distances_left_mid, angles_left_mid = find_points(distances_left_raw, angles_left_raw,
                                                            target_angle=-66)
    distances_left, angles_left = remove_mid_points(distances_left_raw,
                                                      angles_left_raw, target_angle=67)
    
    
    ground_detection.set_normal_vector(normal_vector_right, normal_vector_left,
                                       support_vector_right, support_vector_left, 
                                       unit_normal_right, unit_normal_left)  
    
    right_coord_points, left_coord_points, front_coord_points = ground_detection.run_ground_detection(
        distances_right, angles_right, distances_left, angles_left, distances_front, angles_front)
    mirror_cartesian_points = np.vstack((right_coord_points, left_coord_points))

    cartesian_points = np.vstack((mirror_cartesian_points, front_coord_points))

    mid_cartesian_right, mid_cartesian_left = ground_detection.calculate_mid_point(
        distances_right_mid, angles_right_mid, distances_left_mid, angles_left_mid)
    
    if plot_graph:
        cartesian_plot.plot_graph(ground_detection.get_x_axis, ground_detection.get_y_axis, ground_detection.get_z_axis)
    
    return cartesian_points, mid_cartesian_right, mid_cartesian_left

def convert_to_cartesian(support_vector_right, normal_vector_right,
                      support_vector_left, normal_vector_left, 
                      distances, angles):
    
    # global distances_right, distances_left
    unit_normal_right = normal_vector_right/np.linalg.norm(normal_vector_right)
    unit_normal_left = normal_vector_left/np.linalg.norm(normal_vector_left)

    
    distances_front, angles_front = find_points(distances, angles, target_angle=5)
    distances_right, angles_right = find_points(distances, angles, target_angle=80)
    distances_left, angles_left = find_points(distances, angles, target_angle=-80)
    
    ground_detection.set_normal_vector(normal_vector_right, normal_vector_left,
                                       support_vector_right, support_vector_left, 
                                       unit_normal_right, unit_normal_left)
    
    right_coord_points, left_coord_points, front_coord_points = ground_detection.run_ground_detection(
        distances_right, angles_right, distances_left, angles_left, distances_front, angles_front)
    
    return right_coord_points, left_coord_points, front_coord_points
   