# -*- coding: utf-8 -*-
"""
Created on Wed Jan 12 19:14:33 2022

@author: James Kibii, 296096 EIB
"""

import numpy as np
from ground_detection import mirror_mode
import csv

from parameters import mirror_parameters as mp


def initialize_parameters():
    # Initialize all mirror & surface parameters & make them global
    
    global mirror_normal_right_start, mirror_normal_left_start, surface_normal_start
    global mirror_support_vector_right, mirror_support_vector_left
    global surface_support_vector, surface_support_vector_mid
    global mirror_support_meas_right, mirror_support_meas_left

    mirror_vector_u_right = np.subtract(mp.mirror["mirror vector p2 right"],
                                         mp.mirror["mirror vector p1 right"])
    mirror_vector_v_right = np.subtract(mp.mirror["mirror vector p3 right"],
                                         mp.mirror["mirror vector p1 right"])
    mirror_normal_vector_right = np.cross(mirror_vector_v_right, mirror_vector_u_right)
    mirror_normal_right_start = mirror_normal_vector_right/np.abs(mirror_normal_vector_right[2]) 
    # print(mirror_normal_right_start)


    mirror_vector_u_left = np.subtract(mp.mirror["mirror vector p2 left"],
                                         mp.mirror["mirror vector p1 left"])
    mirror_vector_v_left = np.subtract(mp.mirror["mirror vector p3 left"],
                                         mp.mirror["mirror vector p1 left"])
    mirror_normal_vector_left = np.cross(mirror_vector_u_left, mirror_vector_v_left)
    mirror_normal_left_start = mirror_normal_vector_left/np.abs(mirror_normal_vector_left[2])
    # print(mirror_normal_left_start)


    surface_vector_u = np.subtract(mp.surface_45deg["surface vector p2"],
                                         mp.surface_45deg["surface vector p1"])
    surface_vector_v = np.subtract(mp.surface_45deg["surface vector p3"],
                                         mp.surface_45deg["surface vector p1"])
    surface_normal_vector = np.cross(surface_vector_u, surface_vector_v)
    surface_normal_start = surface_normal_vector/np.abs(surface_normal_vector[1])
    # print(surface_normal_start)



    mirror_support_vector_right = np.array(mp.mirror["support vector right"])
    mirror_support_vector_left = np.array(mp.mirror["support vector left"])
    surface_support_vector = np.array(mp.surface_45deg["surface support vector 0cm"])
    surface_support_vector_mid = np.array(mp.surface_45deg["surface support mid-point 0cm"])
    mirror_support_meas_right = np.array(mp.mirror["support vector right measured"])
    mirror_support_meas_left = np.array(mp.mirror["support vector left measured"])



def read_csv_file():
    global distances, angles
    
    path = 'C:\\Users\\james\\SpyderProjects\\lidar_ground_detection_optimizer\\measurements\\'
    file = 'lidar_measurements_15cycles.csv'
    csvFile = open(path + file)
    csvData = csv.reader(csvFile)
    dataset = list(csvData)
  
    distances = list(map(int, dataset[0]))
    # print(distances)
    angles = list(map(float, dataset[1]))
    # print(angles)
        
    
def calculate_vector_distances(support_vector, normal_vector, 
                              cartesian_points):
    
    distance_points = []
    unit_normal = normal_vector/np.linalg.norm(normal_vector)
    
    for i in range(len(cartesian_points)):
        distance = np.abs((cartesian_points[i] - support_vector).dot(unit_normal))
        distance_points.append(distance)
        
    """
    difference = cartesian_points - support_vector
    
    distances = (difference[:,0]*unit_normal[0]) + (difference[:,1]*unit_normal[1]) + (difference[:,2]*unit_normal[2])
    """ 
    
    return np.array(distance_points)

def calculate_point_distances(support_vector, 
                              cartesian_points):
    
    distance_points = []
    
    for i in range(len(cartesian_points)):
        distance = np.abs((cartesian_points[i] - support_vector))
        distance_points.append(distance)
            
    return np.array(distance_points)

def calculate_mean_std_dev():
    # Start
    initialize_parameters() 
    read_csv_file()
 


    cartesian_points, mid_cartesian_right, mid_cartesian_left = mirror_mode.start_mirror_mode(mirror_support_vector_right,
       mirror_normal_right_start, mirror_support_vector_left, mirror_normal_left_start, 
       distances, angles, plot_graph=True)
    
    distance_points = calculate_vector_distances(surface_support_vector,
                                                 surface_normal_start, 
                                                 cartesian_points)
    
    mid_distance_points_right = calculate_point_distances(surface_support_vector,
                                                 mid_cartesian_right)
    
    mid_distance_points_left = calculate_point_distances(surface_support_vector,
                                                 mid_cartesian_left)
    
    std_dev_all = np.std(distance_points)
    std_dev_mid_right = np.std(mid_distance_points_right)
    std_dev_mid_left = np.std(mid_distance_points_left)
    print(std_dev_all)
    print(std_dev_mid_right)
    print(std_dev_mid_left)
    return std_dev_all, std_dev_mid_right, std_dev_mid_left


if __name__ == "__main__": 
    std_dev = calculate_mean_std_dev()