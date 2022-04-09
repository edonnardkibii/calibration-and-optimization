# -*- coding: utf-8 -*-
"""
Created on Wed Jan 12 19:14:33 2022

@author: James Kibii, 296096 EIB
"""

import numpy as np
# from scipy import optimize
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
    mirror_support_vector_right = np.array(mp.mirror["support vector right"])
    mirror_support_vector_left = np.array(mp.mirror["support vector left"])
    mirror_support_meas_right = np.array(mp.mirror["support vector right measured"])
    mirror_support_meas_left = np.array(mp.mirror["support vector left measured"])
    
    
    

    surface_vector_u = np.subtract(mp.surface_45degx20deg["surface vector p2"],
                                         mp.surface_45degx20deg["surface vector p1"])
    surface_vector_v = np.subtract(mp.surface_45degx20deg["surface vector p3"],
                                         mp.surface_45degx20deg["surface vector p1"])
    
    surface_support_vector = np.array(mp.surface_45degx20deg["surface support vector 30cm"])
    surface_support_vector_mid = np.array(mp.surface_45degx20deg["surface support mid-point 30cm"])

    
    surface_normal_vector = np.cross(surface_vector_u, surface_vector_v)
    surface_normal_start = surface_normal_vector/np.abs(surface_normal_vector[1])
    # print(surface_normal_start)



def read_csv_file():
    global distances, angles
    
    path = 'C:\\Users\\james\\SpyderProjects\\lidar_ground_detection_optimizer\\measurements\\'
    file = 'lidar_std_dev_30cm_45deg_20deg.csv'
    csvFile = open(path + file)
    csvData = csv.reader(csvFile)
    dataset = list(csvData)

    angles = list(map(float, dataset[0]))    
    distances = list(map(int, dataset[1]))
    # print(distances)

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

def calculate_mean_std_dev():
    # Start
    initialize_parameters() 
    read_csv_file()
     

    right_coord_points, left_coord_points, front_coord_points = mirror_mode.convert_to_cartesian(
            mirror_support_vector_right, mirror_normal_right_start, 
            mirror_support_vector_left, mirror_normal_left_start, 
            distances, angles)
    
    
    distance_points_right = calculate_vector_distances(surface_support_vector,
                                                 surface_normal_start, 
                                                 right_coord_points)
    distance_points_left = calculate_vector_distances(surface_support_vector,
                                                 surface_normal_start, 
                                                 left_coord_points)
    distance_points_front = calculate_vector_distances(surface_support_vector,
                                                 surface_normal_start, 
                                                 front_coord_points)
    
    std_dev_right = np.std(distance_points_right)
    std_dev_left = np.std(distance_points_left)
    std_dev_front = np.std(distance_points_front)
    
    # print(std_dev_right, std_dev_left, std_dev_front)
    return std_dev_right, std_dev_left, std_dev_front



if __name__ == "__main__": 
    std_dev_right, std_dev_left, std_dev_front = calculate_mean_std_dev()
    

    
