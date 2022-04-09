# -*- coding: utf-8 -*-
"""
Created on Wed Jan 12 19:14:33 2022

@author: James Kibii, 296096 EIB
Bachelor Thesis
"""

import numpy as np
from scipy import optimize
import mirror_mode
# import math
import csv
# import cartesian_plot


from parameters import mirror_parameters as mp
# from ground_detection import GroundDetection


# ground_detection = GroundDetection()

def initialize_parameters():
    # Initialize all mirror & surface parameters & make them global
    
    global mirror_normal_right_start, mirror_normal_left_start, surface_normal_start
    global mirror_support_vector_right, mirror_support_vector_left, surface_support_vector 

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
    surface_support_vector = np.array(mp.surface_45deg["surface support vector"])



def read_csv_file():
    global distances, angles
    
    path = 'C:\\Users\\james\\SpyderProjects\\lidar_ground_detection_optimizer\\measurements\\'
    file = 'lidar_measurements_20cycles.csv'
    csvFile = open(path+file)
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


def calculate_rms(optimize_values):
    
    surface_support = np.array([0, optimize_values[0], 0])
    surface_normal = np.array([optimize_values[1], surface_normal_start[1], optimize_values[2]])
    
    mirror_support_right = np.array([mirror_support_vector_right[0], 
                                     mirror_support_vector_right[1], 
                                     mirror_support_vector_right[2]])
    
    mirror_normal_right = np.array([optimize_values[3], 
                                    optimize_values[4], mirror_normal_right_start[2]])
    
    mirror_support_left = np.array([mirror_support_vector_left[0],
                                    mirror_support_vector_left[1],
                                    mirror_support_vector_left[2]])
    
    mirror_normal_left = np.array([optimize_values[5], 
                                   optimize_values[6], mirror_normal_left_start[2]])
    
    
    cartesian_points, mid_cartesian_right, mid_cartesian_left = mirror_mode.start_mirror_mode(mirror_support_right, 
        mirror_normal_right, mirror_support_left, mirror_normal_left, 
        distances, angles, plot_graph = False)
    
    
    distance_points = calculate_vector_distances(surface_support,
                                                 surface_normal, 
                                                 cartesian_points)
    
    # rms = math.sqrt(sum(distance_points ** 2))
    rms = np.sqrt(np.mean(distance_points ** 2))
    print(rms)
    """
    if (optimize_values[3] < 70) or (optimize_values[3]>80) or (optimize_values[6] <-80) or (optimize_values[6]>-70):
        rms = rms + 100
        print(rms)
    """
    return rms




# Start
initialize_parameters() 
read_csv_file()
 


cartesian_points_initial = mirror_mode.start_mirror_mode(mirror_support_vector_right,
       mirror_normal_right_start, mirror_support_vector_left, mirror_normal_left_start, 
       distances, angles, plot_graph=True)


start_values = [surface_support_vector[1], 
                surface_normal_start[0], surface_normal_start[2],
                       
                mirror_normal_right_start[0], mirror_normal_right_start[1],
                
                mirror_normal_left_start[0], mirror_normal_left_start[1]]




if __name__ == "__main__": 
      
    # optimized_values = optimize.fmin(calculate_rms, start_values, ftol=0.01, maxfun=10000)
    optimized_values = optimize.fmin(calculate_rms, start_values)
    
    # Optimized Values
    surface_support_opt = np.array([surface_support_vector[0], optimized_values[0], surface_support_vector[2]])
    surface_normal_opt = np.array([optimized_values[1], surface_normal_start[1], optimized_values[2]])
    
    # mirror_support_right_opt = np.array([optimized_values[3], mirror_support_vector_right[1], mirror_support_vector_right[2]])
    mirror_support_right_opt = np.array(mirror_support_vector_right)
    mirror_normal_right_opt = np.array([optimized_values[3], optimized_values[4], mirror_normal_right_start[2]])
    
    # mirror_support_left_opt = np.array([optimized_values[6], mirror_support_vector_left[1], mirror_support_vector_left[2]])
    mirror_support_left_opt = np.array(mirror_support_vector_left)
    mirror_normal_left_opt = np.array([optimized_values[5], optimized_values[6], mirror_normal_left_start[2]])
    
    
    cartesian_points_opt = mirror_mode.start_mirror_mode(mirror_support_right_opt,
               mirror_normal_right_opt, mirror_support_left_opt, mirror_normal_left_opt, 
               distances, angles, plot_graph=True)
    
    # print(len(distances))
    
    # cartesian_plot.plot_graph(ground_detection.get_x_axis, ground_detection.get_y_axis, ground_detection.get_z_axis)



