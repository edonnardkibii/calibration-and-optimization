# -*- coding: utf-8 -*-
"""
Created on Wed Jan 12 19:14:33 2022

@author: James Kibii, 296096 EIB
"""

import numpy as np
from scipy import optimize
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
    file = 'lidar_30cm_45deg_20deg.csv'
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
    
    distances = (difference[:,0]*unit_normal[0]) + 
    (difference[:,1]*unit_normal[1]) + (difference[:,2]*unit_normal[2])
    """ 
    
    return np.array(distance_points)

def calculate_point_distances(support_vector, cartesian_points):
    
    distance_points = []
    
    for i in range(len(cartesian_points)):
        distance = np.abs((cartesian_points[i] - support_vector))
        distance_points.append(distance)
        
    return np.array(distance_points)


def calculate_rms_mirror_fix(optimize_values):
    
    # Free mid point & mirror support fix
    surface_support = np.array([optimize_values[0], optimize_values[1],
                                optimize_values[2]])
    surface_normal = np.array([optimize_values[3], surface_normal_start[1], optimize_values[4]])
    
    mirror_support_right = np.array([mirror_support_meas_right[0], 0, 0])
    mirror_normal_right = np.array([optimize_values[5], 
                                    optimize_values[6], mirror_normal_right_start[2]])
    
    mirror_support_left = np.array([mirror_support_meas_left[0], 0, 0])
    mirror_normal_left = np.array([optimize_values[7], 
                                   optimize_values[8], mirror_normal_left_start[2]])
    
    
    cartesian_points, mid_cartesian_right, mid_cartesian_left = mirror_mode.start_mirror_mode(mirror_support_right, 
        mirror_normal_right, mirror_support_left, mirror_normal_left, 
        distances, angles, plot_graph = False)
    
    
    distance_points = calculate_vector_distances(surface_support,
                                                 surface_normal, 
                                                 cartesian_points)
    
    mid_distance_points_right = calculate_point_distances(surface_support,
                                                 mid_cartesian_right)
    
    mid_distance_points_left = calculate_point_distances(surface_support,
                                                 mid_cartesian_left)
    
    # rms = math.sqrt(sum(distance_points ** 2))
    rms_all = np.sqrt(np.mean(distance_points ** 2))
    rms_mid_right = np.sqrt(np.mean(mid_distance_points_right ** 2))
    rms_mid_left = np.sqrt(np.mean(mid_distance_points_left ** 2))
    
    rms = rms_all + rms_mid_right + rms_mid_left
    
    # print(rms, rms_all, rms_mid_right, rms_mid_left)

    return rms

def calculate_rms(optimize_values):
    surface_support = np.array([optimize_values[0], optimize_values[1],
                                optimize_values[2]])
    surface_normal = np.array([optimize_values[3], surface_normal_start[1], optimize_values[4]])
    
    mirror_support_right = np.array([mirror_support_meas_right[0], 0, 0])
    mirror_normal_right = np.array([optimize_values[5], 
                                    optimize_values[6], mirror_normal_right_start[2]])
    
    mirror_support_left = np.array([mirror_support_meas_left[0], 0, 0])
    mirror_normal_left = np.array([optimize_values[7], 
                                   optimize_values[8], mirror_normal_left_start[2]])
    
    
    cartesian_points, mid_cartesian_right, mid_cartesian_left = mirror_mode.start_mirror_mode(mirror_support_right, 
        mirror_normal_right, mirror_support_left, mirror_normal_left, 
        distances, angles, plot_graph = False)
    
    
    distance_points = calculate_vector_distances(surface_support,
                                                 surface_normal, 
                                                 cartesian_points)
    
    # rms = math.sqrt(sum(distance_points ** 2))
    rms_all = np.sqrt(np.mean(distance_points ** 2))
    return rms_all

def calculate_reflector_position(optimize_values):
    surface_support = np.array([optimize_values[0], optimize_values[1],
                                optimize_values[2]])
    # surface_normal = np.array([optimize_values[3], surface_normal_start[1], optimize_values[4]])
    
    mirror_support_right = np.array([mirror_support_meas_right[0], 0, 0])
    mirror_normal_right = np.array([optimize_values[5], 
                                    optimize_values[6], mirror_normal_right_start[2]])
    
    mirror_support_left = np.array([mirror_support_meas_left[0], 0, 0])
    mirror_normal_left = np.array([optimize_values[7], 
                                   optimize_values[8], mirror_normal_left_start[2]])
    
    
    cartesian_points, mid_cartesian_right, mid_cartesian_left = mirror_mode.start_mirror_mode(
        mirror_support_right, 
        mirror_normal_right, mirror_support_left, mirror_normal_left, 
        distances, angles, plot_graph = False)
    
    mid_distance_points_right = calculate_point_distances(surface_support,
                                                 mid_cartesian_right)
    
    mid_distance_points_left = calculate_point_distances(surface_support,
                                                 mid_cartesian_left)
    
    rms_mid_right = np.sqrt(np.mean(mid_distance_points_right ** 2))
    rms_mid_left = np.sqrt(np.mean(mid_distance_points_left ** 2))
    
    rms = rms_mid_right + rms_mid_left

    return rms

# Surface Free
def run_optimization_surface_free(opt_values_surface_free):
    global optimize_values_surface_free
    
    optimize_values_surface_free = opt_values_surface_free
    
    optimize_values = [optimize_values_surface_free[0], optimize_values_surface_free[1],
                       optimize_values_surface_free[2], optimize_values_surface_free[3],
                       optimize_values_surface_free[4]]
    xopt, fopt, iter, funcalls, warnflag = optimize.fmin(
        run_nelder_mead_downhill_simplex_surface_free, optimize_values,ftol=0.01, 
        maxfun=10000, full_output=True, disp=True)
    
    optimized_values_surface_free = xopt
    
    return optimized_values_surface_free


def run_nelder_mead_downhill_simplex_surface_free(optimize_values):
    # Free mid point & mirror support fix
    surface_support = np.array([optimize_values[0], optimize_values[1],
                                optimize_values[2]])
    surface_normal = np.array([optimize_values[3], surface_normal_start[1], optimize_values[4]])
    
    mirror_support_right = np.array([mirror_support_meas_right[0], 0, 0])
    
    # Fix
    mirror_normal_right = np.array([optimize_values_surface_free[5], 
                                    optimize_values_surface_free[6], mirror_normal_right_start[2]])
    
    mirror_support_left = np.array([mirror_support_meas_left[0], 0, 0])
    mirror_normal_left = np.array([optimize_values_surface_free[7], 
                                   optimize_values_surface_free[8], mirror_normal_left_start[2]])
    
    
    cartesian_points, mid_cartesian_right, mid_cartesian_left = mirror_mode.start_mirror_mode(mirror_support_right, 
        mirror_normal_right, mirror_support_left, mirror_normal_left, 
        distances, angles, plot_graph = False)
    
    
    distance_points = calculate_vector_distances(surface_support,
                                                 surface_normal, 
                                                 cartesian_points)
    
    mid_distance_points_right = calculate_point_distances(surface_support,
                                                 mid_cartesian_right)
    
    mid_distance_points_left = calculate_point_distances(surface_support,
                                                 mid_cartesian_left)
    
    # rms = math.sqrt(sum(distance_points ** 2))
    rms_all = np.sqrt(np.mean(distance_points ** 2))
    rms_mid_right = np.sqrt(np.mean(mid_distance_points_right ** 2))
    rms_mid_left = np.sqrt(np.mean(mid_distance_points_left ** 2))
    
    rms = rms_all + rms_mid_right + rms_mid_left
    
    # print(rms, rms_all, rms_mid_right, rms_mid_left)

    return rms


def calculate_rms_surface_free(optimized_values_surface_free):
    surface_support = np.array([optimized_values_surface_free[0], optimized_values_surface_free[1],
                                optimized_values_surface_free[2]])
    surface_normal = np.array([optimized_values_surface_free[3], surface_normal_start[1], optimized_values_surface_free[4]])
    
    mirror_support_right = np.array([mirror_support_meas_right[0], 0, 0])
    mirror_normal_right = np.array([optimize_values_surface_free[5], 
                                    optimize_values_surface_free[6], mirror_normal_right_start[2]])
    
    mirror_support_left = np.array([mirror_support_meas_left[0], 0, 0])
    mirror_normal_left = np.array([optimize_values_surface_free[7], 
                                   optimize_values_surface_free[8], mirror_normal_left_start[2]])
    
    
    cartesian_points, mid_cartesian_right, mid_cartesian_left = mirror_mode.start_mirror_mode(mirror_support_right, 
        mirror_normal_right, mirror_support_left, mirror_normal_left, 
        distances, angles, plot_graph = False)
    
    
    distance_points = calculate_vector_distances(surface_support,
                                                 surface_normal, 
                                                 cartesian_points)
    
    # rms = math.sqrt(sum(distance_points ** 2))
    rms_all_surface_free = np.sqrt(np.mean(distance_points ** 2))
    return rms_all_surface_free    


def calculate_reflector_position_surface_free(optimized_values_surface_free):
    surface_support = np.array([optimized_values_surface_free[0], optimized_values_surface_free[1],
                                optimized_values_surface_free[2]])
    # surface_normal = np.array([optimize_values[3], surface_normal_start[1], optimize_values[4]])
    
    mirror_support_right = np.array([mirror_support_meas_right[0], 0, 0])
    mirror_normal_right = np.array([optimize_values_surface_free[5], 
                                    optimize_values_surface_free[6], mirror_normal_right_start[2]])
    
    mirror_support_left = np.array([mirror_support_meas_left[0], 0, 0])
    mirror_normal_left = np.array([optimize_values_surface_free[7], 
                                   optimize_values_surface_free[8], mirror_normal_left_start[2]])
    
    
    cartesian_points, mid_cartesian_right, mid_cartesian_left = mirror_mode.start_mirror_mode(mirror_support_right, 
        mirror_normal_right, mirror_support_left, mirror_normal_left, 
        distances, angles, plot_graph = False)
    
    mid_distance_points_right = calculate_point_distances(surface_support,
                                                 mid_cartesian_right)
    
    mid_distance_points_left = calculate_point_distances(surface_support,
                                                 mid_cartesian_left)
    
    rms_mid_right = np.sqrt(np.mean(mid_distance_points_right ** 2))
    rms_mid_left = np.sqrt(np.mean(mid_distance_points_left ** 2))
    
    rms = rms_mid_right + rms_mid_left
 
    # rms = math.sqrt(sum(distance_points ** 2))
    # rms_all = np.sqrt(np.mean(distance_points ** 2))
    return rms

"""
# Start
initialize_parameters() 
read_csv_file()
 


cartesian_points_initial = mirror_mode.start_mirror_mode(mirror_support_vector_right,
       mirror_normal_right_start, mirror_support_vector_left, mirror_normal_left_start, 
       distances, angles, plot_graph=True)

start_values_mirror_fix = [surface_support_vector_mid[0],
                               surface_support_vector_mid[1],surface_support_vector_mid[2], 
                surface_normal_start[0], surface_normal_start[2],
                
                mirror_normal_right_start[0], mirror_normal_right_start[1],
                
                mirror_normal_left_start[0], mirror_normal_left_start[1]]
"""

def start():
    # Start
    # initialize_parameters() 
    # read_csv_file()
     


    cartesian_points_initial = mirror_mode.start_mirror_mode(mirror_support_vector_right,
           mirror_normal_right_start, mirror_support_vector_left, mirror_normal_left_start, 
           distances, angles, plot_graph=True)

    start_values_mirror_fix = [surface_support_vector_mid[0],
                               surface_support_vector_mid[1],surface_support_vector_mid[2], 
                    
                               surface_normal_start[0], surface_normal_start[2],
                    
                               mirror_normal_right_start[0], mirror_normal_right_start[1],
                    
                               mirror_normal_left_start[0], mirror_normal_left_start[1]]
    
    return cartesian_points_initial, start_values_mirror_fix

def start_nelder_mead_downhill_simplex():
    cartesian_points_initial, start_values_mirror_fix = start()
    xopt, fopt, iter, funcalls, warnflag = optimize.fmin(calculate_rms_mirror_fix, start_values_mirror_fix,ftol=0.01, maxfun=10000, full_output=True, disp=True)
    
    # rms_min = fopt
    optimized_values_mirror_fix = xopt
    
    # Optimized Values- Nelder-Mead Downhill Simplex
    # surface_support_opt = np.array([optimized_values_mirror_fix[0], optimized_values_mirror_fix[1], optimized_values_mirror_fix[2]])
    # surface_normal_opt = np.array([optimized_values_mirror_fix[3], surface_normal_start[1], optimized_values_mirror_fix[4]])
    
    mirror_support_right_opt = np.array([mirror_support_meas_right[0], mirror_support_meas_right[1], mirror_support_meas_right[2]])
    mirror_normal_right_opt = np.array([optimized_values_mirror_fix[5], optimized_values_mirror_fix[6], mirror_normal_right_start[2]])
    
    mirror_support_left_opt = np.array([mirror_support_meas_left[0], mirror_support_meas_left[1], mirror_support_meas_left[2]])
    mirror_normal_left_opt = np.array([optimized_values_mirror_fix[7], optimized_values_mirror_fix[8], mirror_normal_left_start[2]])
    
    
    mirror_mode.start_mirror_mode(mirror_support_right_opt,
               mirror_normal_right_opt, mirror_support_left_opt, mirror_normal_left_opt, 
               distances, angles, plot_graph=True)
    
    return optimized_values_mirror_fix



if __name__ == "__main__": 
    initialize_parameters()
    read_csv_file()
    rms_min = start_nelder_mead_downhill_simplex()
    

    
