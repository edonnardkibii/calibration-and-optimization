# -*- coding: utf-8 -*-
"""
Created on Sat Feb 26 18:21:51 2022

@author: James Kibii, 296096 EIB
"""

import pandas as pd
import csv

import mean_euclidean_error
import mean_intensity_value
import sum_of_points_detected
import standard_deviation


def read_csv_file():
    path = 'C:\\Users\\james\\SpyderProjects\\lidar_ground_detection_optimizer\\parameters\\'
    file = 'optimized_values_30cm_45deg_20deg_2.csv'
    location = path + file
    csvFile = open(location)
    csvData = csv.reader(csvFile)
    dataset = list(csvData)
  
    optimized_values = list(map(float, dataset[0]))
    
    return optimized_values


if __name__ == "__main__":
    # optimize_fun = False 
    print("Run scipy.optimize?")
    key = input("Press 1 to optimize or any other key to skip this process. ")
    
    mean_euclidean_error.initialize_parameters()
    mean_euclidean_error.read_csv_file()
    
    
    if key == '1':  
        print("Starting calibration...")
        # global optimized_values
        
        optimized_values = mean_euclidean_error.start_nelder_mead_downhill_simplex()
        
        path = 'C:\\Users\\james\\SpyderProjects\\lidar_ground_detection_optimizer\\parameters\\'
        file = 'optimized_values_30cm_45deg_20deg_2.csv'
        location = path + file
        with open(location, 'w') as csvfile:
            write = csv.writer(csvfile)
            write.writerow(optimized_values)
        print("Calibration completed...")
    else:
        pass
    
    optimized_values = read_csv_file()
    
    rms_min = mean_euclidean_error.calculate_rms(optimized_values)
    reflector_position = mean_euclidean_error.calculate_reflector_position(optimized_values)
    mean_RSSI = mean_intensity_value.calculate_mean_intensity_value()
    detected_laser_points = sum_of_points_detected.detected_points()
    std_dev_right, std_dev_left, std_dev_front = standard_deviation.calculate_mean_std_dev()
    
    
    ######################################################
    # Surface Free
    print("Run scipy.optimize for new surface?")
    key = input("Press 2 to optimize for new surface or any other key to skip this process. ")
    if key == '2':
        print("Starting calibration...")
        optimized_values_surface_free = mean_euclidean_error.run_optimization_surface_free(optimized_values)
        
        path = 'C:\\Users\\james\\SpyderProjects\\lidar_ground_detection_optimizer\\parameters\\'
        file = 'optimized_values_surface_free_30cm_30deg_0deg_5.csv'
        location = path + file
        with open(location, 'w') as csvfile:
            write = csv.writer(csvfile)
            write.writerow(optimized_values_surface_free)
        
        print("Calibration completed...")
        rms_min_surface_free = mean_euclidean_error.calculate_rms_surface_free(optimized_values_surface_free)
        reflector_position_surface_free = mean_euclidean_error.calculate_reflector_position_surface_free(optimized_values_surface_free)
        mean_RSSI_surface_free = mean_intensity_value.calculate_mean_intensity_value()
        detected_laser_points_surface_free = sum_of_points_detected.detected_points()
        std_dev_right_surface_free, std_dev_left_surface_free, std_dev_front_surface_free = standard_deviation.calculate_mean_std_dev()
        
        # Save CSV File
        dict = {'rms': rms_min_surface_free, 'Mean-RSSI': mean_RSSI_surface_free, 
                'Sum of detected points': detected_laser_points_surface_free, 
                'std dev right': std_dev_right_surface_free, 'std dev left': std_dev_left_surface_free,
                'std dev front': std_dev_front_surface_free, 'reflector position': reflector_position_surface_free}
        df = pd.DataFrame(dict, index=[0])
        
        path = 'C:\\Users\\james\\SpyderProjects\\lidar_ground_detection_optimizer\\results\\'
        file = 'lidar_results_surface_free_30cm_30deg_0deg_5.csv'
        
        df.to_csv(path + file, header=True, index=False)
        
        

    else:
        pass
    
    
    ######################################################
    
    
    # Save CSV File
    dict = {'rms': rms_min, 'Mean-RSSI': mean_RSSI, 
            'Sum of detected points': detected_laser_points, 
            'std dev right': std_dev_right, 'std dev left': std_dev_left,
            'std dev front': std_dev_front, 'reflector position': reflector_position }
    df = pd.DataFrame(dict, index=[0])
    
    path = 'C:\\Users\\james\\SpyderProjects\\lidar_ground_detection_optimizer\\results\\'
    file = 'lidar_results_30cm_45deg_20deg_5.csv'
    
    df.to_csv(path + file, header=True, index=False)
    
    print("Data successfully saved in csv File.")