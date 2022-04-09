# -*- coding: utf-8 -*-
"""
Created on Thu Feb 24 11:10:04 2022

@author: James Kibii, 296096 EIB
"""

import numpy as np
import csv

def read_csv_file():
    
    path = 'C:\\Users\\james\\SpyderProjects\\lidar_ground_detection_optimizer\\measurements\\'
    file = 'lidar_RSSI_30cm_45deg_20deg.csv'
    csvFile = open(path + file)
    csvData = csv.reader(csvFile)
    dataset = list(csvData)
  
    # angles = list(map(float, dataset[0]))
    # print(distances)
    RSSI = list(map(int, dataset[0]))
    # print(angles)
    return RSSI

def calculate_mean_intensity_value():
    RSSI = read_csv_file()
    mean_RSSI = np.mean(RSSI)
    # print(mean_RSSI)
    return mean_RSSI
   
if __name__ == "__main__":
    mean_RSSI = calculate_mean_intensity_value()
    print(mean_RSSI)
    