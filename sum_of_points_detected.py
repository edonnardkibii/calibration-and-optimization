# -*- coding: utf-8 -*-
"""
Created on Thu Feb 24 11:10:04 2022

@author: James Kibii, 296096 EIB
"""

# import numpy as np
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


def calculate_sum_of_points(RSSI_value):
    detected_value = []
    for i in range(len(RSSI_value)):
        if RSSI_value[i] > 0:
            detected_value.append(RSSI_value)
    
    # Calculate sum of points as a percentage
    sum_of_points = len(detected_value)/len(RSSI_value)

    return sum_of_points

def detected_points():
    RSSI = read_csv_file()
    sum_of_points = calculate_sum_of_points(RSSI)
    # print(sum_of_points)
    return sum_of_points
    
if __name__ == "__main__":
    sum_of_points = detected_points()