# -*- coding: utf-8 -*-
"""
Created on Wed Jan 12 19:36:06 2022

@author: James Kibii, 296096 EIB
"""

import numpy as np

class GroundDetection(object):
    def __init__(self):
        self.__angles_right_mirror = np.empty([1,0])
        self.__angles_left_mirror = np.empty([1,0])
        self.__distances_right_mirror = np.empty([1,0])
        self.__distances_left_mirror = np.empty([1,0])
        
        self.__normal_vector_right = np.empty([1,0])
        self.__normal_vector_left = np.empty([1,0])
        self.__support_vector_right = np.empty([1,0])
        self.__support_vector_left = np.empty([1,0])
        
        self.__unit_normal_right = np.empty([1,0])
        self.__unit_normal_left = np.empty([1,0])
        
        self.__x_axis_plot = np.empty([1,0])
        self.__y_axis_plot = np.empty([1,0])
        self.__z_axis_plot = np.empty([1,0])

    def __convert_polar2cartesian(self, distances, angles):
        x1 = distances * np.sin(np.radians(angles))
        y1 = distances * np.cos(np.radians(angles))
        z1 = np.zeros(len(angles))

        cartesian_axis = []
        for i in range(len(angles)):
            cartesian_coord = [x1[i], y1[i], z1[i]]
            cartesian_axis.append(cartesian_coord)

        return np.array(cartesian_axis)

    def __calculate_lambda(self, cartesian_points, normal_vector, support_vector):
        # distances length needs to be converted to vector
        lambda_scalar_list = []
        for i in range(len(cartesian_points)):
            lambda_scalar = np.dot(normal_vector, support_vector)/np.dot(normal_vector, cartesian_points[i])
            lambda_scalar_list.append(lambda_scalar)

        # self.__lambda_scalar_list = lambda_scalar_list
        return lambda_scalar_list

    def __calculate_intersection(self, lambda_scalar_list, cartesian_axis):
        intersection_points = []
        mag_intersection_points = []
        for i in range(len(lambda_scalar_list)):
            # intersection = [x * lambda_scalar_list[i] for x in cartesian_axis[i]]
            intersection = lambda_scalar_list[i] * np.array(cartesian_axis[i])
            intersection_points.append(intersection)
            mag_intersection = np.linalg.norm(intersection)
            # mag_intersection = np.sqrt(np.dot(intersection, intersection))
            # mag_intersection =
            mag_intersection_points.append(mag_intersection)

        # print("Magnitude Intersection: " + str(mag_intersection_points))
        return intersection_points, mag_intersection_points

    def __calculate_unit_reflection(self, mag_distance, distance_vector, unit_normal):
        unit_distance_points = []
        unit_reflection_points = []
        for i in range(len(distance_vector)):
            # unit_distance = [x * 1/mag_distance[i] for x in distance_vector[i]]
            unit_distance = 1 / mag_distance[i] * np.array(distance_vector[i])
            unit_distance_points.append(unit_distance)
        for i in range(len(unit_distance_points)):
            unit_reflection = unit_distance_points[i] - 2 * (
                        np.dot(unit_distance_points[i], unit_normal) * np.array(unit_normal))
            unit_reflection_points.append(unit_reflection)
        return unit_reflection_points
    
    def __calculate_gamma(self, distance, mag_intersection):
        # gamma = distance - mag_intersection
        # gamma = []
        gamma = np.subtract(np.abs(distance), np.abs(mag_intersection))
        # print("Gamma: " +str(gamma))
        return gamma

    def __calculate_reflected_vector(self, gamma, unit_reflection):
        reflected_vector_points = []
        for i in range(len(gamma)):
            reflected_vector = gamma[i] * np.array(unit_reflection[i])
            reflected_vector_points.append(reflected_vector)
        return reflected_vector_points

    def __calculate_ground_coordinates(self, intersection_vector, reflected_vector):
        # ground_coordinate_points = []
        ground_coordinate_points = np.add(intersection_vector, reflected_vector)
        # self.__x2 = ground_coordinate_points[:,0]
        # self.__y2 = ground_coordinate_points[:,1]
        # self.__z2 = ground_coordinate_points[:,2]

        return ground_coordinate_points
    
    def set_normal_vector(self, normal_vector_right, normal_vector_left,
                                support_vector_right, support_vector_left,
                                unit_normal_right, unit_normal_left):
        
        self.__normal_vector_right = normal_vector_right
        self.__normal_vector_left = normal_vector_left
        self.__support_vector_right = support_vector_right
        self.__support_vector_left = support_vector_left
        
        self.__unit_normal_right = unit_normal_right
        self.__unit_normal_left = unit_normal_left

        
    
    def run_ground_detection(self, distances_right, angles_right, 
                        distances_left, angles_left,
                        distances_front, angles_front):
        
        self.__distances_right_mirror = distances_right
        self.__distances_left_mirror = distances_left
        self.__angles_right_mirror = angles_right
        self.__angles_left_mirror = angles_left
        
        
        
        cartesian_axis_right = self.__convert_polar2cartesian(
            self.__distances_right_mirror, self.__angles_right_mirror)    
        cartesian_axis_left = self.__convert_polar2cartesian(
            self.__distances_left_mirror, self.__angles_left_mirror)
        cartesian_axis_front = self.__convert_polar2cartesian(distances_front,
                                                              angles_front)
        
        lambda_scalar_list_right = self.__calculate_lambda(cartesian_axis_right,
                            self.__normal_vector_right, self.__support_vector_right)
        lambda_scalar_list_left = self.__calculate_lambda(cartesian_axis_left,
                            self.__normal_vector_left, self.__support_vector_left)
        
        intersection_points_right, mag_intersection_right = self.__calculate_intersection(
            lambda_scalar_list_right,
            cartesian_axis_right)        
        intersection_points_left, mag_intersection_left = self.__calculate_intersection(
            lambda_scalar_list_left, cartesian_axis_left)
        
        unit_reflection_right = self.__calculate_unit_reflection(self.__distances_right_mirror,
                                                                        cartesian_axis_right,
                                                                        self.__unit_normal_right)
        unit_reflection_left = self.__calculate_unit_reflection(self.__distances_left_mirror,
                                                                       cartesian_axis_left,
                                                                       self.__unit_normal_left)
        
        gamma_right = self.__calculate_gamma(self.__distances_right_mirror, mag_intersection_right)
        gamma_left = self.__calculate_gamma(self.__distances_left_mirror, mag_intersection_left)
        
        reflected_right = self.__calculate_reflected_vector(gamma_right, unit_reflection_right)
        reflected_left = self.__calculate_reflected_vector(gamma_left, unit_reflection_left)
        
        ground_coordinates_right = self.__calculate_ground_coordinates(intersection_points_right,
                                                                              reflected_right)
        
        ground_coordinates_left = self.__calculate_ground_coordinates(intersection_points_left,
                                                                             reflected_left)
        
        # mirror_cartesian_points = np.vstack((ground_coordinates_right, ground_coordinates_left))

        # cartesian_points = np.vstack((mirror_cartesian_points, cartesian_axis_front))
        
        # Plotting values
        x = np.append(ground_coordinates_right[:,0], ground_coordinates_left[:,0])
        self.__x_axis_plot = np.append(x, cartesian_axis_front[:,0])
        
        y = np.append(ground_coordinates_right[:,1], ground_coordinates_left[:,1])
        self.__y_axis_plot = np.append(y, cartesian_axis_front[:,1])
        
        z = np.append(ground_coordinates_right[:,2], ground_coordinates_left[:,2])
        self.__z_axis_plot = np.append(z, cartesian_axis_front[:,2])
        # print(self.__z_axis_plot)
        
        return ground_coordinates_right, ground_coordinates_left, cartesian_axis_front
    
    def calculate_mid_point(self, distances_right_mid, angles_right_mid,
                            distances_left_mid, angles_left_mid):
       
        cartesian_axis_right = self.__convert_polar2cartesian(
            distances_right_mid, angles_right_mid)    
        cartesian_axis_left = self.__convert_polar2cartesian(
            distances_left_mid, angles_left_mid)

        
        lambda_scalar_list_right = self.__calculate_lambda(cartesian_axis_right,
                            self.__normal_vector_right, self.__support_vector_right)
        lambda_scalar_list_left = self.__calculate_lambda(cartesian_axis_left,
                            self.__normal_vector_left, self.__support_vector_left)
        
        intersection_points_right, mag_intersection_right = self.__calculate_intersection(
            lambda_scalar_list_right,
            cartesian_axis_right)        
        intersection_points_left, mag_intersection_left = self.__calculate_intersection(
            lambda_scalar_list_left, cartesian_axis_left)
        
        unit_reflection_right = self.__calculate_unit_reflection(distances_right_mid,
                                                                        cartesian_axis_right,
                                                                        self.__unit_normal_right)
        unit_reflection_left = self.__calculate_unit_reflection(distances_left_mid,
                                                                       cartesian_axis_left,
                                                                       self.__unit_normal_left)
        
        gamma_right = self.__calculate_gamma(distances_right_mid, mag_intersection_right)
        gamma_left = self.__calculate_gamma(distances_left_mid, mag_intersection_left)
        
        reflected_right = self.__calculate_reflected_vector(gamma_right, unit_reflection_right)
        reflected_left = self.__calculate_reflected_vector(gamma_left, unit_reflection_left)
        
        mid_right = self.__calculate_ground_coordinates(intersection_points_right,
                                                                              reflected_right)
        
        mid_left = self.__calculate_ground_coordinates(intersection_points_left,
                                                                             reflected_left)
        
        # cartesian_points_mid = np.vstack((ground_coordinates_right, ground_coordinates_left))

        # cartesian_points = np.vstack((mirror_cartesian_points, cartesian_axis_front))
        
        """
        # Plotting values
        x = np.append(ground_coordinates_right[:,0], ground_coordinates_left[:,0])
        self.__x_axis_plot = np.append(x, cartesian_axis_front[:,0])
        
        y = np.append(ground_coordinates_right[:,1], ground_coordinates_left[:,1])
        self.__y_axis_plot = np.append(y, cartesian_axis_front[:,1])
        
        z = np.append(ground_coordinates_right[:,2], ground_coordinates_left[:,2])
        self.__z_axis_plot = np.append(z, cartesian_axis_front[:,2])
        # print(self.__z_axis_plot)
        """
        return mid_right, mid_left

    @property
    def get_x_axis(self):
        return self.__x_axis_plot

    @property
    def get_y_axis(self):
        return self.__y_axis_plot

    @property
    def get_z_axis(self):
        return self.__z_axis_plot
