# -*- coding: utf-8 -*-
"""
Created on Sun Aug 23 15:34:37 2020

@author: merli
"""
import numpy as np
import math
import scipy.spatial as spatial
import random


#%%
#define class and appropriate functions for generating all points covered in a range
class nav_coord:
    def __init__(self,occupied_points,total_coordinate_grid,blast_zone,updated_nav_goals,potential_nav_goals,waypoints,all_potential_coord):
        self.occupied_points = occupied_points
        self.total_coordinate_grid = total_coordinate_grid
        self.blast_zone = blast_zone
        self.potential_nav_goals = potential_nav_goals
        self.start_coord = start_coord
        self.waypoints = waypoints
        self.all_potential_coord = all_potential_coord
    def blast(self,occupied_points,total_coordinate_grid,blast_zone,updated_nav_goals,potential_nav_goals,waypoints,all_potential_coord):
        radius = 2
        current_x_point = updated_nav_goals[0]
        current_y_point = updated_nav_goals[1]
        all_potential_coord = []
        blast_zone = []
        for item in total_coordinate_grid:
                if item not in occupied_points:
                    all_potential_coord.append(item)
        for i in range(len(all_potential_coord)):
            potential_coord = all_potential_coord[i]
            pot_x_coord = potential_coord[0]
            pot_y_coord = potential_coord[1]
            distance_formula = math.sqrt( ((current_x_point-pot_x_coord)**2)+((current_y_point-pot_y_coord)**2) )
            if distance_formula <= radius:
                blast_zone.append(potential_coord)
            else:
                continue
        return blast_zone
                
        
    def update_occupied_points(self,occupied_points,total_coordinate_grid,blast_zone,updated_nav_goals,potential_nav_goals,waypoints,all_potential_coord):
        occupied_points += blast_zone
        return occupied_points
    def update_potential_coordinates(self,occupied_points,total_coordinate_grid,blast_zone,updated_nav_goals,potential_nav_goals,waypoints,all_potential_coord):
        set_difference = set(total_coordinate_grid) - set(occupied_points)
        all_potential_coord = list(set_difference)
        return all_potential_coord
    def potential_nav(self,occupied_points,total_coordinate_grid,blast_zone,updated_nav_goals,potential_nav_goals,waypoints,all_potential_coord):
        radius = 3.5
        current_x_point = updated_nav_goals[0]
        current_y_point = updated_nav_goals[1]
        potential_nav_goals= []
        for i in range(len(all_potential_coord)):
            potential_coord = all_potential_coord[i]
            distance_formula = 0
            pot_x_coord = potential_coord[0]
            pot_y_coord = potential_coord[1]
            distance_formula = math.sqrt( ((current_x_point-pot_x_coord)**2)+((current_y_point-pot_y_coord)**2) )
            if distance_formula == radius:
                potential_nav_goals += [(potential_coord)]
                continue
            else:
                continue
        try:
            potential_nav_goals[0]
        except IndexError:
            potential_nav_goals = [random.choice(all_potential_coord)]
                #this means there are no available points in radius 3.5
        
        return potential_nav_goals
    def choose_nav_goal(self,occupied_points,total_coordinate_grid,blast_zone,updated_nav_goals,potential_nav_goals,waypoints,all_potential_coord):
        updated_nav_goals = potential_nav_goals[0]
        return updated_nav_goals
    def update_waypoints(self,occupied_points,total_coordinate_grid,blast_zone,updated_nav_goals,potential_nav_goals,waypoints,all_potential_coord):
        waypoints += [(updated_nav_goals)]
        return waypoints
#%%
#define variables
start_coord = (5,5)
obstacle_point_list = [start_coord]
occupied_points = [start_coord]
all_potential_coord = []
for x in range (3,5,2):
    for y in range (0,4,3):
        obstacle_point_list.append((x,y))

vector_with_points_and_increments = np.arange(0,6,0.1)
total_coordinate_grid= []
for i in range(len(vector_with_points_and_increments)):
        num_one  = vector_with_points_and_increments[i]
        for t in range(len(vector_with_points_and_increments)):
            num_two = vector_with_points_and_increments[t]
            order_pair = (num_one,num_two)
            total_coordinate_grid += [order_pair]
for i in range(len(obstacle_point_list)):
    occupied_point = obstacle_point_list[i]
    occupied_points += [occupied_point]
#occupied_points = total_coordinate_grid
set_difference = set(total_coordinate_grid) - set(occupied_points)
all_potential_coord = list(set_difference)


blast_zone = []
nav_goals = []
updated_nav_goals = start_coord
#point_tree = spatial.cKDTree(total_coordinate_grid)
potential_nav_goals = []
waypoints = [start_coord]
#navigation complex is the brain of it, perhaps our "algorithim"
navigation_complex = nav_coord(occupied_points,total_coordinate_grid,blast_zone,updated_nav_goals,potential_nav_goals,waypoints,all_potential_coord)

#%%
while True:
    blast_zone = navigation_complex.blast(occupied_points,total_coordinate_grid,blast_zone,updated_nav_goals,potential_nav_goals,waypoints,all_potential_coord)
    navigation_complex.update_occupied_points(occupied_points,total_coordinate_grid,blast_zone,updated_nav_goals,potential_nav_goals,waypoints,all_potential_coord)
    all_potential_coord = navigation_complex.update_potential_coordinates(occupied_points,total_coordinate_grid,blast_zone,updated_nav_goals,potential_nav_goals,waypoints,all_potential_coord)
    if not all_potential_coord:
        print('Navigation is done')
        break
    else:
        potential_nav_goals = navigation_complex.potential_nav(occupied_points,total_coordinate_grid,blast_zone,updated_nav_goals,potential_nav_goals,waypoints,all_potential_coord)
        updated_nav_goals = navigation_complex.choose_nav_goal(occupied_points,total_coordinate_grid,blast_zone,updated_nav_goals,potential_nav_goals,waypoints,all_potential_coord)
        waypoints = navigation_complex.update_waypoints(occupied_points,total_coordinate_grid,blast_zone,updated_nav_goals,potential_nav_goals,waypoints,all_potential_coord)
        #loop is done because there are no potential_nav_goals globally
#waypoints should be updated with a list of x,y coordinates that we can feed into navigation stack as end points. in each iteration of the path planner
#start with the starting coordinate as start point --> goal is waypoint[1], then after running lamp --> new startpoint is waypoint[1], goalpoint is waypoint[2]
#this continues until goalpoint == last point of waypoints list --> means it has ran entire course.
