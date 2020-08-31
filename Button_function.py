# -*- coding: utf-8 -*-
"""
Created on Sun Aug 30 01:26:49 2020

@author: Ryan
"""
from gpiozero import Button 
import time 
import sys
from free_roam_avoid_obstacles import *
import os
import subprocess
import roslibpy
from nav_complex import main

button = Button(13)
button.when_pressed = Free_Roam()
# ROS scripts should be called 
# alongside free roam function

proc = subprocess.Popen(["./B.O.B. software\rplidar_ros-master\rplidar_ros-master\launch", filename])
proc.wait()
proc = subprocess.Popen(["./B.O.B. software/hector_slam-melodic-devel/hector_slam-melodic-devel/hector_slam_launch/launch/tutorial.launch", filename])
proc.wait()

time.sleep(900)
# Obtain occupancy grid from Hector SLAM
rospy.ServiceProxy(nav_msgs/GetMap.srv, service_class, persistent=False, headers=None)
#Function that starts the sanitation program
main() #run main function nav_complex 

def Stop():
    gpiozeo.cleanup()
    sys.exit()
    
Stop()

button.when_released = Stop()