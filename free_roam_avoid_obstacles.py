# -*- coding: utf-8 -*-
"""
Created on Sat Aug 29 22:54:52 2020

@author: Ryan
"""

from gpiozero import DistanceSensor as ds
from gpiozero import Motor
import time 
senFor = ds(31,32, max_distance = 1, threshold_distance = .15)
senLeft= ds(33,34,max_distance = 1, threshold_distance = .15)
senRight= ds(35,36,max_distance = 1, threshold_distance = .15)
sen4Back= ds(37,38,max_distance = 1, threshold_distance = .15)
motor1 = Motor(17,18)#left
motor2 = Motor(19,20)#right



def OnRoadAgain:
    motor1.forward(.2)
    motor2.forward(.2)



def forwardOBPlan():
     motor1.forward(0)
     motor2.forward(0)
     time.sleep(1)
     motor1.backward(.1)
     motor2.backward(.1)
     sendFor.wait_for_out_of_range(20)
     motor1.forward(0)
     motor2.forward(.4)
     time.sleep(3)



def LeftOBPlan():
    motor1.forward(0)
    motor2.forward(0)
    time.sleep(1)
    motor1.backward(.1)
    motor2.backward(.1)
    time.sleep(1.5)
    motor1.forward(0)
    motor2.forward(.4)
    sendLeft.wait_for_out_of_range(20)
    


def RightOBPlan():
    motor1.forward(0)
    motor2.forward(0)
    time.sleep(1)
    motor1.backward(.1)
    motor2.backward(.1)
    time.sleep(1.5)
    motor1.forward(.4)
    motor2.forward(0)
    senRight.wait_for_out_of_range(20)
    


def BackOBPlan():
    motor1.forward(0)
    motor2.forward(0)
    time.sleep(1)
    motor1.forward(.1)
    motor2.forward(.1)
    time.sleep(1.5)
    motor1.forward(.4)
    motor2.forward(0)
    senBack.wait_for_out_of_range(20)


def Obstacle_avoid_pp():
    senBack.when_in_range = BackOBPlan()
    senRight.when_in_range = RightOBPlan()
    senLeft.when_in_range = LeftOBPlan()
    senFor.when_in_range= forwardOBPlan()
    

def forwardOBPlan1
     motor1.forward(0)
     motor2.forward(0)
     time.sleep(1)
     motor1.backward(.1)
     motor2.backward(.1)
     sendFor.wait_for_out_of_range(20)
     motor1.forward(0)
     motor2.forward(.4)
     time.sleep(3)
     OnRoadAgain()



def LeftOBPlan1():
    motor1.forward(0)
    motor2.forward(0)
    time.sleep(1)
    motor1.backward(.1)
    motor2.backward(.1)
    time.sleep(1.5)
    motor1.forward(0)
    motor2.forward(.4)
    sendLeft.wait_for_out_of_range(20)
    OnRoadAgain()
    


def RightOBPlan1():
    motor1.forward(0)
    motor2.forward(0)
    time.sleep(1)
    motor1.backward(.1)
    motor2.backward(.1)
    time.sleep(1.5)
    motor1.forward(.4)
    motor2.forward(0)
    senRight.wait_for_out_of_range(20)
    OnRoadAgain()
    


def BackOBPlan1():
    motor1.forward(0)
    motor2.forward(0)
    time.sleep(1)
    motor1.forward(.1)
    motor2.forward(.1)
    time.sleep(1.5)
    motor1.forward(.4)
    motor2.forward(0)
    senBack.wait_for_out_of_range(20)  
    OnRoadAgain()
    
def Free_Roam():
    senBack.when_in_range = BackOBPlan1()
    senRight.when_in_range = RightOBPlan1()
    senLeft.when_in_range = LeftOBPlan1()
    senFor.when_in_range= forwardOBPlan1()
    






    

    
