# -*- coding: utf-8 -*-
"""
Created on Sat Aug 29 21:47:03 2020

@author: Ryan Wing
"""
def UV_Motion:
    from gpiozero import LED
    from gpiozero import MotionSensor as mos
    from time import sleep
    MS1= mos(1)
    MS2 = mos(2)
    MS3 = mos(3)
    MS4= mos(4)
    n = 300
    while n>0 :
        if MS1.motion_detected == False and MS2.motion_detected == False and MS3.motion_detected == False and MS4.motion_detected ==False:
        led.on()
        time.sleep(1)
        n = n-1
    else:
        led.off()
        mos.wait_for_no_motion(90)
        
    
    
