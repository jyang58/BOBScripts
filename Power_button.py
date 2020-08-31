# -*- coding: utf-8 -*-
"""
Created on Sun Aug 30 02:11:06 2020

@author: Ryan
"""


from gpiozero import Button
import os
Button(15).wait_for_press()
os.system("sudo poweroff")