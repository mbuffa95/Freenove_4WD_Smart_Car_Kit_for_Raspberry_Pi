import cv2
import numpy as np
import matplotlib.image as mpimg
import time
import matplotlib.pyplot as plt
import os
import math
import logging
import servo
import Motor
import Ultrasonic

target_speed = 1000

ultrasonic=Ultrasonic()

dist = ultrasonic.get_distance()

try:
    while True:
        data=ultrasonic.get_distance()   #Get the value
        print ("Obstacle distance is "+str(data)+"CM")
        time.sleep(1)
except KeyboardInterrupt:
    print ("\nEnd of program")