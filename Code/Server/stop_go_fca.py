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
from Ultrasonic import *

target_speed = 1000
num_readings_to_avg = 5

ultrasonic=Ultrasonic()

try:
    while True:
      dist = 0
      
      for i in range(num_readings_to_avg):
        dist+=ultrasonic.get_distance()
      
      avg_dist = (dist / num_readings_to_avg)
      print ("Averaged obstacle distance is "+str(avg_dist)+"CM")
      time.sleep(1)
      
except KeyboardInterrupt:
    print ("\nEnd of program")
