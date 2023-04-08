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

target_speed = 750
num_readings_to_avg = 5

stop_dist = 5
slow_dist = 25

ultrasonic=Ultrasonic()
PWM= Motor.Motor()

# center ultrasonic sensor
servo = servo.Servo()
servo.setServoPwm('0',90)
servo.setServoPwm('1',90)

# start at zero speed
PWM.setMotorModel(0, 0, 0, 0)



try:
    while True:
      dist = 0
      
      for i in range(num_readings_to_avg):
        dist+=ultrasonic.get_distance()
      
      avg_dist = (dist / num_readings_to_avg)
      print ("Averaged obstacle distance is "+str(avg_dist)+"CM")

      if( avg_dist > slow_dist ):
        PWM.setMotorModel(target_speed, target_speed, target_speed, target_speed)
      elif (avg_dist <= slow_dist) and (avg_dist > stop_dist):
        PWM.setMotorModel(target_speed * 0.5, target_speed * 0.5, target_speed * 0.5, target_speed * 0.5)
      else:
        PWM.setMotorModel(0, 0, 0, 0)

      time.sleep(1)
      
except KeyboardInterrupt:
    print ("\nEnd of program")
