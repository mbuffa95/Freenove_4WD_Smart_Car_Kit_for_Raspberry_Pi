import cv2
import numpy as np
import matplotlib.image as mpimg
import time
import matplotlib.pyplot as plt
import os
import math
import logging
import servo
from Motor import *
from Ultrasonic import *

target_speed = -500
slow_speed = int( ( target_speed ) * 0.5 )
num_readings_to_avg = 5

stop_dist = 10
slow_dist = 35

ultrasonic=Ultrasonic()
PWM= Motor()

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
        PWM.setMotorModel(slow_speed, slow_speed, slow_speed, slow_speed)
      else:
        PWM.setMotorModel(0, 0, 0, 0)

      time.sleep(1)
      
except KeyboardInterrupt:
    PWM.setMotorModel(0, 0, 0, 0)
    print ("\nEnd of program")
