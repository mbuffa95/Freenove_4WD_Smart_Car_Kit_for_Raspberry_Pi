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

target_dist = 30
kp = 0.1

max_speed = -800
initial_speed = int( max_speed * 0.75 )
num_readings_to_avg = 5

ultrasonic=Ultrasonic()
PWM= Motor()

# center ultrasonic sensor
servo = servo.Servo()
servo.setServoPwm('0',90)
servo.setServoPwm('1',90)

# start at zero speed
PWM.setMotorModel( initial_speed, initial_speed, initial_speed, initial_speed )
curr_speed = initial_speed

try:
    while True:
      dist = 0
      
      for i in range(num_readings_to_avg):
        dist+=ultrasonic.get_distance()
      
      avg_dist = (dist / num_readings_to_avg)
      print("Averaged obstacle distance: " + str(avg_dist) + "CM" )

      err = avg_dist - target_dist
      speed_change = ( kp * err )
      print( "Speed change: " + str( speed_change ) )

      if( ( curr_speed + speed_change ) > max_speed ):
        curr_speed = max_speed # cap that speed to the max
      else: 
        curr_speed = ( curr_speed + speed_change )

      print( "Current speed: " + str(curr_speed) )
      PWM.setMotorModel( curr_speed, curr_speed, curr_speed, curr_speed )

      time.sleep(1)
      
except KeyboardInterrupt:
    PWM.setMotorModel(0, 0, 0, 0)
    print ("\nEnd of program")
