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
import sys
import argparse

move = False

argParser = argparse.ArgumentParser()
argParser.add_argument("-m", "--move", help="drives motors")

args = argParser.parse_args()
print("args=%s" % args)

print("args.move=%s" % args.move)

if( args.move == "true" ):
    print( "move is true" )
    move = True
else:
    print( "move is false" )

target_dist = 20
kp = 5

max_speed = -1500
min_speed = -450

initial_speed = int( max_speed * 0.5 )
num_readings_to_avg = 1

ultrasonic=Ultrasonic()
PWM = Motor()

# center ultrasonic sensor
servo = servo.Servo()
servo.setServoPwm('0',90)
servo.setServoPwm('1',90)

# start at zero speed
if move:
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
      print( "Distance error: " + str( err ) )
     
      speed_change = ( kp * err )
      print( "Speed change: " + str( speed_change ) )

      if( int( curr_speed - speed_change ) < max_speed ):
        print( "Clamped at max speed" )
        curr_speed = max_speed # cap that speed to the max
      elif int( curr_speed - speed_change ) > min_speed:
        curr_speed = min_speed
      else: 
        curr_speed = int( curr_speed - speed_change ) # minus will slow it down

      print( "Current speed: " + str(curr_speed) )
      
      if move:
          PWM.setMotorModel( curr_speed, curr_speed, curr_speed, curr_speed )

      time.sleep(0.25)
      
except KeyboardInterrupt:
    PWM.setMotorModel(0, 0, 0, 0)
    print ("\nEnd of program")
