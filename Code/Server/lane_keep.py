# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import sys
import numpy as np
#sys.path.append('/usr/local/lib/python2.7/site-packages')
import cv2
import servo

servo = servo.Servo()

servo.setServoPwm('0',90)
servo.setServoPwm('1',90)

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
rawCapture = PiRGBArray(camera)
# allow the camera to warmup
time.sleep(0.1)
# grab an image from the camera
camera.capture(rawCapture, format="bgr")
image = rawCapture.array
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

lower_blue = np.array([60, 40, 40])
upper_blue = np.array([150, 255, 255])
mask = cv2.inRange(hsv, lower_blue, upper_blue)



#edges = cv2.Canny(mask, 200, 400)



# display the image on screen and wait for a keypress
cv2.imshow("orig", image)
cv2.imshow("Image", mask)
cv2.waitKey(0)
