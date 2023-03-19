import cv2
import numpy as np
import matplotlib.image as mpimg
import time
import matplotlib.pyplot as plt
import os
from picamera.array import PiRGBArray
from picamera import PiCamera
import servo

servo = servo.Servo()

servo.setServoPwm('0',90)
servo.setServoPwm('1',90)

# Read image 
#image = cv2.imread('/home/mbuffa/test_img/test_images/exit-ramp.jpg', cv2.IMREAD_COLOR) # roadpng is the filename
camera = PiCamera()
rawCapture = PiRGBArray(camera)
# allow the camera to warmup
time.sleep(0.1)
# grab an image from the camera
camera.capture(rawCapture, format="bgr")
image = rawCapture.array

cv2.imshow("orig", image)

# Convert the image to gray-scale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

#kernel_size = 1
#blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)

low_threshold = 50
high_threshold = 150
masked_edges = cv2.Canny(image, low_threshold, high_threshold)

def region_of_interest(edges):
   height, width = edges.shape
   mask = np.zeros_like(edges)

   #crop out top portion of the screen
   polygon = np.array([[
      (0, height * 2 / 3),
      (width, height * 2 / 3),
      (width, height),
      (0, height),
   ]], np.int32)

   cv2.fillPoly(mask, polygon, 255)
   cropped_edges = cv2.bitwise_and(edges, mask)
   return cropped_edges


cv2.imshow("masked edges", masked_edges)
cropped_img = region_of_interest(masked_edges)
cv2.imshow("cropped image", cropped_img)


'''rho = 1
theta = np.pi/180
threshold = 1
min_line_length = 10
max_line_gap = 1
line_image = np.copy(image)*0

lines = cv2.HoughLinesP(masked_edges, rho, theta, threshold, np.array([]), min_line_length, max_line_gap)

for line in lines:
   for x1, y1, x2, y2 in line:
      cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),10)

color_edges = np.dstack((masked_edges, masked_edges, masked_edges))

combo = cv2.addWeighted(color_edges, 0.8, line_image, 1, 0)

cv2.imshow("final", combo)
'''

os.chdir("/home/mbuffa/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Images")
cv2.imwrite("masked_edges.png", masked_edges)
cv2.imwrite("original.png", image)
cv2.imwrite("gray.png", gray)
cv2.imwrite("cropped.png", cropped_img)
cv2.waitKey(0)
