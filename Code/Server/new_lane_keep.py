import cv2
import numpy as np
import matplotlib.image as mpimg
import time
import matplotlib.pyplot as plt
import os
import logging
from picamera.array import PiRGBArray
from picamera import PiCamera
import servo

servo = servo.Servo()

servo.setServoPwm('0',90)
servo.setServoPwm('1',90)
crop_ratio = ( 13 / 20 )

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
      (0, height * crop_ratio),
      (width, height * crop_ratio),
      (width, height),
      (0, height),
   ]], np.int32)

   cv2.fillPoly(mask, polygon, 255)
   cropped_edges = cv2.bitwise_and(edges, mask)
   return cropped_edges

def detect_line_segments(cropped_edges):
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # distance precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
    min_threshold = 10  # minimal of votes
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, 
                                    np.array([]), minLineLength=8, maxLineGap=4)

    return line_segments

def display_lines(frame, lines, line_color=(0, 255, 0), line_width=10):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image

def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * crop_ratio)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]

def average_slope_intercept(frame, line_segments):
    """
    This function combines line segments into one or two lane lines
    If all line slopes are < 0: then we only have detected left lane
    If all line slopes are > 0: then we only have detected right lane
    """
    lane_lines = []
    if line_segments is None:
        logging.info('No line_segment segments detected')
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1/3
    left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
    right_region_boundary = width * boundary # right lane line segment should be on left 2/3 of the screen

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    logging.debug('lane lines: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]

    return lane_lines



cv2.imshow("masked edges", masked_edges)
cropped_img = region_of_interest(masked_edges)

cv2.imshow("cropped image", cropped_img)

line_segs = detect_line_segments(cropped_img)
line_segs_img = display_lines(image, line_segs)
cv2.imshow("detected lines", line_segs_img)

lane_lines = average_slope_intercept(image, line_segs)
lane_lines_img = display_lines(image, lane_lines)
cv2.imshow("lane lines", lane_lines_img)

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
cv2.imshow("detected_lines.png", line_segs_img)
cv2.imshow("lane_lines.png", lane_lines_img)
cv2.waitKey(0)
