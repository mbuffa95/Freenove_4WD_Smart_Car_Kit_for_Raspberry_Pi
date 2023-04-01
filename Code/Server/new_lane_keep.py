import cv2
import numpy as np
import matplotlib.image as mpimg
import time
import matplotlib.pyplot as plt
import os
import math
import logging
from picamera.array import PiRGBArray
from picamera import PiCamera
import servo
import Motor
#import keyboard

max_speed = 1000
wheel_speed_min = 350 # the speed at which the wheel stop spinning
change_dir_err = 22

slope = (max_speed - wheel_speed_min) / change_dir_err

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
                                    np.array([]), minLineLength=115, maxLineGap=20)

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
    if slope != 0:
        x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
        x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
        return [[x1, y1, x2, y2]], True
    else:
        return [[x1, y1, x2, y2]], False

def average_slope_intercept(frame, line_segments):
    """
    This function combines line segments into one or two lane lines
    If all line slopes are < 0: then we only have detected left lane
    If all line slopes are > 0: then we only have detected right lane
    """
    lane_lines = []
    if line_segments is None:
        logging.info('No line_segment segments detected')
        return lane_lines, False

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1/2
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
        [[x1, y1, x2, y2]], success = make_points(frame, left_fit_average)
        if success:
            lane_lines.append([[x1, y1, x2, y2]])
        else:
            return lane_lines, False
    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        [[x1, y1, x2, y2]], success = make_points(frame, right_fit_average)
        if success:
            lane_lines.append([[x1, y1, x2, y2]])
        else:
            return lane_lines, False
    logging.debug('lane lines: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]

    return lane_lines, True

def calculate_steering_angle(x_offset, y_offset):
    angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
    return angle_to_mid_deg + 90  # this is the steering angle needed by picar front wheel

def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5 ):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    # figure out the heading line from steering angle
    # heading line (x1,y1) is always center bottom of the screen
    # (x2, y2) requires a bit of trigonometry

    # Note: the steering angle of:
    # 0-89 degree: turn left
    # 90 degree: going straight
    # 91-180 degree: turn right 
    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image

def stabilize_steering_angle(
          curr_steering_angle, 
          new_steering_angle, 
          num_of_lane_lines, 
          max_angle_deviation_two_lines=60, 
          max_angle_deviation_one_lane=40):
    """
    Using last steering angle to stabilize the steering angle
    if new angle is too different from current angle, 
    only turn by max_angle_deviation degrees
    """
    if num_of_lane_lines == 2 :
        # if both lane lines detected, then we can deviate more
        max_angle_deviation = max_angle_deviation_two_lines
    else :
        # if only one lane detected, don't deviate too much
        max_angle_deviation = max_angle_deviation_one_lane
    
    angle_deviation = new_steering_angle - curr_steering_angle
    if abs(angle_deviation) > max_angle_deviation:
        stabilized_steering_angle = int(curr_steering_angle
            + max_angle_deviation * angle_deviation / abs(angle_deviation))
    else:
        stabilized_steering_angle = new_steering_angle
    return stabilized_steering_angle

def display_still(caption, frame):
    if DISPLAY_STILLS:
        cv2.imshow(caption, frame)
    return

def write_still(file_name, frame):
    if WRITE_STILLS:
        os.chdir("/home/mbuffa/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Images")
        cv2.imwrite(file_name, frame)
    return

def get_wheel_speeds(goal_steer_angle):
    if( goal_steer_angle >= 88 and goal_steer_angle <= 92 ):
        print('going straight')
        return (-max_speed, -max_speed, -max_speed, -max_speed)
    elif( goal_steer_angle < 88 ):
        # need to turn left
        print('going left')
        err = 87 - goal_steer_angle
        print('err: ', err)

        if err < change_dir_err:
            # keep turning the left wheels in the same direction, but at a slower rate
            slope_x_error = slope * err
            print('both wheels turning same direction')
            print('slope =', slope)
            print('slope * error = ', slope_x_error )
            print('slope * error - b = ', slope_x_error + (-max_speed)) 
            speed = int(slope_x_error + (-max_speed))
            return (speed, speed, -max_speed, -max_speed)
        else:
            # turn the wheel in the opposite direction to turn sharper
            print('left wheels spinning opposite!!!')
            slope_x_error = slope * err
            b = (wheel_speed_min/(slope*change_dir_err))
            print('slope =', slope)
            print('slope * error = ', slope_x_error )
            print('slope * error - b = ', slope_x_error + b) 
            speed = int(slope_x_error + b)
            return (speed, speed, -max_speed, -max_speed)

    else:
        # need to turn right
        print('going right')
        err = goal_steer_angle - 93
        print('err: ', err)

        if err < change_dir_err:
            # keep turning the left wheels in the same direction, but at a slower rate
            print('both wheels turning same direction')
            slope_x_error = slope * err
            print('slope =', slope)
            print('slope * error = ', slope_x_error )
            print('slope * error - b = ', slope_x_error + (-max_speed)) 
            speed = int(slope_x_error + (-max_speed))
            return (-max_speed, -max_speed, speed, speed)
        else:
            # turn the wheel in the opposite direction to turn sharper
            print('right wheels spinning opposite!!!')
            slope_x_error = slope * err
            b = (wheel_speed_min/(slope*change_dir_err))
            print('slope =', slope)
            print('slope * error = ', slope_x_error )
            print('slope * error - b = ', slope_x_error + b) 
            speed = int(slope_x_error + b)
            return (-max_speed, -max_speed, speed, speed)


def cleanup():
    """ Reset the hardware"""
    logging.info('Stopping the car, resetting hardware.')
    '''self.back_wheels.speed = 0
    self.front_wheels.turn(90)
    self.camera.release()
    self.video_orig.release()
    self.video_lane.release()
    self.video_objs.release()'''
    PWM.setMotorModel(0,0,0,0)
    cv2.destroyAllWindows()
    return

DISPLAY_STILLS = False
WRITE_STILLS = False

servo = servo.Servo()
servo.setServoPwm('0',90)
servo.setServoPwm('1',90)
crop_ratio = ( 14 / 20 )
cur_steering_angle = 90 #assume starting out exactly in the middle of the lanes

# Read image 
#image = cv2.imread('/home/mbuffa/test_img/test_images/exit-ramp.jpg', cv2.IMREAD_COLOR) # roadpng is the filename
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
heading_error = 0

# allow the camera to warmup
time.sleep(1)

lastFrameTime = time.time()

frame = PiRGBArray(camera, size = (640,480))
PWM= Motor.Motor()
#PWM.setMotorModel(-350,-350,-350,-350)
#PWM.setMotorModel(-2000,-2000,0,0)
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    image = frame.array

    display_still("orig", image)

    # Convert the image to gray-scale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    #kernel_size = 1
    #blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)

    low_threshold = 50
    high_threshold = 150
    masked_edges = cv2.Canny(image, low_threshold, high_threshold)

    display_still("masked edges", masked_edges)
    cropped_img = region_of_interest(masked_edges)

    display_still("cropped image", cropped_img)

    line_segs = detect_line_segments(cropped_img)
    line_segs_img = display_lines(image, line_segs)
    display_still("detected lines", line_segs_img)

    lane_lines, success = average_slope_intercept(image, line_segs)

    if success: 
        lane_lines_img = display_lines(line_segs_img, lane_lines, (255, 0, 255))
        display_still("lane lines", lane_lines_img)

        height, width, _ = image.shape

        if len(lane_lines) == 2:
            # detected 2 lane lines
            print('identified 2 lane lines')
            _, _, left_x2, _ = lane_lines[0][0]
            _, _, right_x2, _ = lane_lines[1][0]
            mid = int(width / 2)
            x_offset = (left_x2 + right_x2) / 2 - mid
            y_offset = int(height / 2)
        elif len(lane_lines) == 1:
            # only detected 1 lane line
            print('identified 1 lane lines')
            x1, _, x2, _ = lane_lines[0][0]
            x_offset = x2 - x1
            y_offset = int(height / 2)

        if len(lane_lines) == 1 or len(lane_lines) == 2:
            new_steering_angle = calculate_steering_angle(x_offset, y_offset)
            new_stable_steering_angle = stabilize_steering_angle(cur_steering_angle, new_steering_angle, len(lane_lines))
            
            print('current steering angle: ', cur_steering_angle)
            print('new steering angle: ', new_steering_angle)
            print('new stable steering angle: ', new_stable_steering_angle)

            if new_stable_steering_angle >= 0 and new_stable_steering_angle <= 180:
                # desired heading is pretty straight  
                fl_speed, bl_speed, fr_speed, br_speed = get_wheel_speeds(new_stable_steering_angle) 
                print('Front left speed: ', fl_speed)
                print('Back left speed: ', bl_speed)
                print('Front right speed: ', fr_speed)
                print('Back right speed: ', br_speed)
                PWM.setMotorModel(fl_speed, bl_speed, fr_speed, br_speed)
            else:
                print('invalid new stable steering angle: ', new_stable_steering_angle)

            heading_img = display_heading_line(lane_lines_img, new_stable_steering_angle)
            display_still("heading", heading_img)

            cur_steering_angle = new_stable_steering_angle
            if( time.time() - lastFrameTime >= (1/30) ):
                lastFrameTime = time.time()
                cv2.imshow('detected lines', lane_lines_img)
        else:
            print('identified an invalid number of lane lines', len(lane_lines), ', discarding this frame' )
    else:
        print('failed to average lines, skipping this frame')

    #write_still("gray.png", gray)
    #write_still("masked_edges.png", masked_edges)
    #write_still("cropped.png", cropped_img)
    #write_still("original.png", image)
    #write_still("detected_lines.png", line_segs_img)
    #write_still("lane_lines.png", lane_lines_img)
    #write_still("heading_img.png", heading_img)


    frame.truncate(0)

    if cv2.waitKey(1) & 0xFF == ord('q'): #or keyboard.is_pressed("q"):
        cleanup()
        break

