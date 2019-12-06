import os.path
from os import path
import numpy as np
import cv2
import cv2.aruco as aruco
from signal import signal, SIGINT, SIGABRT, SIGFPE
from sys import exit
import time
from PID import *
from teensy_controller import *
from area import *
import math

def handler(signal_received, frame):
    # Handle any cleanup here
    print('Removing launch file and exiting')
    video_capture.release()
    os.remove("launch")
    exit(0)

signal(SIGINT, handler)
signal(SIGABRT, handler)
signal(SIGFPE, handler)

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

MIDDLE_X = IMAGE_WIDTH/2

WRONG_DIRECTION_THRESHOLD = 90
CLOSE_MARKER_WIDTH = 150
MAX_SPEED = 0
APPROACH_SPEED = 50
LOST_MARKER_ANGULAR_SPEED = 45

SERIAL_PORT = "/dev/ttyAMA0"
BAUDRATE = 9600

angular_pid_marker = PID(Kp = 0.2, Ki = 0.0002)
linear_pid_marker = PID(Kp = 0.5)

video_capture = cv2.VideoCapture(0)
video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH)
video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT)

robot = TeensyController(SERIAL_PORT, BAUDRATE)

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters =  aruco.DetectorParameters_create()

is_arrived = False
ingredient_not_found = True
firstDirection = True

stopCounter = 0
middlepoint = [0,0]

def grabbingInstructions():
    print("Ingredient Found --> Grabbing...")
    robot.set_velocities(0,0)
    robot.open_gripper()
    robot.set_gripper_grabbing_position()
    time.sleep(0.5)
    robot.set_velocities(APPROACH_SPEED,0)
    time.sleep(0.5)
    robot.set_velocities(0,0)
    time.sleep(0.3)
    robot.close_gripper()
    time.sleep(0.5)
    robot.set_gripper_releasing_position()
    time.sleep(0.5)
    robot.set_gripper_grabbing_position()
    #robot.set_velocities(-APPROACH_SPEED,0)

def arucoDetection():
    # Capture frame-by-frame
    ret, frame = video_capture.read()
    rows,cols = IMAGE_HEIGHT, IMAGE_WIDTH
    M = cv2.getRotationMatrix2D(((cols-1)/2.0,(rows-1)/2.0),180,1)
    frame = cv2.warpAffine(frame,M,(cols,rows))
    #print(frame.shape) #480x640
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    #print(corners)
    gray = aruco.drawDetectedMarkers(gray, corners)
    return gray, corners, ids

while True:
    if path.exists("launch") and ingredient_not_found:
        print("Salad Launched")
        while ingredient_not_found:
            gray, corners, ids = arucoDetection()
            
            if len(corners):
                #print(corners)
                for i in range(len(corners)):
                    if(ids[i]==6):
                        middlepoint = [int((corners[i][0][0][0]+corners[i][0][1][0]+corners[i][0][2][0]+corners[i][0][3][0])/4),
                                        int((corners[i][0][0][1]+corners[i][0][1][1]+corners[i][0][2][1]+corners[i][0][3][1])/4)]
                        cv2.line(gray,(middlepoint[0],0),(middlepoint[0],720),(255,0,0),1)
                        print("Direction :", end="")
                        print(MIDDLE_X-middlepoint[0])
                        
                        #Drive the robot to the direction
                        ang_vel = angular_pid_marker.update(middlepoint[0], MIDDLE_X)
                        lin_vel = max(0,MAX_SPEED-abs(linear_pid_marker.update(middlepoint[0], MIDDLE_X)))
                        robot.set_velocities(lin_vel, -ang_vel)
                        time.sleep(0.5)
                        robot.reset_odometry()
                        robot.set_velocities(0,0)
                        
                        if abs(corners[i][0][0][0]-corners[i][0][1][0]) > CLOSE_MARKER_WIDTH and ingredient_not_found:
                            cv2.imwrite('closeMarker.jpg', gray)
                            grabbingInstructions()
                            ingredient_not_found = False
                            

            #Make the robot slowly turn until it sees the necessary marker
            else:
                if abs(robot.get_odometry()[2]) < WRONG_DIRECTION_THRESHOLD and firstDirection:
                    if stopCounter > 2:
                        robot.set_velocities(0, math.copysign(LOST_MARKER_ANGULAR_SPEED,MIDDLE_X-middlepoint[0]))
                        stopCounter = 0
                    else:
                        stopCounter = stopCounter + 1
                        robot.set_velocities(0, 0)
                    time.sleep(0.5)
                else:
                    firstDirection = False
                    if stopCounter > 2:
                        robot.set_velocities(0, -math.copysign(LOST_MARKER_ANGULAR_SPEED,MIDDLE_X-middlepoint[0]))
                        stopCounter = 0
                    else:
                        stopCounter = stopCounter + 1
                        robot.set_velocities(0, 0)
                    time.sleep(0.5)
                

            # Display the resulting frame
            # cv2.imshow('frame',gray)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break

    if ingredient_not_found==False:
        #Program End -> Cleanup
        video_capture.release()
        os.remove("launch")
        break        
