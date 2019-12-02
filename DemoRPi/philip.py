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

video_capture = cv2.VideoCapture(0)
video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH)
video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT)

MIDDLE_X = IMAGE_WIDTH/2

endOfLine = False

angular_pid_line = PID(Kp = 0.005, Ki = 0.0025)
linear_pid_line = PID(Kp = 2)

angular_pid_marker = PID(Kp = 0.05, Ki = 0.025)
linear_pid_marker = PID(Kp = 2)

MAX_SPEED = 500

SERIAL_PORT = 
BAUDRATE = 9600

#create the object to control the robot 
robot = TeensyController(SERIAL_PORT, BAUDRATE)

while True:
    if path.exists("launch"):
        print("Launched")
        while(not(endOfLine)):
 
            # Capture the frames
            ret, frame = video_capture.read()
        
            # Crop the image
            crop_img = frame[340:480, 0:640]
            # Convert to grayscale
            gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
        
            # Gaussian blur
            blur = cv2.GaussianBlur(gray,(5,5),0)
        
            # Color thresholding
            ret,thresh = cv2.threshold(blur,((np.amax(blur)-np.amin(blur))/2),255,cv2.THRESH_BINARY_INV)
            # Find the contours of the frame
            contours,hierarchy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)
        
            # Find the biggest contour (if detected)
            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                contourValue = cv2.contourArea(c)
                if(contourValue > 400):
                    M = cv2.moments(c)
            
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
            
                    cv2.line(crop_img,(cx,0),(cx,720),(255,0,0),1)
                    cv2.line(crop_img,(0,cy),(1280,cy),(255,0,0),1)
            
                    cv2.drawContours(crop_img, contours, -1, (0,255,0), 1)

                    print("Deviation: ", end="")
                    print(320-cx)
                    #Control Motors with Deviation
                    
                    ang_vel = angular_pid_line.update(cx, MIDDLE_X)
                    lin_vel = MAX_SPEED - linear_pid_line.update(cx, MIDDLE_X)
                    
                    robot.set_velocities(lin_vel, ang_vel)
                else:
                    print("End Of Line")
                    endOfLine = True
                    robot.set_velocities(0, 0)
            else:
                print("No Line")
                robot.set_velocities(0, 1)
        
            #Display the resulting frame
            cv2.imshow('frame',crop_img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
       
       
        #start searching the marker
        while(True):
            # Capture frame-by-frame
            ret, frame = video_capture.read()
            #print(frame.shape) #480x640
            # Our operations on the frame come here
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
            parameters =  aruco.DetectorParameters_create()
            
            #lists of ids and the corners beloning to each id
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            #print(corners)
        
            gray = aruco.drawDetectedMarkers(gray, corners)
            
            if len(corners):
                #print(corners)
                for i in range(len(corners)):
                    if(ids[i]==6):
                        middlepoint = [int((corners[i][0][0][0]+corners[i][0][1][0]+corners[i][0][2][0]+corners[i][0][3][0])/4),
                                       int((corners[i][0][0][1]+corners[i][0][1][1]+corners[i][0][2][1]+corners[i][0][3][1])/4)]
                        cv2.line(gray,(middlepoint[0],0),(middlepoint[0],720),(255,0,0),1)
                        print("Direction :", end="")
                        print(320-middlepoint[0])
                        #Drive the robot to the direction
                        linear_pid_marker
            
            #Make the robot slowly turn until it sees the necessary marker
            robot.set_velocities(0, 0.5)

            # Display the resulting frame
            cv2.imshow('frame',gray)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        video_capture.release()
        cv2.destroyAllWindows()
        os.remove("launch")



