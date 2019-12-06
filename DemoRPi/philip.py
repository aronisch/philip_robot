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
SEARCH_AREA_HEIGHT = 50
SEARCH_AREA_WIDTH = 200

MIDDLE_X = IMAGE_WIDTH/2

WRONG_DIRECTION_THRESHOLD = 165

MAX_SPEED = 250
LOST_LINE_ANGULAR_SPEED = 45

SERIAL_PORT = "/dev/ttyAMA0"
BAUDRATE = 9600

video_capture = cv2.VideoCapture(0)
video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH)
video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT)

endOfLine = False

angular_pid_line = PID(Kp = 0.27, Ki = 0.00002, windup = 45)
linear_pid_line = PID(Kp = 1.0)

angular_pid_marker = PID(Kp = 0.05, Ki = 0.025)
linear_pid_marker = PID(Kp = 2)

#create the object to control the robot 
robot = TeensyController(SERIAL_PORT, BAUDRATE)
lin_vel = 0
ang_vel = 0
    
search_area = Area(IMAGE_WIDTH/2, IMAGE_HEIGHT-SEARCH_AREA_HEIGHT/2,IMAGE_WIDTH, SEARCH_AREA_HEIGHT) 

firstDirection = True

while True:
    if path.exists("launch"):
        print("Launched")
        while(not(endOfLine)):
 
            # Capture the frames
            ret, frame = video_capture.read()
            rows,cols = 480, 640
            M = cv2.getRotationMatrix2D(((cols-1)/2.0,(rows-1)/2.0),180,1)
            frame = cv2.warpAffine(frame,M,(cols,rows))
            
            x_min, x_max, y_min, y_max = search_area.corners()
            #print(search_area.corners())
            # Crop the image
            crop_img = frame[max(0,int(y_min)): min(IMAGE_HEIGHT,int(y_max)),max(int(x_min),0): min(IMAGE_WIDTH, int(x_max))]
            # Convert to grayscale
            gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
        
            # Gaussian blur
            blur = cv2.GaussianBlur(gray,(5,5),0)
        
            # Color thresholding
            ret,thresh = cv2.threshold(blur,((np.amax(blur)-np.amin(blur))*3/4),255,cv2.THRESH_BINARY_INV)
            
            # Find the contours of the frame
            contours,hierarchy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)
            
            # Find the biggest contour (if detected)
            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                contourValue = cv2.contourArea(c)
                if(contourValue > 500):
                    M = cv2.moments(c)
            
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
            
                    cv2.line(crop_img,(cx,0),(cx,720),(255,0,0),1)
                    cv2.line(crop_img,(0,cy),(1280,cy),(255,0,0),1)
            
                    
                    cv2.drawContours(crop_img, [c], -1, (0,255,0), 1)
                    # w,h = search_area.shape()
                    # contourImg = np.zeros(shape=[h, w, 1], dtype=np.uint8)
                    
                    # cv2.drawContours(contourImg, [c], -1, 255, 1)

                    # cv2.imshow('con',contourImg)
                    # first_point = find_first_point(c)
                    # furthest_point = find_furthest_point(contourImg, first_point, len(c)/2)

                    
                    #Control Motors with Deviation
                    line_real_loc = search_area.get_real_coordinate(cx,cy)
                    print("Deviation: ", end="")
                    print(MIDDLE_X-line_real_loc[0])
                    ang_vel = angular_pid_line.update(line_real_loc[0], MIDDLE_X)
                    lin_vel = MAX_SPEED - abs(linear_pid_line.update(line_real_loc[0], MIDDLE_X))
                    
                    robot.set_velocities(lin_vel, ang_vel)
                    search_area.set_position(min(IMAGE_WIDTH-search_area.width/2,max(0+search_area.width/2,line_real_loc[0])), min(IMAGE_HEIGHT-search_area.height/2,max(0+search_area.height/2,IMAGE_HEIGHT-40)) )
                    search_area.set_shape(SEARCH_AREA_WIDTH,SEARCH_AREA_HEIGHT)
                    robot.reset_odometry()
                    firstDirection = True
                # else:
                #     print("End Of Line")
                #     endOfLine = True
                #     robot.set_velocities(0, 0)
            else:
                print("No Line")
                print(robot.get_odometry())
                if abs(robot.get_odometry()[2]) < WRONG_DIRECTION_THRESHOLD and firstDirection:
                    robot.set_velocities(0, math.copysign(LOST_LINE_ANGULAR_SPEED,MIDDLE_X-line_real_loc[0]))
                else:
                    firstDirection = False
                    robot.set_velocities(0, -math.copysign(LOST_LINE_ANGULAR_SPEED,MIDDLE_X-line_real_loc[0]))

            #Display the resulting frame
            # cv2.imshow('frame',crop_img)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break
            
       
        is_arrived = False 
        #start searching the marker
        while(True):
            # Capture frame-by-frame
            ret, frame = video_capture.read()
            rows,cols = 480, 640
            M = cv2.getRotationMatrix2D(((cols-1)/2.0,(rows-1)/2.0),180,1)
            frame = cv2.warpAffine(frame,M,(cols,rows))
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
                        ang_vel = angular_pid_marker.update(middlepoint[0], MIDDLE_X)
                        robot.set_velocities(200, ang_vel)

            #Make the robot slowly turn until it sees the necessary marker
            else:
                robot.set_velocities(0, 0.5)

            # Display the resulting frame
            # cv2.imshow('frame',gray)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break

            if is_arrived:
                break
        
        video_capture.release()
        cv2.destroyAllWindows()
        os.remove("launch")



def find_first_point(contour):
    index_y_max = 0
    first_point = []
    for point in c:
        if point[0][1] > index_y_max:
            index_y_max = point[0][1]
            first_point = point[0]
    
    return tuple(first_point)

# def find_neighbors(point, contour):
#     neighbors = []
#     for c in contour:
#         if abs(point[0]-c[0])<=1 and abs(point[1]-c[1])<=1 and point!=c:
#             neighbors.append(c)
            
#     return neighbors

def find_neighbors(point, contour):
    neighbors = []
    w,h,_= contour.shape
    for i in [-1,0,1]:
        for j in [-1,0,1]:
            xi, yj = min(h-1, max(0,point[0]+i)), min(w-1, max(0,point[1]+j))
            if contour[yj,xi] == 255 and not (i == j and i==0):
                neighbors.append((xi,yj))
    return neighbors

def maxCost(elem):
    return elem[1]

def find_furthest_point(contourImg, first_point, max_search_dist):
    closed_list = []
    open_list = []
    current_point = [first_point, 0]
    open_list.append(current_point)
    while len(open_list):
        # print("Open")
        # print(open_list)
        # print("Closed")
        # print(closed_list)
        open_list.sort(key=maxCost)
        open_list = open_list[::-1]
        current_point = open_list.pop(0)
        closed_list.append(current_point)
        neighbors = find_neighbors(current_point[0], contourImg)
        
        if current_point[1] >= max_search_dist:
             return current_point[0]
        for n in neighbors:
            temp_neighbor = [n, current_point[1]+1]
            if not any(n in sublist for sublist in closed_list):
                append = True
                for node in open_list:
                    if node[0]==temp_neighbor[0] and node[1]>temp_neighbor[1]:
                        append = False
                if append:
                    open_list.append(temp_neighbor)
            else:
                for o in closed_list:
                    if o[0] == temp_neighbor[0] and o[1] > temp_neighbor[1]:
                        closed_list.remove(o)
                        open_list.append(temp_neighbor)
    

    closed_list.sort(key=maxCost)
    print(closed_list[-1][1])
    return closed_list[-1][0]