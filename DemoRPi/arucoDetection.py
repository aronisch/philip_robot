import numpy as np
import cv2
import cv2.aruco as aruco
import time
 
 
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4,480)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    #print(frame.shape) #480x640
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()
    
    #lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    print(corners)
  
    gray = aruco.drawDetectedMarkers(gray, corners)
    
    if len(corners):
        for i in range(len(corners)):
            if(ids[i]==6):
                middlepoint = int((corners[i][0][0][0]+corners[i][0][1][0])/2)
                cv2.line(gray,(middlepoint,0),(middlepoint,720),(255,0,0),1)
                print(corners[i][0][0][0], end=" X1 \n")
                print(corners[i][0][1][0], end=" X2 \n")
                print(corners[i][0][2][0], end=" X3 \n")
                print(corners[i][0][3][0], end=" X4 \n")
                print(time.monotonic())
        
    # Display the resulting frame
    # cv2.imshow('frame',gray)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break
 
cap.release()
#cv2.destroyAllWindows()
