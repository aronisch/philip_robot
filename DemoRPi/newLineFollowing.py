import numpy as np
import cv2
import cv2.aruco as aruco

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

video_capture = cv2.VideoCapture(0)
video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH)
video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT)

MIDDLE_X = IMAGE_WIDTH/2

endOfLine = False

lin_vel = 0
ang_vel = 0

class Area:
    
    def __init__(self, x, y, w, h):
        self.height = h
        self.width = w
        self.x = x
        self.y = y
    
    def set_position(self, x, y):
        self.x = x
        self.y = y
    
    def set_shape(self, w, h):
        self.width = w
        self.height = h
    
    def shape(self):
        return self.width,self.height
    
    def corners(self):
        return self.x-self.width/2, self.x+self.width/2, self.y-self.height/2, self.y+self.height/2
    
    def get_real_coordinate(self, x_loc, y_loc):
        return self.x-self.width/2 + x_loc, self.y-self.height/2+ y_loc
    
    
search_area = Area(IMAGE_WIDTH/2, IMAGE_HEIGHT-40,IMAGE_WIDTH, 80) 

while True:
    
    print("Launched")

    # Capture the frames
    ret, frame = video_capture.read()
    rows,cols = 480, 640
    M = cv2.getRotationMatrix2D(((cols-1)/2.0,(rows-1)/2.0),180,1)
    frame = cv2.warpAffine(frame,M,(cols,rows))
    
    x_min, x_max, y_min, y_max = search_area.corners()
    print(search_area.corners())
    # Crop the image
    crop_img = frame[max(0,int(y_min)): min(IMAGE_HEIGHT,int(y_max)),max(int(x_min),0): min(IMAGE_WIDTH, int(x_max))]
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
            
            #Control Motors with Deviation
            line_real_loc = search_area.get_real_coordinate(cx,cy)
            print("Deviation: ", end="")
            print(320-line_real_loc[0])
            ang_vel = angular_pid_line.update(line_real_loc[0], MIDDLE_X)
            lin_vel = 130#MAX_SPEED - linear_pid_line.update(cx, MIDDLE_X)
            
            robot.set_velocities(lin_vel, ang_vel)
            search_area.set_position(min(IMAGE_WIDTH-search_area.width/2,max(0+search_area.width/2,line_real_loc[0])), min(IMAGE_HEIGHT-search_area.height/2,max(0+search_area.height/2,line_real_loc[1])) )
            search_area.set_shape(400,80)
            
        # else:
        #     print("End Of Line")
        #     endOfLine = True
        #     robot.set_velocities(0, 0)
    else:
        print("No Line")
        robot.set_velocities(0, math.copysign(45,320-line_real_loc[0]))

    #Display the resulting frame
    # cv2.imshow('frame',crop_img)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break