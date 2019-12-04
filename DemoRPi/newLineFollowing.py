import numpy as np
import cv2
import cv2.aruco as aruco
import math

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

video_capture = cv2.VideoCapture(0)
video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH)
video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT)

MIDDLE_X = IMAGE_WIDTH/2

endOfLine = False

lin_vel = 0
ang_vel = 0

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

# def find_furthest_point(contour, first_point):
#     contour_list = np_to_list(contour)
#     closed_list = []
#     open_list = []
#     current_point = [first_point, 0]
#     open_list.append(current_point)
#     while len(open_list):
#         # print("Open")
#         # print(open_list)
#         # print("Closed")
#         # print(closed_list)
#         open_list.sort(key=maxCost)
#         open_list = open_list[::-1]
#         current_point = open_list.pop(0)
#         closed_list.append(current_point)
#         neighbors = find_neighbors(current_point[0], contour_list)
        
#         if current_point[1] >= 180:
#             return current_point[0]
#         for n in neighbors:
#             temp_neighbor = [n, current_point[1]+1]
#             if not any(n in sublist for sublist in closed_list):
#                 append = True
#                 for node in open_list:
#                     if node[0]==temp_neighbor[0] and node[1]>temp_neighbor[1]:
#                         append = False
#                 if append:
#                     open_list.append(temp_neighbor)
#             else:
#                 for o in closed_list:
#                     if o[0] == temp_neighbor[0] and o[1] > temp_neighbor[1]:
#                         closed_list.remove(o)
#                         open_list.append(temp_neighbor)
    

#     closed_list.sort(key=maxCost)
#     print(closed_list[-1][1])
#     return closed_list[-1][0]
                
    
def np_to_list(contour):
    not_shitty_list = []
    for p in contour:
        not_shitty_list.append( (p[0][0], p[0][1]))
    return not_shitty_list
    
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
        return (self.width,self.height)
    
    def corners(self):
        return self.x-self.width/2, self.x+self.width/2, self.y-self.height/2, self.y+self.height/2
    
    def get_real_coordinate(self, x_loc, y_loc):
        return self.x-self.width/2 + x_loc, self.y-self.height/2+ y_loc
    
    
search_area = Area(IMAGE_WIDTH/2, IMAGE_HEIGHT-40,IMAGE_WIDTH, 80) 

while True:
    # Capture the frames
    ret, frame = video_capture.read()
    rows,cols = 480, 640
    M = cv2.getRotationMatrix2D(((cols-1)/2.0,(rows-1)/2.0),0,1)
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
    ret,thresh = cv2.threshold(blur,((np.amax(blur)-np.amin(blur))/2),255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
    # Find the contours of the frame
    contours,hierarchy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)
    
    # Find the biggest contour (if detected)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        contourValue = cv2.contourArea(c)
        if(contourValue > 200):#################################################TOCHECK
            M = cv2.moments(c)

            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
    
            
    
            cv2.drawContours(crop_img, [c], -1, (0,255,0), 1)
            w,h = search_area.shape()
            contourImg = np.zeros(shape=[h, w, 1], dtype=np.uint8)
            
            cv2.drawContours(contourImg, [c], -1, 255, 1)

            # cv2.imshow('con',contourImg)
            first_point = find_first_point(c)
            furthest_point = find_furthest_point(contourImg, first_point, len(c)/2)
            print(first_point)
            print(furthest_point)
            cv2.line(crop_img,(first_point[0],0),(first_point[0],720),(255,0,0),1)
            cv2.line(crop_img,(0,first_point[1]),(1280,first_point[1]),(255,0,0),1)

            cv2.line(crop_img,(furthest_point[0],0),(furthest_point[0],720),(0,255,0),1)
            cv2.line(crop_img,(0,furthest_point[1]),(1280,furthest_point[1]),(0,255,0),1)
            
            c_list = np_to_list(c)
            midpoint = (c_list[int(len(c_list)/2)][0], c_list[int(len(c_list)/2)][1])
            
            # cv2.line(crop_img,(midpoint[0],0),(midpoint[0],720),(0,0,255),1)
            # cv2.line(crop_img,(0,midpoint[1]),(1280,midpoint[1]),(0,0,255),1)
            
            #Control Motors with Deviation
            line_real_loc = search_area.get_real_coordinate(cx,cy)
            print("Deviation: ", end="")
            print(320-line_real_loc[0])
            #ang_vel = angular_pid_line.update(line_real_loc[0], MIDDLE_X)
            lin_vel = 130#MAX_SPEED - linear_pid_line.update(cx, MIDDLE_X)
            
            #robot.set_velocities(lin_vel, ang_vel)
            
            cv2.imshow('frame',crop_img)
            #cv2.imwrite('frame.jpg', crop_img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        # else:
        #     print("End Of Line")
        #     endOfLine = True
        #     robot.set_velocities(0, 0)
    else:
        print("No Line")
        cv2.imshow('frame',crop_img)
        #cv2.imwrite('frame.jpg', crop_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        #robot.set_velocities(0, math.copysign(45,320-line_real_loc[0]))

    #Display the resulting frame
    # cv2.imshow('frame',crop_img)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break