#import the necessary modules
import freenect
import cv2
import frame_convert2
import numpy as np
import shapedetector
from scipy.spatial import distance as dist
#import RPi.GPIO as GPIO            # import RPi.GPIO module  
import time

# UPDATES: 
# Code is fully functioning on a plugged in, regular kinect
# Issues witht the electrical enginnering and code optimization at this point

# an iteration beyond the previous contouring 
# had hsv processing to find contours and then if a box was found within a certain range
# it would detect it as a flag
# kept the running average of conoturs as well in order to aim the robot
# added GPIO code in order to send signals from Raspberry Pi to Brain
avg = []
size = 10
#GPIO.setmode(GPIO.BCM)
READY = 17
READY2SHOOT = 18
LEFT = 22
RIGHT = 23
#GPIO.setup(READY, GPIO.OUT)
#GPIO.setup(READY2SHOOT, GPIO.OUT)
#GPIO.setup(LEFT, GPIO.OUT)
#GPIO.setup(RIGHT, GPIO.OUT)
 
#function to get RGB image from kinect
def get_video():
    array,_ = freenect.sync_get_video()
    return array
 
#function to get depth image from kinect
def get_depth():
    array,_ = freenect.sync_get_depth()
    array = array.astype(np.uint8)
    return array

def midpoint(ptA, ptB):
    return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)


def reset():
    #GPIO.output(READY, 0)
    #GPIO.output(READY2SHOOT, 0)
    #GPIO.output(LEFT,0)
    #GPIO.output(RIGHT, 0)
    return

def go_ready():
    reset()
    #GPIO.output(READY, 1)

def go_shoot():
    reset()
    #GPIO.output(READY2SHOOT, 1)

def go_left():
    reset()
    #GPIO.output(RIGHT, 1)

def go_right():
    reset()
    #GPIO.output(LEFT, 1)


if __name__ == "__main__":
    start = time.time()
    while time.time() - start < 3:
        #GPIO.output(READY, 0)
        temp = get_video()
    while 1:
        #get a frame from RGB camera
        #frame = get_video()
        #get a frame from depth sensor
        #depth = get_depth()
        #GPIO.output(READY, 1)
        
        # lower mask (0-10)
        lower_red = np.array([0,200,100])
        upper_red = np.array([10,255,255])
        array = get_video()
        img_hsv = cv2.cvtColor(array,cv2.COLOR_BGR2HSV)
        mask0 = cv2.inRange(img_hsv, lower_red, upper_red)


        # upper mask (170-180)
        lower_red = np.array([170,200,100])
        upper_red = np.array([180,255,255])
        mask1 = cv2.inRange(img_hsv, lower_red, upper_red)

        lower_blue = np.array([100, 100, 100])
        upper_blue = np.array([170, 255, 255])
        maskBlue = cv2.inRange(img_hsv, lower_blue, upper_blue)

        #lower_yellow = np.array([15, 0, 0])
        #upper_yellow = np.array([60, 255, 255])
        #maskYellow = cv2.inRange(img_hsv, lower_yellow, upper_yellow)

        mask = mask0+mask1+maskBlue#+maskYellow


        #mask = cv2.inRange(cv2.cvtColor(frame_convert2.video_cv(data), cv2.COLOR_BGR2HSV), (20, 0, 100), (55, 255, 255))
        # contouring code to find the largest objects still in reference after the mask
        output = cv2.bitwise_and(array, array, mask = mask)

        gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (7, 7), 0)

# perform edge detection, then perform a dilation + erosion to
# close gaps in between object edges
        edged = cv2.Canny(gray, 50, 100)
        edged = cv2.dilate(edged, None, iterations=1) #likely don't need erosion and dilation - saves some compute
        edged = cv2.erode(edged, None, iterations=1)
 
# find contours in the edge map
        cnts, hier = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    #gets the largest area of what is included in the contours and focuses on that
    #cnts = imutils.grab_contours(cnts[0])
        largest = 0
        area = 0
        num = 0
        for x in range(len(cnts)):
            area = cv2.contourArea(cnts[x])
            if area > largest:
                largest = area
                num = x
        #adds the center of the contours to the average array in orde to have a running average every frame
        rect = cv2.minAreaRect(cnts[num])
        box = cv2.cv.BoxPoints(rect)
        box = np.int0(box)
        avg.append(midpoint(box[0],box[2]))
        ex = 0
        why = 0
        for each in avg:
            ex += each[0]/size
            why += each[1]/size
        print(ex, why)
        if len(avg) >= size:
                avg.pop(0)
        if(ex < 300):
            go_left()
        elif(ex > 340):
            go_right()
        elif ex < 340 and ex > 300:
            go_shoot()
            
            
        #display RGB image
        #cv2.imshow('RGB image',frame)
        #display depth image
        #cv2.imshow('Depth image',depth)
 
        # quit program when 'esc' key is pressed
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break
    cv2.destroyAllWindows()
