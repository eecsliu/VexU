import freenect
import cv2
import frame_convert2
import numpy as np
import shapedetector
from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
import imutils

keep_running = True
red = True
iter = 0

def displayRedOrBlue(dev, data, timestamp):
        global keep_running
        global red
        global iter
        # lower mask (0-10)
        lower_red = np.array([0,200,100])
        upper_red = np.array([10,255,255])
        img_hsv = cv2.cvtColor(frame_convert2.video_cv(data), cv2.COLOR_BGR2HSV)
        mask0 = cv2.inRange(img_hsv, lower_red, upper_red)


        # upper mask (170-180)
        lower_red = np.array([170,200,100])
        upper_red = np.array([180,255,255])
        mask1 = cv2.inRange(img_hsv, lower_red, upper_red)

        lower_blue = np.array([100, 100, 100])
        upper_blue = np.array([170, 255, 255])
        maskBlue = cv2.inRange(img_hsv, lower_blue, upper_blue)

        lower_yellow = np.array([20, 50, 50])
        upper_yellow = np.array([50, 255, 255])
        maskYellow = cv2.inRange(img_hsv, lower_yellow, upper_yellow)

        mask = mask0+mask1+maskBlue+maskYellow


        #mask = cv2.inRange(cv2.cvtColor(frame_convert2.video_cv(data), cv2.COLOR_BGR2HSV), (20, 0, 100), (55, 255, 255))
        
        output = cv2.bitwise_and(frame_convert2.video_cv(data), frame_convert2.video_cv(data), mask = mask)

        gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (7, 7), 0)

# perform edge detection, then perform a dilation + erosion to
# close gaps in between object edges
        edged = cv2.Canny(gray, 50, 100)
        edged = cv2.dilate(edged, None, iterations=1) #likely don't need erosion and dilation - saves some compute
        edged = cv2.erode(edged, None, iterations=1)
 
# find contours in the edge map
        cnts, hier = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
	
	#cnts = imutils.grab_contours(cnts[0])
        largest = 0
        area = 0
        num = 0
        for x in range(len(cnts)):
            area = cv2.contourArea(cnts[x])
            if area > largest:
                largest = area
                num = x
	
        rect = cv2.minAreaRect(cnts[num])
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        print(box)
        print(midpoint(box[0],box[2]))
        cv2.drawContours(output,[box],0,(0,0,255),2) 
        output = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(output, cv2.HOUGH_GRADIENT, 1, 10, param1=50,param2=30,minRadius=20,maxRadius=50)
        if circles is not None:
	# convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
 
	# loop over the (x, y) coordinates and radius of the circles
            for (x, y, r) in circles:
		# draw the circle in the output image, then draw a rectangle
		# corresponding to the center of the circle
                cv2.circle(output, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
 
        cv2.imshow("Line Boi", output)
        if cv2.waitKey(10) == 27:
            keep_running = False

def body(*args):
    if not keep_running:
        raise freenect.Kill
        
def midpoint(ptA, ptB):
    return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)

freenect.runloop(video=displayRedOrBlue, body=body)

print('Press ESC in window to stop')

