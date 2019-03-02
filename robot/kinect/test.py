#library imports for vision processing
import freenect					#kinect library
import cv2					#vision processing library
import frame_convert2				#retrieves video
import numpy as np
import shapedetector				#used for contouring
from scipy.spatial import distance as dist
import RPi.GPIO as GPIO            # import RPi.GPIO module  
from time import sleep
#from imutils import perspective
#from imutils import contours
#import imutils

#Set up General Purpose Input/Output(GPIO) pins for communication
#between the raspberry pi and Vex brain
GPIO.setmode(GPIO.BCM)
FORWARD = 8					#The assigned integer is the 
BACKWARD = 18					#associated GPIO pin for the call
LEFT = 22
RIGHT = 23
GPIO.setup(FORWARD, GPIO.OUT)			
GPIO.setup(BACKWARD, GPIO.OUT)
GPIO.setup(LEFT, GPIO.OUT)
GPIO.setup(RIGHT, GPIO.OUT)

keep_running = True
red = True
iter = 0
avg = []					#will keep a running average of the center of the contours
size = 5					#length of list average

def displayRedOrBlue(dev, data, timestamp):
        GPIO.output(FORWARD, 1)
            
        global keep_running
        global red
        global iter
	
	#creating masks using HSV values to allow for accurate vison processing of the
	#red and blue flags and yellow ball
        #lower mask (0-10) for the red values at the lower end of the HSV spectrum
        lower_red = np.array([0,200,100])
        upper_red = np.array([10,255,255])
        img_hsv = cv2.cvtColor(frame_convert2.video_cv(data), cv2.COLOR_BGR2HSV)
        mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

        #upper mask (170-180) for the red values at the higher end of the HSV spectrum
        lower_red = np.array([170,200,100])
        upper_red = np.array([180,255,255])
        mask1 = cv2.inRange(img_hsv, lower_red, upper_red)
	
	#mask for blue values in the HSV spectrum 
        lower_blue = np.array([100, 100, 100])
        upper_blue = np.array([170, 255, 255])
        maskBlue = cv2.inRange(img_hsv, lower_blue, upper_blue)
	
	#mask for yellow values in the HSV spectrum
        lower_yellow = np.array([15, 0, 0])
        upper_yellow = np.array([60, 255, 255])
        maskYellow = cv2.inRange(img_hsv, lower_yellow, upper_yellow)
	
	#combine the masks to filter out everything that isn't red blue or yellow
        mask = mask0+mask1+maskBlue+maskYellow


        #mask = cv2.inRange(cv2.cvtColor(frame_convert2.video_cv(data), cv2.COLOR_BGR2HSV), (20, 0, 100), (55, 255, 255))
        #uses the constructed masks to process out general noise
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
	
	#finds the largest contour and sets that to be the one targeted by the kinect
	#cnts = imutils.grab_contours(cnts[0])
        largest = 0
        area = 0
        num = 0
        for x in range(len(cnts)):
            area = cv2.contourArea(cnts[x])
            if area > largest:
                largest = area
                num = x
	
	#gets the box of the largest rectangle and then finds the midpoint
	#in order to determine how the robot should move
        rect = cv2.minAreaRect(cnts[num])
        box = cv2.cv.BoxPoints(rect)
        box = np.int0(box)
	#updates running average
        avg.append(midpoint(box[0],box[2]))
        print(np.sum(avg)/size)
        if len(avg) >= size:
                avg.pop(0)
	#contours for circles were created but later not needed
        #cv2.drawContours(output,[box],0,(0,0,255),2) 
        #output = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        #circles = cv2.HoughCircles(output, cv2.HOUGH_GRADIENT, 1, 50, param1=50,param2=35,minRadius=20,maxRadius=50)
        #if circles is not None:
	# convert the (x, y) coordinates and radius of the circles to integers
        #    circles = np.round(circles[0, :]).astype("int")
	# loop over the (x, y) coordinates and radius of the circles
         #   for (x, y, r) in circles:
		# draw the circle in the output image, then draw a rectangle
		# corresponding to the center of the circle
         #       cv2.circle(output, (x, y), r, (0, 255, 0), 4)
         #       cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
 	
	#Shows contour image
        #cv2.imshow("Contours", output)
        #if cv2.waitKey(10) == 27:
         #   keep_running = False

def body(*args):
    if not keep_running:
        raise freenect.Kill
        
def midpoint(ptA, ptB):
    return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)

#displays vision processing
freenect.runloop(video=displayRedOrBlue, body=body)

print('Press ESC in window to stop')

