import freenect
import cv2
import frame_convert2
import numpy as np
import argparse
import imutils
import shapedetector

cv2.namedWindow('Blue')
cv2.namedWindow('Red')
keep_running = True

#original test code for kinect
#used to test out showing video streams from the kinect
#first attempt at creating RGB masks
'''def display_depth(dev, data, timestamp):
    global keep_running
    #imgray = cv2.cvtColor(frame_convert2.pretty_depth_cv(data),cv2.COLOR_BGR2GRAY)
    #ret,thresh = cv2.threshold(imgray,127,255,0)
    #im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #cv2.drawContours(frame_convert2.pretty_depth_cv(data), contours, -1, (0,255,0), 3)
    
    image = frame_convert2.pretty_depth_cv(data)
    resized = imutils.resize(image, width=300)
    ratio = image.shape[0] / float(resized.shape[0])
 
    # convert the resized image to grayscale, blur it slightly,
    # and threshold it
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
 
    # find contours in the thresholded image and initialize the
    # shape detector
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    sd = ShapeDetector()
    #cv2.imshow('Depth', frame_convert2.pretty_depth_cv(data))
    if cv2.waitKey(10) == 27:
        keep_running = False'''

#displayed the normal video stream from the kinect
def display_rgb(dev, data, timestamp):
    global keep_running
    cv2.imshow('RGB', frame_convert2.video_cv(data))
    if cv2.waitKey(10) == 27:
        keep_running = False

#used to terminate program
def body(*args):
    if not keep_running:
        raise freenect.Kill

#inital code for shape detection that solely finds contours of the depth image
def findCountours():
    cnts = cv2.findContours(frame_convert2.pretty_depth_cv(data), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

#code for red/blue filter
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help = "path to the image")
args = vars(ap.parse_args())
#[greenVal, blueVal, redVal]
boundaries = [[0, 0, 30], [60, 30, 255], #red bound
	      [10, 10, 0], [180, 255, 80]] #blue bound

def display_red(dev, data, timestamp):
	global keep_running
	lower = np.array(boundaries[0], dtype = "uint8")
	upper = np.array(boundaries[1], dtype = "uint8")

 
	# find the colors within the specified boundaries and apply
	# the mask
	mask = cv2.inRange(frame_convert2.video_cv(data), lower, upper) # filter for colors not in range
	output = cv2.bitwise_and(frame_convert2.video_cv(data), frame_convert2.video_cv(data), mask = mask)
	cv2.imshow('Red', output)
	if cv2.waitKey(10) == 27:
		keep_running = False

def display_blue(dev, data, timestamp):
	global keep_running
	lower = np.array(boundaries[2], dtype = "uint8")
	upper = np.array(boundaries[3], dtype = "uint8")
 
	# find the colors within the specified boundaries and apply
	# the mask
	mask = cv2.inRange(frame_convert2.video_cv(data), lower, upper)
	output = cv2.bitwise_and(frame_convert2.video_cv(data), frame_convert2.video_cv(data), mask = mask)
	cv2.imshow('Blue', output)
	if cv2.waitKey(10) == 27:
		keep_running = False


