import freenect
import cv2
import frame_convert2
import numpy as np
import argparse
import imutils

cv2.namedWindow('Depth')
cv2.namedWindow('RGB')
cv2.namedWindow('Red')
keep_running = True


def display_depth(dev, data, timestamp):
    global keep_running
    imgray = cv2.cvtColor(frame_convert2.pretty_depth_cv(data),cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(imgray,127,255,0)
    contours, heirarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frame_convert2.pretty_depth_cv(data),contours,-1,(0,255,0),3)
    cv2.imshow('Depth', frame_convert2.pretty_depth_cv(data))
    if cv2.waitKey(10) == 27:
        keep_running = False


def display_rgb(dev, data, timestamp):
    global keep_running
    cv2.imshow('RGB', frame_convert2.video_cv(data))
    if cv2.waitKey(10) == 27:
        keep_running = False


def body(*args):
    if not keep_running:
        raise freenect.Kill

#code for shape detection

def findCountours():
    cnts = cv2.findContours(frame_convert2.pretty_depth_cv(data), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

#code for red/blue filter

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help = "path to the image")
args = vars(ap.parse_args())

boundaries = [[0, 0, 0], [90, 255, 255]]

def display_red(dev, data, timestamp):
	global keep_running
	lower = np.array(boundaries[0], dtype = "uint8")
	upper = np.array(boundaries[1], dtype = "uint8")
 
	# find the colors within the specified boundaries and apply
	# the mask
	mask = cv2.inRange(frame_convert2.video_cv(data), lower, upper)
	output = cv2.bitwise_and(frame_convert2.video_cv(data), frame_convert2.video_cv(data), mask = mask)
	cv2.imshow('Red', output)
	if cv2.waitKey(10) == 27:
		keep_running = False

print('Press ESC in window to stop')
freenect.runloop(depth=display_depth,
                 #video=display_rgb,
                 video=display_red,
                 body=body)
