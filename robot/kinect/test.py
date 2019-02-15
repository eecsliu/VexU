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

boundaries = [[0, 0, 20], [80, 30, 255], #red
	      [10, 10, 0], [180, 255, 80]] #blue

def displayRedOrBlue(dev, data, timestamp):
	global keep_running
	global red
	global iter
	if(red):
		lower = np.array(boundaries[0], dtype = "uint8")
		upper = np.array(boundaries[1], dtype = "uint8")
		iter+=1
	elif(not red):
		lower = np.array(boundaries[2], dtype = "uint8")
		upper = np.array(boundaries[3], dtype = "uint8")
		iter+=1

	mask = cv2.inRange(frame_convert2.video_cv(data), lower, upper)
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
	box = cv2.cv.BoxPoints(rect)
	box = np.int0(box)
	print(box)
	print(midpoint(box[0],box[2]))
	cv2.drawContours(output,[box],0,(0,0,255),2) 
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

