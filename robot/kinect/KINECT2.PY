import freenect
import cv2
import frame_convert2
import numpy as np
import shapedetector

cv2.namedWindow('Red and Blue')
keep_running = True
red = True
iter = 0

boundaries = [[0, 0, 30], [60, 30, 255], #red
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
	if(iter%10==0):
		red = not red

	mask = cv2.inRange(frame_convert2.video_cv(data), lower, upper)
	output = cv2.bitwise_and(frame_convert2.video_cv(data), frame_convert2.video_cv(data), mask = mask)
	cv2.imshow('Red and Blue', output)
	if cv2.waitKey(10) == 27:
		keep_running = False

def body(*args):
    if not keep_running:
        raise freenect.Kill

freenect.runloop(video=displayRedOrBlue, body=body)

print('Press ESC in window to stop')