#import the necessary modules
import freenect
import cv2 as cv
import numpy as np
import RPi.GPIO as GPIO            # import RPi.GPIO module
import sys
import time

avg = []
size = 3 #5 gives a more smooth and accurate reading but lags slightly behind object
ready = False
GPIO.setmode(GPIO.BCM)
READY = 17
READY2SHOOT = 18
LEFT = 22
RIGHT = 23
GPIO.setup(READY, GPIO.OUT)
GPIO.setup(READY2SHOOT, GPIO.OUT)
GPIO.setup(LEFT, GPIO.OUT)
GPIO.setup(RIGHT, GPIO.OUT)

def midpoint(ptA, ptB):
    return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)


def reset():
    GPIO.output(READY2SHOOT, 0)
    GPIO.output(LEFT,0)
    GPIO.output(RIGHT, 0)
    return

def go_ready():
    reset()
    GPIO.output(READY, 1)

def go_shoot():
    reset()
    GPIO.output(LEFT, 0)
    GPIO.output(RIGHT, 0)

def go_left():
    reset()
    GPIO.output(RIGHT, 1)

def go_right():
    reset()
    GPIO.output(LEFT, 1)

cap = cv.VideoCapture(0)

if __name__ == "__main__":
    start = time.time()
    frames = 0
    try:
        while 1:
            #get a frame from RGB camera
            # lower mask (0-10)
            ret, array = cap.read()
            if not ret:
                continue
            frame = array[:240, :, :]

            # lower mask (0-10)
            lower_red = np.array([0,200,100])
            upper_red = np.array([10,255,255])
            img_hsv = cv.cvtColor(frame,cv.COLOR_BGR2HSV)
            mask0 = cv.inRange(img_hsv, lower_red, upper_red)

            # upper mask (170-180)
            lower_red = np.array([170,200,100])
            upper_red = np.array([180,255,255])
            mask1 = cv.inRange(img_hsv, lower_red, upper_red)

            #blue mask
            lower_blue = np.array([110, 50, 50])
            upper_blue = np.array([140, 255, 200])
            maskBlue = cv.inRange(img_hsv, lower_blue, upper_blue)

            #yellow mask
            #lower_yellow = np.array([15, 0, 0])
            #upper_yellow = np.array([60, 255, 255])
            #maskYellow = cv2.inRange(img_hsv, lower_yellow, upper_yellow)

            #combine masks
            mask = mask0 +mask1#maskBlue#+maskYellow
            output = cv.bitwise_and(frame, frame, mask = mask)

            #grayscale for contour detection
            gray = cv.cvtColor(output, cv.COLOR_BGR2GRAY)
            gray = cv.GaussianBlur(gray, (7, 7), 0)
            # perform edge detection, then perform a dilation + erosion to
            # close gaps in between object edges
            edged = cv.Canny(gray, 50, 100)
            edged = cv.morphologyEx(edged, cv.MORPH_CLOSE, None)
            # find contours in the edge map
            #opencv4
            cnts, hier = cv.findContours(edged.copy(), cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
            #opencv3
            #_, cnts, hier = cv.findContours(edged.copy(), cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
            if cnts == None:
                continue
            if not ready:
                go_ready()
                ready = True

            if len(cnts) != 0:
                cnt = max(cnts, key = cv.contourArea)

                x, y, w, h = cv.boundingRect(cnt)
                avg.append((int(x+w/2), int(y+h/2)))
                if len(avg) > size:
                    avg.pop(0)
                length = len(avg)
                cX=0
                cY=0
                for each in avg:
                    cX += each[0]/length
                    cY += each[1]/length

                if cX > 360:
                    go_left()
                elif cX < 280:
                    go_right()
                else:
                    go_shoot()
                    #cv.circle(array, (int(cX), int(cY)), 7, (255, 255, 255), -1)

            #cv.rectangle(array,(x,y),(x+w,y+h),(0,255,0),2)
            #epsilon = 0.1*cv.arcLength(cnt, True)
            #approx = cv.approxPolyDP(cnt, epsilon, True)
            #cv.drawContours(array, cnt, 0, (0, 0, 255), 3)
            # upper mask (170-180)
            #display RGB image
            cv.imshow('video',array)
            #display depth image
            #cv2.imshow('Depth image',depth)

            # quit program when 'esc' key is pressed
            k = cv.waitKey(5) & 0xFF
            if k == 27:
                break
        print(frames/(time.time() - start))
        cap.release()
        reset()
        cv.destroyAllWindows()
    except (KeyboardInterrupt):
        print('this is the number of frames at the end: '+str(frames))
        print('time.time(): '+str(time.time())+', start: '+str(start)+', time.time() - start: '+str(time.time() - start))
        print('frames per second: '+str(frames/(time.time() - start)))
        cap.release()
        reset()
        cv.destroyAllWindows()
        sys.exit(0)
