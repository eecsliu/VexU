import cv2 as cv
import numpy as np
# import RPi.GPIO as GPIO            # import RPi.GPIO module
import sys
import time

avg = []
size = 3 #5 gives a more smooth and accurate reading but lags slightly behind object
cap = cv.VideoCapture(0)


if __name__ == "__main__":
    start = time.time()
    frames = 0
    try:
        while 1:
            # lower mask (0-10)
            ret, array = cap.read()
            if not ret:
                continue
            #if the image needs to be rotated:
            #h,w = array.shape[:2]
            #center = (w/2, h/2)
            #M = cv.getRotationMatrix2D(center, 180, 1.0)
            #array = cv.warpAffine(array, M, (w, h))
            
            #adjust so only the top half of the frame is processed
            frame = array

            #color masks
            img_hsv = cv.cvtColor(frame,cv.COLOR_BGR2HSV)
            lower_red = np.array([0,200,100])
            upper_red = np.array([10,255,255])
            mask0 = cv.inRange(img_hsv, lower_red, upper_red)
            # upper mask (170-180)
            lower_red = np.array([170,200,100])
            upper_red = np.array([180,255,255])
            mask1 = cv.inRange(img_hsv, lower_red, upper_red)
            #blue mask
            lower_blue = np.array([110, 50, 50])
            upper_blue = np.array([140, 255, 200])
            maskBlue = cv.inRange(img_hsv, lower_blue, upper_blue)
            #lower_yellow = np.array([15, 0, 0])
            #upper_yellow = np.array([60, 255, 255])
            #maskYellow = cv2.inRange(img_hsv, lower_yellow, upper_yellow)

            mask = mask0 +mask1#maskBlue#+maskYellow

            output = cv.bitwise_and(frame, frame, mask = mask)

            #grayscale for edge detection
            gray = cv.cvtColor(output, cv.COLOR_BGR2GRAY)
            gray = cv.GaussianBlur(gray, (7, 7), 0)
            #morph to help remove noise
            edged = cv.Canny(gray, 50, 100)
            edged = cv.morphologyEx(edged, cv.MORPH_CLOSE, None)

            #contour detection
            #opencv4
            #cnts, hier = cv.findContours(edged.copy(), cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
            #opencv3
            _, cnts, hier = cv.findContours(edged.copy(), cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
            frames += 1
            if cnts == None:
                continue

            #moving average for detection of center of contour
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
                    cv.circle(array, (int(cX), int(cY)), 7, (255, 255, 255), -1)
                #elif cX < 280:
                    #go_right()
                #cv.circle(array, (int(cX), int(cY)), 7, (255, 255, 255), -1)

            cv.imshow("video", array)
            # quit program when 'esc' key is pressed
            k = cv.waitKey(5) & 0xFF
            if k == 27:
                break
        print(frames/(time.time() - start))
        cap.release()
        cv.destroyAllWindows()
    except (KeyboardInterrupt):
        print(frames/(time.time() - start))
        cap.release()
        cv.destroyAllWindows()

