import RPi.GPIO as GPIO            # import RPi.GPIO module  
from time import sleep

GPIO.setmode(GPIO.BCM)
FORWARD = 17
BACKWARD = 18
LEFT = 22
RIGHT = 23
GPIO.setup(FORWARD, GPIO.OUT)
GPIO.setup(BACKWARD, GPIO.OUT)
GPIO.setup(LEFT, GPIO.OUT)
GPIO.setup(RIGHT, GPIO.OUT)

try:  
    while True:  
<<<<<<< HEAD
        GPIO.output(24, 1)         # set GPIO24 to 1/GPIO.HIGH/True  
        sleep(0.5)                 # wait half a second  
        if GPIO.input(24):  
            print "LED just about to switch off"  
        GPIO.output(24, 0)         # set GPIO24 to 0/GPIO.LOW/False  
        sleep(0.5)                 # wait half a second  
        if not GPIO.input(24):  
            print "LED just about to switch on"  
=======
        GPIO.output(FORWARD, 1)         # set GPIO24 to 1/GPIO.HIGH/True  
        sleep(1)                 # wait half a second  
        if GPIO.input(FORWARD):  
            print "LED off"  
        GPIO.output(FORWARD, 0)         # set GPIO24 to 0/GPIO.LOW/False  
        sleep(3)                 # wait half a second  
        if not GPIO.input(FORWARD):  
            print "LED on"  
>>>>>>> 6f8c83bf7f4b88f4c7c557bd33aaf497888716d8
except KeyboardInterrupt:          # trap a CTRL+C keyboard interrupt  
    GPIO.cleanup()                 # resets all GPIO ports used by this program  

def reset():
    GPIO.output(FORWARD, 0)
    GPIO.output(BACKWARD, 0)
    GPIO.output(LEFT,0)
    GPIO.output(RIGHT, 0)

def go_forward():
    reset()
    GPIO.output(FORWARD, 1)

def go_backward():
    reset()
    GPIO.output(BACKWARD, 1)

def go_left():
    reset()
    GPIO.output(LEFT, 1)

def go_right():
    reset()
    GPIO.output(RIGHT, 1)
