import numpy as np
import cv2
import RPi.GPIO as io

io.setwarnings(False)
io.setmode(io.BOARD)

#define driver pins
io.setup(3,io.OUT)#IN1
io.setup(5,io.OUT)#IN2
io.setup(8,io.OUT)#IN3
io.setup(10,io.OUT)#IN4

#define micro water pump pin 
io.setup(40,io.OUT)

#this function moves the robot forward
def forward():
    io.output(3,1)
    io.output(5,0)
    io.output(8,1)
    io.output(10,0)
    
#this function moves the robot backward
def backward():
    io.output(3,0)
    io.output(5,1)
    io.output(8,0)
    io.output(10,1)
    
#this function stops the robot
def stopfcn():
    io.output(3,0)
    io.output(5,0)
    io.output(8,0)
    io.output(10,0)
    
#this function moves the robot right
def right():
    io.output(3,1)
    io.output(5,0)
    io.output(8,0)
    io.output(10,0)
    
#this function moves the robot left
def left():
    io.output(3,0)
    io.output(5,0)
    io.output(8,1)
    io.output(10,0)


cap = cv2.VideoCapture(0)

while True:
    ret , frame = cap.read()

    blur = cv2.GaussianBlur(frame,(21,21),0)
    hsv = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)

    lower = [18 , 50 , 50]
    upper = [35 , 255 , 255]

    lower = np.array(lower,dtype="uint8")
    upper = np.array(upper,dtype="uint8")
    mask = cv2.inRange(hsv , lower , upper)

    output = cv2.bitwise_and(frame , hsv , mask = mask)
    no_red = cv2.countNonZero(mask)
    
    cv2.imshow("camera",frame)
    cv2.imshow("fire detector",output)
    
    #control the robot by computer keyboard
    #i control the robot from VNC viewer
    k = cv2.waitKey(1) & 0xFF
    if k == ord('w'):
        forward()
    elif k == ord('s'):
        backward()
    elif k == ord('d'):
        right()
    elif k == ord('a'):
        left()
    elif k == ord('b'):
        stopfcn()
    elif k == ord('t'):
        #turn on water pump
        io.output(40,1)
    elif k == ord('y'):
        #turn off water pump
        io.output(40,0)
    elif k == 27:
        break
    else:
        pass
    

cv2.destroyAllWindows()
cap.release()