from __future__ import print_function
import cv2
from collections import deque
import numpy as np
import argparse
import time
from time import sleep
import os
import sys
import RPi.GPIO as GPIO



#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#Pin allocation to pan and tilt servo
panServo = 17 
TiltServo = 27 


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(TiltServo, GPIO.OUT) # white => TILT
GPIO.setup(panServo, GPIO.OUT) # gray ==> PAN


#Position of pan and tilt servo
panPos = 1250
tiltPos = 1250

#Distance of how pan and tilt servo motor will move 
minMov = 30
maxMov = 100

num_frames=20

# Minimum required radius of enclosing circle of contour
MIN_RADIUS = 2


# Initialize angle servos at 90-90 position
global panAngle
panAngle = 90
global tiltAngle
tiltAngle =90


frameSize= (640,420)

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

# argument parsing
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the file")
ap.add_argument("-b", "--buffer", type=int, default=128, help="max buffer size")
args = vars(ap.parse_args())
pts = deque(maxlen=args["buffer"])  # g1

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
def nothing(x):
    pass
cv2.namedWindow("Trackbars")
cv2.createTrackbar("H_MIN", "Trackbars", 0, 179, nothing)
cv2.createTrackbar("S_MIN", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("V_MIN", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("H_MAX", "Trackbars", 179, 179, nothing)
cv2.createTrackbar("S_MAX", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("V_MAX", "Trackbars", 255, 255, nothing)


	
#Tracked object x and y axis coordinate 
def mapObjectPosition (x, y):
    print ("[INFO] Object Center coordinates at X0 = {0} and Y0 =  {1}".format(x, y))

#position servos 
  
def Servo_position(servo, angle):
	assert angle >=30 and angle <= 150
	pwm = GPIO.PWM(servo, 50)
	pwm.start(8)
	dutyCycle = angle / 18. + 3.
	pwm.ChangeDutyCycle(dutyCycle)
	sleep(0.3)
	pwm.stop()
	
	print("[INFO] Positioning servo at GPIO {0} to {1} degrees\n".format(servo, angle))

if __name__ == '__main__':
	import sys
	if len(sys.argv) == 1:
		Servo_position(panServo, 65) #position correction instead 90 degreees
		Servo_position(TiltServo, 80) #position correction instead 90 degreees
	else:
		Servo_position(panServo, int(sys.argv[1])) # 30 ==> 90 (middle point) ==> 150
		Servo_position(TiltServo, int(sys.argv[2])) # 30 ==> 90 (middle point) ==> 150


def mapServoPosition(x,y):
	global panAngle
	global tiltAngle
	
	if (x< 220):
		panAngle +=10
		if panAngle > 140:
			panAngle = 140
			Servo_position(panServo, panAngle)
			
	if (x > 280):
		panAngle -=10
		if panAngle < 40:
			panAngle = 40
			Servo_position(panServo, panAngle)
			
	if (y < 160):
		tiltAngle +=10
		if tiltAngle > 140:
			tiltAngle = 140
		Servo_position(TiltServo, tiltAngle)
		
	if (y < 210):
		tiltAngle -=10
		if tiltAngle < 40:
			tiltAngle = 40
		Servo_position (TiltServo, tiltAngle)

	
#Tracked object radius 
def mapObjectSize(radius):
    print (" object size at radius ={0}". format(radius))
    
# positioning Pan/Tilt servos at initial position
Servo_position (panServo, panAngle)
Servo_position (TiltServo, tiltAngle)

cap = cv2.VideoCapture(0)

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

while True:
    ret, frame = cap.read()
    frame = cv2.flip(frame, -1) # Flip camera vertically
    frame1 =cv2.resize(frame,(frameSize)) # resize the frame1, 
    img_filter = cv2.GaussianBlur(frame1.copy(), (3, 3), 0)
    hsv = cv2.cvtColor(img_filter, cv2.COLOR_BGR2HSV)
    hsv1=cv2.resize(hsv,(frameSize)) # resize the frame, 
    
    h_min = cv2.getTrackbarPos("H_MIN", "Trackbars")
    s_min = cv2.getTrackbarPos("S_MIN", "Trackbars")
    v_min = cv2.getTrackbarPos("V_MIN", "Trackbars")
    h_max = cv2.getTrackbarPos("H_MAX", "Trackbars")
    s_max = cv2.getTrackbarPos("S_MAX", "Trackbars")
    v_max = cv2.getTrackbarPos("V_MAX", "Trackbars")
    
    lower_green = np.array([h_min, s_min, v_min])
    upper_green = np.array([h_max, s_max, v_max])
    
    img_binary = cv2.inRange(hsv1, lower_green, upper_green)
    img_binary = cv2.erode(img_binary, None, iterations=2)
    img_binary = cv2.dilate(img_binary, None, iterations=2)
    
    img_contours = img_binary.copy()
    contours, hierarchy = cv2.findContours(img_contours, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Find the largest contour and use it to compute the min enclosing circle
    center = None

    radius = 0
    flagGreen1 = 0
    
    if len(contours) > 0:
         c = max(contours, key=cv2.contourArea)
         ((x, y), radius) = cv2.minEnclosingCircle(c)
         M = cv2.moments(c)
         
    if M["m00"] > 0:
         center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
         
         if radius < MIN_RADIUS:
                center = None
         
    # Print out the location and size (radius) of the largest detected contour
    mapObjectPosition(int(x), int(y))
    mapObjectSize(int(radius))
    
    # position Servo at center of circle
    mapServoPosition(int(x), int(y))
  
          
         #Draw a green circle around the largest enclosed contour
    if center != None:
            cv2.circle(frame1, center, int(round(radius)), (0, 255, 0))
            cv2.circle(frame1, center,5,(0,0,255,),-1)
        
  
    cv2.imshow("result", frame1)
    cv2.imshow("mask", img_binary)
    cv2.imshow("frame", img_contours)

    key = cv2.waitKey(1)
    if key == 27:
       
        break
print("\n [INFO] Exiting Program and cleanup stuff \n")
Servo_position (panServo, 90)
Servo_position (TiltServo, 90)
cv2.destroyAllWindows()
cap.release()
GPIO.cleanup()







