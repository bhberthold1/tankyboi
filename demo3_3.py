from Raspi_MotorHAT import Raspi_MotorHAT, Raspi_DCMotor

import math
import time
import atexit
from PID import calcOBSPID, calcLinePID
import cv2
import numpy as np
from obstacleAvoid import avoidObs
from LineFollower import LineFollower


f = open("demo3_3.csv",'w')

MAX_SPEED = 140


# Default values for contours
h = 0
w = 0


### Obstacle PID Constants ###
obs_sp = 80
dt = .033
old_err = 0
ObsPIDgains = [6,0,.00002]

### Line Follow PID Constants ###
line_sp = 160
old_line_err = 0
#LinePIDgains = [.25,0,0]  ### WORKS WELL IN 294 LAB
LinePIDgains = [.75,0,0.000000025] # P = .5, D = .0025

### Motor Setup ###

# create a default object, no changes to I2C address or frequency
mh = Raspi_MotorHAT(addr=0x6f)

# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
	mh.getMotor(1).run(Raspi_MotorHAT.RELEASE)
	mh.getMotor(2).run(Raspi_MotorHAT.RELEASE)
	mh.getMotor(3).run(Raspi_MotorHAT.RELEASE)
	mh.getMotor(4).run(Raspi_MotorHAT.RELEASE)

atexit.register(turnOffMotors)

leftMotor = mh.getMotor(1)
rightMotor = mh.getMotor(3)

leftMotor.run(Raspi_MotorHAT.RELEASE)
rightMotor.run(Raspi_MotorHAT.RELEASE)


### Setup Webcam as video input ###
cap = cv2.VideoCapture(0)
cap.set(3,320.0) #set the size
cap.set(4,240.0)  #set the size
cap.set(5,15)  #set the frame rate

for i in range(0,2):
	flag, trash = cap.read() #starting unwanted null value

### Main Loop, Will be broken into functions ###
while True:
	# Set cx = 0 (centroid of masked tape)
	cx = 0

	# Set motor direction to FORWARD
	leftMotor.run(Raspi_MotorHAT.FORWARD)
	rightMotor.run(Raspi_MotorHAT.FORWARD)

	# Get one frame from the webcam
	flag, frame = cap.read()

	# Resize the frame
	frame = frame[60:180, 0:320]

	# Call the LineFollower function
	cx = LineFollower(frame)

	# Check if returned value is NaN or infinity
	if not np.isnan(cx) or not np.isinf(cx):
		line_motor_diff = int(calcLinePID(line_sp, cx, old_line_err, LinePIDgains, dt))
		print("cx:"+str(cx)+" PID:"+str(line_motor_diff))
		leftMotor.setSpeed(MAX_SPEED-line_motor_diff)
		rightMotor.setSpeed(MAX_SPEED+line_motor_diff)
		f.write(str(cx)+","+str(line_motor_diff)+"\n")
	else:
		leftMotor.setSpeed(0)
		rightMotor.setSpeed(0)
		#line_motor_diff = int(0)

	# Calculate total error
	old_line_err = old_line_err + (line_sp - cx)

	time.sleep(dt)

