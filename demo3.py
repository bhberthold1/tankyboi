from Raspi_MotorHAT import Raspi_MotorHAT, Raspi_DCMotor

import time
import atexit
from PID import calcOBSPID, calcLinePID
import cv2
import numpy as np
from obstacleAvoid import avoidObs
from LineFollower import LineFollower

# Default values for contours
h = 0
w = 0


### Obstacle PID Constants ###
obs_sp = 80
dt = .1
old_err = 0
ObsPIDgains = [6,0,.00002]

### Line Follow PID Constants ###
line_sp = 0
old_line_err = 0
LinePIDgains = [1,5,1]

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

### Line Follower Class Init
l = LineFollower()
ind = 120


### Setup Webcam as video input ###
cap = cv2.VideoCapture(0)


### Main Loop, Will be broken into functions ###
while True:

	leftMotor.run(Raspi_MotorHAT.FORWARD)
	rightMotor.run(Raspi_MotorHAT.FORWARD)

	ret, frame = cap.read()

	xsize = int(frame.shape[1]/2)
	ysize = int(frame.shape[0]/4)

	px = np.array(frame[int(ysize), int(xsize)])

	percent_makeup = avoidObs(frame, px, w, h)


	masked = l.color_filter(frame, ind)
	roi_img = l.roi(masked)
	canny_img = l.canny(roi_img)
	hough_img = l.linedetect(canny_img)


#	print(l.differentialSlope)
#	print("Percent of Screen Taken: "+str(percent_makeup))



	obs_motor_speed = calcOBSPID(obs_sp, percent_makeup, old_err, ObsPIDgains, dt)
	obs_motor_speed = int(obs_motor_speed/3)
	old_err = old_err + (obs_sp - percent_makeup)

	if not np.isinf(l.differentialSlope) and not np.isnan(l.differentialSlope):
		line_motor_diff = int(calcLinePID(line_sp, l.differentialSlope, old_line_err, LinePIDgains, dt))
		print(line_motor_diff)

	else:
		line_motor_diff = int(0)

#	print(obs_motor_speed/3)


	leftMotor.setSpeed(70-line_motor_diff)
	rightMotor.setSpeed(70+line_motor_diff)

	time.sleep(dt)
