from Raspi_MotorHAT import Raspi_MotorHAT, Raspi_DCMotor

import math
import time
import atexit
from PID import calcOBSPID, calcLinePID
import cv2
import numpy as np
from obstacleAvoid import avoidObs
from LineFollower import LineFollower


MAX_SPEED = 140


# Default values for contours
h = 0
w = 0
bluebound = ([80,40,10],[150,80,30])
redbound = ([0,0,50],[60,80,200])

### Obstacle PID Constants ###
obs_sp = 25
dt = .05
old_err = 0
ObsPIDgains = [6,0.002,.0000004]

### Line Follow PID Constants ###
line_sp = 160
old_line_err = 0
#LinePIDgains = [.25,0,0]  ### WORKS WELL IN 294 LAB
LinePIDgains = [.5,0,.0000000025] # P = .5, D = .0025

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
	cx = 0
	percent_makeup = 0

	leftMotor.run(Raspi_MotorHAT.FORWARD)
	rightMotor.run(Raspi_MotorHAT.FORWARD)

	flag, frame = cap.read() #read the video in frames

	cframe = frame[60:180, 0:320]

	cx = LineFollower(cframe)

#############################################################
	xsize = int(frame.shape[1]/2)
	ysize = int(frame.shape[0]/4)

	px = np.array(frame[int(ysize), int(xsize)])

	print(px)
	if px[0] > bluebound[0][0] and px[0] < bluebound[1][0]:
		if px[1] > bluebound[0][1] and px[0] < bluebound[1][1]:
			if px[2] > bluebound[0][2] and px[0] < bluebound[1][2]:
				searchbound = bluebound
				print('Found something blue!')
			else:
				searchbound = ([0,0,0],[0,0,0])
		else:
			searchbound = ([0,0,0],[0,0,0])
	elif px[0] > redbound[0][0] and px[0] < redbound[1][0]:
		if px[1] > redbound[0][1] and px[0] < redbound[1][1]:
			if px[2] > redbound[0][2] and px[0] < redbound[1][2]:
				searchbound = redbound
				print('Found something red!')
			else:
				searchbound = ([0,0,0],[0,0,0])
		else:
			searchbound = ([0,0,0],[0,0,0])
	else:
		searchbound = ([0,0,0],[0,0,0])


	boundaries = [
		searchbound
	]

	for (lower, upper) in boundaries:
		# create NumPy arrays from the boundaries
		lower = np.array(lower, dtype = "uint8")
		upper = np.array(upper, dtype = "uint8")
#	   print(lower,upper)

		# find the colors within the specified boundaries and apply the mask
		mask = cv2.inRange(frame, lower, upper)

		output = cv2.bitwise_and(frame, frame, mask = mask)

	ret,thresh = cv2.threshold(mask, 0, 255, 0)
	im2,contours,hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

	if len(contours) != 0:
		# draw in blue the contours that were founded
		cv2.drawContours(output, contours, -1, 255, 3)

		#find the biggest area
		c = max(contours, key = cv2.contourArea)

		x,y,w,h = cv2.boundingRect(c)

		if w < 50 or h < 50:
			w = 0
			h = 0

	if h and w:
		percent_makeup = (h*w)/(480*640)*100
	else:
		percent_makeup = 0




##########################################################################################

	obs_motor_speed = int(calcOBSPID(obs_sp, percent_makeup, old_err, ObsPIDgains, dt))

	if obs_motor_speed > MAX_SPEED:
		obs_motor_speed = MAX_SPEED

	print("% makeup: "+str(percent_makeup)+" PID Motor Speed: "+str(obs_motor_speed))


	if not np.isinf(cx) and not np.isnan(cx):
		line_motor_diff = int(calcLinePID(line_sp, cx, old_line_err, LinePIDgains, dt))
		print("cx:"+str(cx)+" PID:"+str(line_motor_diff)+"\n")
		leftMotor.setSpeed(obs_motor_speed-line_motor_diff)
		rightMotor.setSpeed(obs_motor_speed+line_motor_diff)
	else:
		leftMotor.setSpeed(0)
		rightMotor.setSpeed(0)

	old_err = old_err + (obs_sp - percent_makeup)
	old_line_err = old_line_err + (line_sp - cx)

#	print(obs_motor_speed/3)



	time.sleep(dt)

