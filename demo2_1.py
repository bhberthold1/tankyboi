from Raspi_MotorHAT import Raspi_MotorHAT, Raspi_DCMotor

import time
import atexit
from PID import calcOBSPID
import cv2
import numpy as np

f = open("demo2.csv",'w')
timecount = 0

### Computer vision boundaries ###
# Looking for red and blue objects (obstacles) within this region of BGR values #
bluebound = ([80,40,10],[150,80,30])
redbound = ([0,0,50],[60,80,200])

# Default values for contours
h = 0
w = 0


### Obstacle PID Constants ###
obs_sp = 50
dt = .1
old_err = 0
PIDgains = [3,.00001,.00000020]


### Motor Setup ###

# create a default object, no changes to I2C address or frequency
mh = Raspi_MotorHAT(addr=0x6f)

# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
	mh.getMotor(1).run(Raspi_MotorHAT.RELEASE)
	mh.getMotor(2).run(Raspi_MotorHAT.RELEASE)
	mh.getMotor(3).run(Raspi_MotorHAT.RELEASE)
	mh.getMotor(4).run(Raspi_MotorHAT.RELEASE)
	f.close()
	cap.release()

atexit.register(turnOffMotors)

################################# DC motor test!
leftMotor = mh.getMotor(1)
rightMotor = mh.getMotor(3)

leftMotor.run(Raspi_MotorHAT.RELEASE)
rightMotor.run(Raspi_MotorHAT.RELEASE)

leftMotor.run(Raspi_MotorHAT.FORWARD)
rightMotor.run(Raspi_MotorHAT.FORWARD)


### Setup Webcam as video input ###
cap = cv2.VideoCapture(0)



### Main Loop, Will be broken into functions ###
while True:

	h = 0
	w = 0

	#leftMotor.run(Raspi_MotorHAT.FORWARD)
	#rightMotor.run(Raspi_MotorHAT.FORWARD)

	ret, frame = cap.read()

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
#		print(lower,upper)
		# find the colors within the specified boundaries and apply
		# the mask
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

		if w < 150 or h < 150:
			w = 0
			h = 0

		# draw the book contour (in green)
		#cv2.rectangle(output,(x,y),(x+w,y+h),(0,255,0),3)
		#cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),3)


	#cv2.rectangle(frame,(xsize,ysize),(xsize,ysize),(0,0,255),3)
	#cv2.rectangle(output,(xsize,ysize),(xsize,ysize),(0,0,255),3)

	if h and w:
		percent_makeup = (h*w)/(480*640)*100
	else:
		percent_makeup = 0

	print("Percent of Screen Taken: "+str(percent_makeup))


	obsPIDvalue = calcOBSPID(obs_sp, percent_makeup, old_err, PIDgains, dt)
	obs_motor_speed = int(obsPIDvalue)

	old_err = old_err + (obs_sp - percent_makeup)

	print(obs_motor_speed)

	if obs_motor_speed < 0:
		leftMotor.run(Raspi_MotorHAT.BACKWARD)
		rightMotor.run(Raspi_MotorHAT.BACKWARD)
	else:
		leftMotor.run(Raspi_MotorHAT.FORWARD)
		rightMotor.run(Raspi_MotorHAT.FORWARD)


#	if abs(obs_motor_speed) < 30:
#		obs_motor_speed = np.sign(obs_motor_speed)*30


	leftMotor.setSpeed(abs(obs_motor_speed))
	rightMotor.setSpeed(abs(obs_motor_speed))

	f.write(str(timecount)+','+str(percent_makeup)+','+str(obsPIDvalue)+'\n')

	timecount = timecount + dt

#	time.sleep(dt)
