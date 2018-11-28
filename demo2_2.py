from Raspi_MotorHAT import Raspi_MotorHAT, Raspi_DCMotor

import time
import atexit
from PID import calcOBSPID
import cv2
import numpy as np
from obstacleAvoid import avoidObs

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

	percent_makeup = avoidObs(frame, px, w, h)

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
