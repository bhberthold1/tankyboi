from Raspi_MotorHAT import Raspi_MotorHAT, Raspi_DCMotor

import time
import atexit
from PID import calcOBSPID, calcLinePID
import cv2
import numpy as np
from obstacleAvoid import avoidObs
from LineFollower import LineFollower


### Open a file for data output ###
f = open("demo4_t_pm_lerr_ms_lm_rm_run8.csv",'w')

### Set CONST variables ###
MAX_SPEED = 120
FRAME_RATE = 30
TIMECOUNT = 0


### Obstacle PID Constants ###
obs_sp = 30
dt = 1/FRAME_RATE
old_err = 0
ObsPIDgains = [8.35,0.15,.0000008]


### Line Follow PID Constants ###
line_sp = 160
old_line_err = 0
LinePIDgains = [.85,0,.000000025]


### MotorHAT Initialization ###
mh = Raspi_MotorHAT(addr=0x6f)


### Create a function to turn off motors on program exit ###
def turnOffMotors():
	mh.getMotor(1).run(Raspi_MotorHAT.RELEASE)
	mh.getMotor(2).run(Raspi_MotorHAT.RELEASE)
	mh.getMotor(3).run(Raspi_MotorHAT.RELEASE)
	mh.getMotor(4).run(Raspi_MotorHAT.RELEASE)


### Register turnOffMotors to run at exit ###
atexit.register(turnOffMotors)


### Define variables for left and right motors ###
leftMotor = mh.getMotor(1)
rightMotor = mh.getMotor(3)


### Make sure the motors are off ###
leftMotor.run(Raspi_MotorHAT.RELEASE)
rightMotor.run(Raspi_MotorHAT.RELEASE)


### Default values for contours ###
h = 0
w = 0


### Setup Webcam as video input ###
cap = cv2.VideoCapture(0)
cap.set(3,320.0)					# Resize the image x-direction
cap.set(4,240.0)					# Resize the image y-direction
cap.set(5,FRAME_RATE)			   # Set the webcam frame rate


### Grab two frames to throw away ###
for i in range(0,2):
	flag, trash = cap.read()

### Main Loop, Will be broken into functions ###
while True:

	### Zero the x-centroid and percent_makeup variables every loop ###
	cx = 0
	percent_makeup = 0

	### Set the motor directions ###
	leftMotor.run(Raspi_MotorHAT.FORWARD)
	rightMotor.run(Raspi_MotorHAT.FORWARD)

	### Read one frame from the webcam ###
	flag, frame = cap.read()

	### Resize the frame to remove borders ###
	frame = frame[60:180, 0:320]

	### Find the size of the x- and y-direction ###
	xsize = int(frame.shape[1]/2)
	ysize = int(frame.shape[0]/4)

	### Find the BGR value of sample pixel ###
	px = np.array(frame[int(ysize), int(xsize)])

	### Enter the avoidObs function ###
	percent_makeup = avoidObs(frame, px, w, h)

	### Enter the LineFollower function ###
	cx = LineFollower(frame)

	### Calculate the PID value for motor speed ###
	obs_motor_speed = int(calcOBSPID(obs_sp, percent_makeup, old_err, ObsPIDgains, dt))

	### Print values to the terminal ###
	print("cx: "+str(cx)+"% makeup: "+str(percent_makeup)+" PID Motor Speed: "+str(obs_motor_speed))

	### Cap the motor speed ###
	if obs_motor_speed > MAX_SPEED:
		obs_motor_speed = MAX_SPEED

	### Turn the motors backwards if less than zero ###
	if obs_motor_speed < 0:
		leftMotor.run(Raspi_MotorHAT.BACKWARD)
		rightMotor.run(Raspi_MotorHAT.BACKWARD)
		obs_motor_speed = abs(obs_motor_speed)
	else:
		leftMotor.run(Raspi_MotorHAT.FORWARD)
		rightMotor.run(Raspi_MotorHAT.FORWARD)
 

	### Initialize the differential between motors ###
	line_motor_diff = 0

	### If the centroid is not NaN or Inf ###
	if not np.isinf(cx) and not np.isnan(cx):

		### Calculate the PID value for motor speed differential ###
		line_motor_diff = int(calcLinePID(line_sp, cx, old_line_err, LinePIDgains, dt))

		### Print values to the terminal ###
		print("cx:"+str(cx)+" PID:"+str(line_motor_diff)+"\n")

		### Set the left and right motor speed ###
		leftMotor.setSpeed(obs_motor_speed-line_motor_diff)
		rightMotor.setSpeed(obs_motor_speed+line_motor_diff)

	else:
		### Set the left and right motor speed ###
		leftMotor.setSpeed(0)
		rightMotor.setSpeed(0)

	### Calculate the running error ###
	old_err = old_err + (obs_sp - percent_makeup)
	old_line_err = old_line_err + (line_sp - cx)

	### Write data to file ###
	f.write(str(TIMECOUNT)+","+str(obs_sp - percent_makeup)+","+str(line_sp-cx)+","+str(obs_motor_speed)+","+str(obs_motor_speed-line_motor_diff)+","+str(obs_motor_speed+line_motor_diff)+"\n")

	### Update TIMECOUNT ###
	TIMECOUNT = TIMECOUNT + dt

	### Sleep for one frame ###
	time.sleep(dt)
