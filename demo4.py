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
FRAME_RATE = 30

# Default values for contours
h = 0
w = 0


### Obstacle PID Constants ###
obs_sp = 20
dt = 1/FRAME_RATE
old_err = 0
ObsPIDgains = [6,0.0004,.0000002]

### Line Follow PID Constants ###
line_sp = 160
old_line_err = 0
#LinePIDgains = [.25,0,0]  ### WORKS WELL IN 294 LAB
LinePIDgains = [.75,0,.000000025] # P = .5, D = .0025

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
cap.set(5,FRAME_RATE)  #set the frame rate

for i in range(0,2):
    flag, trash = cap.read() #starting unwanted null value

### Main Loop, Will be broken into functions ###
while True:
    cx = 0
    percent_makeup = 0

    leftMotor.run(Raspi_MotorHAT.FORWARD)
    rightMotor.run(Raspi_MotorHAT.FORWARD)

    flag, frame = cap.read() #read the video in frames

    frame = frame[60:180, 0:320]

    xsize = int(frame.shape[1]/2)
    ysize = int(frame.shape[0]/4)

    px = np.array(frame[int(ysize), int(xsize)])

    percent_makeup = avoidObs(frame, px, w, h)
    cx = LineFollower(frame)

    obs_motor_speed = int(calcOBSPID(obs_sp, percent_makeup, old_err, ObsPIDgains, dt))

    print("cx: "+str(cx)+"% makeup: "+str(percent_makeup)+" PID Motor Speed: "+str(obs_motor_speed))

    if obs_motor_speed > MAX_SPEED:
        obs_motor_speed = MAX_SPEED

    if not np.isinf(cx) and not np.isnan(cx) and cx < 50:
        line_motor_diff = int(calcLinePID(line_sp, cx, old_line_err, LinePIDgains, dt))
        print("cx:"+str(cx)+" PID:"+str(line_motor_diff)+"\n")
        leftMotor.setSpeed(obs_motor_speed-line_motor_diff)
        rightMotor.setSpeed(obs_motor_speed+line_motor_diff)
    else:
        leftMotor.setSpeed(0)
        rightMotor.setSpeed(0)

    old_err = old_err + (obs_sp - percent_makeup)
    old_line_err = old_line_err + (line_sp - cx)

#   print(obs_motor_speed/3)

    time.sleep(dt)


