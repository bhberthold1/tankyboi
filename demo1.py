#!/usr/bin/python
from Raspi_MotorHAT import Raspi_MotorHAT, Raspi_DCMotor

import time
import atexit

# create a default object, no changes to I2C address or frequency
mh = Raspi_MotorHAT(addr=0x6f)
lroffset = 0
# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
      mh.getMotor(1).run(Raspi_MotorHAT.RELEASE)
      mh.getMotor(2).run(Raspi_MotorHAT.RELEASE)
      mh.getMotor(3).run(Raspi_MotorHAT.RELEASE)
      mh.getMotor(4).run(Raspi_MotorHAT.RELEASE)

#atexit.register(turnOffMotors)

################################# DC motor test!
leftMotor = mh.getMotor(1)
rightMotor = mh.getMotor(2)

# set the speed to start, from 0 (off) to 255 (max speed)
leftMotor.setSpeed(100)
rightMotor.setSpeed(100)
# turn on motor
leftMotor.run(Raspi_MotorHAT.RELEASE)
rightMotor.run(Raspi_MotorHAT.RELEASE)

maxSpeed = 255

for i in range(0,1):
      print("Forward! ")
      leftMotor.run(Raspi_MotorHAT.FORWARD)
      rightMotor.run(Raspi_MotorHAT.FORWARD)

      print("\tSpeed up...")
      for i in range(maxSpeed):
            leftMotor.setSpeed(i)
            rightMotor.setSpeed(i-lroffset)
            time.sleep(0.001)

      print("\tSlow down...")
      for i in reversed(range(maxSpeed)):
            leftMotor.setSpeed(i)
            rightMotor.setSpeed(i-lroffset)
            time.sleep(0.01)

      print("Backward! ")
      leftMotor.run(Raspi_MotorHAT.BACKWARD)
      rightMotor.run(Raspi_MotorHAT.BACKWARD)

      print("\tSpeed up...")
      for i in range(maxSpeed):
            leftMotor.setSpeed(i)
            rightMotor.setSpeed(i-lroffset)
            time.sleep(0.01)

      print("\tSlow down...")
      for i in reversed(range(maxSpeed)):
            leftMotor.setSpeed(i)
            rightMotor.setSpeed(i-lroffset)
            time.sleep(0.01)


      print("Left!")
      leftMotor.run(Raspi_MotorHAT.BACKWARD)
      rightMotor.run(Raspi_MotorHAT.FORWARD)

      print("\tSpeed up...")
      for i in range(maxSpeed):
            leftMotor.setSpeed(i)
            rightMotor.setSpeed(i)
            time.sleep(0.01)

      print("\tSlow down...")
      for i in reversed(range(maxSpeed)):
            leftMotor.setSpeed(i)
            rightMotor.setSpeed(i)
            time.sleep(0.01)

      print("Right!")
      leftMotor.run(Raspi_MotorHAT.FORWARD)
      rightMotor.run(Raspi_MotorHAT.BACKWARD)

      print("\tSpeed up...")
      for i in range(maxSpeed):
            leftMotor.setSpeed(i)
            rightMotor.setSpeed(i)
            time.sleep(0.01)

      print("\tSlow down...")
      for i in reversed(range(maxSpeed)):
            leftMotor.setSpeed(i)
            rightMotor.setSpeed(i)
            time.sleep(0.01)

      print("Release")
      leftMotor.run(Raspi_MotorHAT.RELEASE)
      rightMotor.run(Raspi_MotorHAT.RELEASE)
      time.sleep(1.0)
