import cv2
import numpy as np

def LineFollower(frame):
	cx = 0
	area = 0
	gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)#convert each frame to grayscale.
	blur=cv2.GaussianBlur(gray,(5,5),0)#blur the grayscale image
	ret,th1 = cv2.threshold(blur,35,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)#using threshold remove noise
	ret1,th2 = cv2.threshold(th1,127,255,cv2.THRESH_BINARY_INV)# invert the pixels of the image frame
	_, contours, hierarchy = cv2.findContours(th2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) #find the contours
	#	 cv2.drawContours(frame,contours,-1,(0,255,0),3)
	#	 cv2.imshow('frame',frame) #show video
	for cnt in contours:
		if cnt is not None:
			area = cv2.contourArea(cnt) # find the area of contour
		if area>=1000:
			# find moment and centroid
			M = cv2.moments(cnt)
			cx = int(M['m10']/M['m00'])

	return cx
