import cv2
import numpy as np

def avoidObs(frame, px, w, h):
	percent_makeup = 0
	### Computer vision boundaries ###
	# Looking for red and blue objects (obstacles) within this region of BGR values #
	bluebound = ([80, 40, 10], [150, 80, 30])
	redbound = ([0, 0, 50], [60, 80, 150])
	searchbound = ([],[])

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

		if w < 50 or h < 50:
			w = 0
			h = 0

		# draw the book contour (in green)
		#cv2.rectangle(output,(x,y),(x+w,y+h),(0,255,0),3)
		#cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),3)


	#cv2.rectangle(frame,(xsize,ysize),(xsize,ysize),(0,0,255),3)
	#cv2.rectangle(output,(xsize,ysize),(xsize,ysize),(0,0,255),3)

	if h and w:
		percent_makeup = (h*w)/(480*640)*100

	return percent_makeup
