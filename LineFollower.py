import cv2

def LineFollower(frame):
	### Zero the x-centroid and contour area ###
	cx = 0
	contArea = 0

	### Make the frame grayscale ###
	gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)#convert each frame to grayscale.

	### Blur the image ###
	blur=cv2.GaussianBlur(gray,(5,5),0)#blur the grayscale image

	### Remove noise with a threshold ###
	_, removedNoise = cv2.threshold(blur,35,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)#using threshold remove noise

	### Invert image pixels ###
	_, inverted = cv2.threshold(removedNoise,127,255,cv2.THRESH_BINARY_INV)# invert the pixels of the image frame

	### Find the contours of the image ###
	_, contours, _ = cv2.findContours(inverted,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) #find the contours

	### For each of the contours, iterate through the list ###
	for c in contours:

		### If the contours are populated: ###
		if c is not None:

			### Find the area of each contours ###
			contArea = cv2.contourArea(c)

		### Threshold the area: 1000 sq. pixels ###
		if contArea >= 1000:

			### Find the moments of the large contour ###
			M = cv2.moments(c)

			### Calculate the x-centroid ###
			cx = int(M['m10']/M['m00'])

	### Return the position of the x-centroid ###
	return cx
