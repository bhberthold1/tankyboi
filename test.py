import numpy as np
import cv2

def color_filter(image, ind):
    #convert to HLS to mask based on HLS
    hls = image #cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
    lower = np.array([0,0,0])
    upper = np.array([ind,ind,ind])
    yellower = np.array([10,0,90])
    yelupper = np.array([50,255,255])
    yellowmask = cv2.inRange(hls, yellower, yelupper)
    whitemask = cv2.inRange(hls, lower, upper)
    mask = cv2.bitwise_or(yellowmask, whitemask)
    masked = cv2.bitwise_and(image, image, mask = mask)
    return masked

def roi(img):
    x = int(img.shape[1])
    y = int(img.shape[0])
    shape = np.array([[int(0), int(y)], [int(x), int(y)], [int(0.55*x), int(0.6*y)], [int(0.45*x), int(0.6*y)]])
    #define a numpy array with the dimensions of img, but comprised of zeros
    mask = np.zeros_like(img)
    #Uses 3 channels or 1 channel for color depending on input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255
    #creates a polygon with the mask color
    cv2.fillPoly(mask, np.int32([shape]), ignore_mask_color)
    #returns the image only where the mask pixels are not zero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def grayscale(img):
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

def canny(img):
    return cv2.Canny(grayscale(img),50,120)


####
rightSlope, leftSlope, rightIntercept, leftIntercept = [],[],[],[]
differentialSlope = 0
def draw_lines(img, lines, thickness=5):
    global rightSlope, leftSlope, rightIntercept, leftIntercept, differentialSlope
    rightColor=[0,255,0]
    leftColor=[255,0,0]

    #this is used to filter out the outlying lines that can affect the average
    #We then use the slope we determined to find the y-intercept of the filtered lines by solving for b in y=mx+b
    for line in lines:
        for x1,y1,x2,y2 in line:
            slope = (y1-y2)/(x1-x2)
            if slope > 0.3 and x1 > img.shape[1]/2:
                yintercept = y2 - (slope*x2)
                rightSlope.append(slope)
                rightIntercept.append(yintercept)
            elif slope < -0.3 and x1 < img.shape[1]/2:
                yintercept = y2 - (slope*x2)
                leftSlope.append(slope)
                leftIntercept.append(yintercept)


    #We use slicing operators and np.mean() to find the averages of the 30 previous frames
    #This makes the lines more stable, and less likely to shift rapidly
    leftavgSlope = np.mean(leftSlope[-30:])
    leftavgIntercept = np.mean(leftIntercept[-30:])
    rightavgSlope = np.mean(rightSlope[-30:])
    rightavgIntercept = np.mean(rightIntercept[-30:])
    #Here we plot the lines and the shape of the lane using the average slope and intercepts

    differentialSlope = leftavgSlope+rightavgSlope

    try:
        left_line_x1 = int((0.65*img.shape[0] - leftavgIntercept)/leftavgSlope)
        left_line_x2 = int((img.shape[0] - leftavgIntercept)/leftavgSlope)
        right_line_x1 = int((0.65*img.shape[0] - rightavgIntercept)/rightavgSlope)
        right_line_x2 = int((img.shape[0] - rightavgIntercept)/rightavgSlope)
        pts = np.array([[left_line_x1, int(0.65*img.shape[0])],[left_line_x2, int(img.shape[0])],[right_line_x2, int(img.shape[0])],[right_line_x1, int(0.65*img.shape[0])]], np.int32)
        pts = pts.reshape((-1,1,2))
        cv2.fillPoly(img,[pts],(0,0,255))
        cv2.line(img, (left_line_x1, int(0.65*img.shape[0])), (left_line_x2, int(img.shape[0])), leftColor, 10)
        cv2.line(img, (right_line_x1, int(0.65*img.shape[0])), (right_line_x2, int(img.shape[0])), rightColor, 10)
#        cv2.line(img, ((left_line_x1+right_line_x1)/2,int(.65*img.shape[0])),((left_line_x2+right_line_x2)/2,int(img.shape[0])),[255,0,255],10)
    except ValueError:
    #I keep getting errors for some reason, so I put this here. Idk if the error still persists.
        pass

####


def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
    #`img` should be the output of a Canny transform.
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    draw_lines(line_img, lines)
    return line_img

def linedetect(img):
    return hough_lines(img, 1, np.pi/180, 1, 20,100)



cap = cv2.VideoCapture(0)
ind = 100

while(True):
    # Capture frame-by-frame
    ret, image = cap.read()
#    frame = cv2.resize(image, (480,240))

    height , width , layers =  image.shape
    new_h=int(height/2)
    new_w=int(width/2)
    image = cv2.resize(image, (new_w, new_h))


    try:
        masked = color_filter(image,ind)
        roi_img = roi(masked)
        canny_img = canny(roi_img)
        hough_img = linedetect(canny_img)

        print(differentialSlope)

        cv2.imshow('frame',image)
        cv2.imshow('canny',canny_img)
        cv2.imshow('hough',hough_img)
    except:
        cv2.imshow('frame',image)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
