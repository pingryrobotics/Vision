import cv2
import numpy as np
import math
import time
import os
import threading
from networktables import NetworkTables
from grip import GripPipeline
def calculateDistance(heightOfCamera, heightOfTarget, pitch):
    heightOfTargetFromCamera = heightOfTarget - heightOfCamera

    # Uses trig and pitch to find distance to target
    '''
    d = distance
    h = height between camera and target
    a = angle = pitch

    tan a = h/d (opposite over adjacent)

    d = h / tan a

                         .
                        /|
                       / |
                      /  |h
                     /a  |
              camera -----
                       d
    '''
    try:
        distance = math.fabs(heightOfTargetFromCamera / math.tan(math.radians(pitch)))
    except:
        distance = None
    return distance
# Uses trig and focal length of camera to find yaw.
# Link to further explanation: https://docs.google.com/presentation/d/1ediRsI-oR3-kwawFJZ34_ZTlQS2SDBLjZasjzZ-eXbQ/pub?start=false&loop=false&slide=id.g12c083cffa_0_298
def calculateYaw(pixelX, centerX, hFocalLength):
    yaw = math.degrees(math.atan((pixelX - centerX) / hFocalLength))
    return yaw


# Link to further explanation: https://docs.google.com/presentation/d/1ediRsI-oR3-kwawFJZ34_ZTlQS2SDBLjZasjzZ-eXbQ/pub?start=false&loop=false&slide=id.g12c083cffa_0_298
def calculatePitch(pixelY, centerY, vFocalLength):
    pitch = math.degrees(math.atan((pixelY - centerY) / vFocalLength))
    # Just stopped working have to do this:
    pitch *= -1
    return round(pitch)
def twoLargest(contours):
	ret = []
	for c in contours:
		#M = cv2.moments(c)
		if len(ret) < 2:
			ret.append(c)
		else:
			for compare in ret:
				#N = cv2.moments(compare)
				if cv2.contourArea(c) >= cv2.contourArea(compare):
					ret[ret.index(compare)] = c
					break
	"""
	ret = []
	if(len(contours) == 0):
		return (None, None)
	if(len(contours) == 1):
		return (contours[0], None)
	ret = contours[:1]
	contours = contours[2:]
	cntsSorted = sorted(contours, key=lambda x: cv2.contourArea(x))
	for c in contours:
		ret.append(c)
		cntsSorted = sorted(contours, key=lambda x: cv2.contourArea(x))
		ret = ret[1:]"""
	return ret
print("OpenCV version: " + str(cv2.__version__))
print("Starting Vision...")

bytes = ''
version = int(cv2.__version__[:1])
streamRunning = True
pipeline = GripPipeline()

"""cond = threading.Condition()
notified = [False]

def connectionListener(connected, info):
    print(info, '; Connected=%s' % connected)
    with cond:
	notified[0] = True
	cond.notify()

NetworkTables.initialize(server='10.25.77.2')
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

with cond:
    print("Waiting")
    if not notified[0]:
	cond.wait()
sd = NetworkTables.getTable("SmartDashboard")"""

#Angles in radians

#image size ratioed to 16:9
image_width = 320
image_height = 240

#Lifecam 3000 from datasheet
#Datasheet: https://dl2jx7zfbtwvr.cloudfront.net/specsheets/WEBC1010.pdf
diagonalView = math.radians(68.5)

#16:9 aspect ratio
horizontalAspect = 16
verticalAspect = 9

#Reasons for using diagonal aspect is to calculate horizontal field of view.
diagonalAspect = math.hypot(horizontalAspect, verticalAspect)
#Calculations: http://vrguy.blogspot.com/2013/04/converting-diagonal-field-of-view-and.html
horizontalView = math.atan(math.tan(diagonalView/2) * (horizontalAspect / diagonalAspect)) * 2
verticalView = math.atan(math.tan(diagonalView/2) * (verticalAspect / diagonalAspect)) * 2

#Focal Length calculations: https://docs.google.com/presentation/d/1ediRsI-oR3-kwawFJZ34_ZTlQS2SDBLjZasjzZ-eXbQ/pub?start=false&loop=false&slide=id.g12c083cffa_0_165
H_FOCAL_LENGTH = image_width / (2*math.tan((horizontalView/2)))
V_FOCAL_LENGTH = image_height / (2*math.tan((verticalView/2)))
CAMERA_HEIGHT = 18#inches
VISION_TARGET_HEIGHT = 24.573#inches

# Insert your processing code here
try:
	cap = cv2.VideoCapture(0)
except:
	cap = cv2.VideoCapture(1)
cap.set(3,image_width)
cap.set(4,image_height)
start = time.time()
num_frames = 0
while streamRunning:
	# Capture frame-by-frame
	ret, frame = cap.read()
	# Our operations on the frame come here
	# Display the resulting frame
	pipeline.process(frame)
	#print("found " + str(len(pipeline.filter_contours_output)) + " contours")
	largestTwoContours = twoLargest(pipeline.filter_contours_output)
	screenHeight, screenWidth, _ = frame.shape
	centerX = (screenWidth / 2) - .5
	centerY = (screenHeight / 2) - .5
	if len(largestTwoContours) == 2:
		center = [0,0]
		x1,y1,w1,h1 = cv2.boundingRect(largestTwoContours[0])
		x2,y2,w2,h2 = cv2.boundingRect(largestTwoContours[1])
		lenContourPx = int(x2) + int(w2) - int(x1)
		if lenContourPx < 0:
			lenContourPx = int(x1) + int(w1) - int(x2)
		for c in largestTwoContours:
			M = cv2.moments(c)
			try:
				cX = int(M["m10"] / M["m00"])
				cY = int(M["m01"] / M["m00"])
			except:
				print("divby0")
				cX = 0
				cY = 0
			cv2.circle(frame, (cX, cY), 7, (255, 0, 0), -1)
			x,y,w,h= cv2.boundingRect(c)
			cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
			center[0] += cX
			center[1] += cY
		center[0] = int(center[0]/2)
		center[1] = int(center[1]/2)
		cv2.circle(frame, (center[0], center[1]), 7, (0, 0, 255), -1)
		print("length of target in px: " + str(lenContourPx))
		distFromCenterPx = math.hypot(center[0]-centerX/2,center[1]-centerY/2) # Correct
		#print(distFromCenterPx)
		#calculatedAngle = math.atan((distFromCenterPx)/h) - math.pi/4
		calculatedAngle = calculateYaw(center[0],centerX,H_FOCAL_LENGTH)
		print("angle: " + str(calculatedAngle))
		calculatedDistance = calculateDistance(CAMERA_HEIGHT,VISION_TARGET_HEIGHT,calculatePitch(center[0],centerY,V_FOCAL_LENGTH))
		print("Distance: " + str(calculatedDistance) + " inches")
		#sd.putNumber("aTarget",calculatedAngle)
		#sd.putNumber("dTarget",calculatedDistance)
	cv2.imshow('frame',frame)
	cv2.imshow('hsv',pipeline.hsv_threshold_output)
	cv2.imshow('blur',pipeline.blur_output)
	num_frames += 1
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
end = time.time()
print("fps: " + str(num_frames/(end - start)))
cap.release()
cv2.destroyAllWindows()
print("Done!")
