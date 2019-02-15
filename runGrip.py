import cv2
import numpy as np
import math
import datetime
import os
import threading
from networktables import NetworkTables
from grip import GripPipeline
def twoLargest(contours):
	ret = []
	for c in contours:
		#M = cv2.moments(c)
		if len(ret) <w 2:
			ret.append(c)
		else:
			for compare in ret:
				#N = cv2.moments(compare)
				if cv2.contourArea(c) >= cv2.contourArea(compare):
					ret[ret.index(compare)] = c
					break
	return ret
now = datetime.datetime.now()
print("Vision Log for " + str(now.day) + "/" + str(now.month) + "/" + str(now.year) + "  ~    " + str(now.hour) + ":" + str(now.minute) +":" + str(now.second))
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

CAMERA_RESOLUTION = [310.0, 220.0] # 320 x 240
CAMERA_FOV_DEGREES = [52.45, 38.89] # 50.7, 39.9

# Insert your processing code here
cap = cv2.VideoCapture(0)
#cap.set(3,320)
#cap.set(4,240)
while streamRunning:
	# Capture frame-by-frame
	ret, frame = cap.read()
	height, width = frame.shape[:2] #640x480
	# Our operations on the frame come here
	# Display the resulting frame
	pipeline.process(frame)
	#print("found " + str(len(pipeline.filter_contours_output)) + " contours")
	largestTwoContours = twoLargest(pipeline.filter_contours_output)
	for c in largestTwoContours:
		M = cv2.moments(c)
		cX = int(M["m10"] / M["m00"])
		cY = int(M["m01"] / M["m00"])
		cv2.circle(frame, (cX, cY), 7, (255, 0, 0), -1)
	cv2.imshow('frame',frame)
	#sd.putNumber("aTarget",calculatedAngle)
	#sd.putNumber("dTarget",calculatedDistance)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
cap.release()
cv2.destroyAllWindows()
print("Done!")
