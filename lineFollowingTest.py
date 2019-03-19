import cv2
import networktables
from networktables import NetworkTables, NetworkTablesInstance
import numpy as np
import math
import time
import os
import threading
def largestContour(cnts):
	cntsSorted = sorted(cnts, key=lambda x: cv2.contourArea(x), reverse=True)
	return cntsSorted[0]
cap = cv2.VideoCapture(0)
sd = NetworkTables.getTable("SmartDashboard")
#allows for tuning of values for easy testing
autoHueMin = sd.getAutoUpdateValue("hueMin",0)
autoHueMax = sd.getAutoUpdateValue("hueMax",255)
autoSatMin = sd.getAutoUpdateValue("satMin",0)
autoSatMax = sd.getAutoUpdateValue("satMax",255)
autoValMin = sd.getAutoUpdateValue("valMin",0)
autoValMax = sd.getAutoUpdateValue("valMax",255)
autoBlurRadius = sd.getAutoUpdateValue("blurRadius",1)
mode = cv2.RETR_LIST
method = cv2.CHAIN_APPROX_SIMPLE
while True:
	ret, frame = cap.read()
	if ret:
		#convert BGR image to HSV for processing
		HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

		threshOut = cv2.inRange(HSV, (autoHueMin.value, autoSatMin.value, autoValMin.value), (autoHueMax.value, autoSatMax.value, autoValMax.value))

		ksize = int(2 * round(autoBlurRadius.value) + 1)
		blurOut = cv2.blur(threshOut, (ksize,ksize))


		im2, contours, hierarchy = cv2.findContours(blurOut, mode=mode, method=method)

		largestContour = largestContour(contours)
		rect = cv2.minAreaRect(largestContour)
		points = cv2.boxPoints(rect)
		points = np.int0(points)
		print("point0X" + points[0].x)
		print("point0Y" + points[0].y)
		cv2.circle(frame, (points[0].x, points[0].y), 7, (255, 0, 0), -1)
		cv2.imshow("frame",frame)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
	else:
		break