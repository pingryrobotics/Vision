import cv2
import networktables
from networktables import NetworkTables, NetworkTablesInstance
import numpy as np
import math
import time
import os
import threading
def order_points(pts):
	ptsSorted = sorted(pts, key=lambda x: x[1], reverse=True)
	return ptsSorted
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
autoValMin = sd.getAutoUpdateValue("valMin",20)
autoValMax = sd.getAutoUpdateValue("valMax",255)
autoBlurRadius = sd.getAutoUpdateValue("blurRadius",2)
mode = cv2.RETR_LIST
method = cv2.CHAIN_APPROX_SIMPLE
while True:
	ret, frame = cap.read()
	cv2.imshow("frame",frame)
	if ret:
		#convert BGR image to HSV for processing
		HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

		threshOut = cv2.inRange(HSV, (autoHueMin.value, autoSatMin.value, autoValMin.value), (autoHueMax.value, autoSatMax.value, autoValMax.value))
		ksize = int(2 * round(autoBlurRadius.value) + 1)
		blurOut = cv2.blur(threshOut, (ksize,ksize))
		im2, contours, hierarchy = cv2.findContours(blurOut, mode=mode, method=method)
		cv2.drawContours(frame,contours,-1,(0,255,0),3)
		if len(contours) > 0:
			largest = largestContour(contours)
			rect = cv2.minAreaRect(largest)
			points = cv2.boxPoints(rect)
			points = np.int0(points)
			points = order_points(points)
			upperMidpoint = [int((points[2][0]+points[3][0])/2),int((points[2][1]+points[3][1])/2)]
			lowerMidpoint = [int((points[0][0]+points[1][0])/2),int((points[0][1]+points[1][1])/2)]
			cv2.line(img=frame,pt1=(upperMidpoint[0],upperMidpoint[1]),pt2=(lowerMidpoint[0],lowerMidpoint[1]),color=(255,0,0),thickness=4,lineType=8)
			cv2.imshow("centerLine",frame)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
	else:
		break
