"""
FRC code to find the slope of a line and publish it to NetworkTables:
Intended platform: Jetson TX1 with a Microsoft Lifecam HD 3000

"""
import cv2
import networktables
from networktables import NetworkTables, NetworkTablesInstance
import numpy as np
import math
import time
import os
import threading


def connectionListener(connected, info):
    print(info, '; Connected=%s' % connected)
    with cond:
        notified[0] = True
        cond.notify()


def order_points(pts):
    ptsSorted = sorted(pts, key=lambda x: x[1], reverse=True)
    return ptsSorted


def largestContour(cnts):
    cntsSorted = sorted(cnts, key=lambda x: cv2.contourArea(x), reverse=True)
    return cntsSorted[0]


cap = cv2.VideoCapture(0)
# Configure networktables
cond = threading.Condition()
notified = [False]
NetworkTables.initialize(server='10.25.77.2')
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)
sd = NetworkTables.getTable("SmartDashboard")

# NetworkTables AutoUpdateValues allow for tuning of OpenCV pipeline values for easy testing
autoHueMin = sd.getAutoUpdateValue("hueMin", 0)
autoHueMax = sd.getAutoUpdateValue("hueMax", 255)
autoSatMin = sd.getAutoUpdateValue("satMin", 0)
autoSatMax = sd.getAutoUpdateValue("satMax", 255)
autoValMin = sd.getAutoUpdateValue("valMin", 75)
autoValMax = sd.getAutoUpdateValue("valMax", 255)
autoBlurRadius = sd.getAutoUpdateValue("blurRadius", 5)
mode = cv2.RETR_LIST
method = cv2.CHAIN_APPROX_SIMPLE
cap.set(3, 320)
cap.set(4, 240)
while True:
    ret, frame = cap.read()
    frame = cv2.flip(frame, -1)
    if ret:
        # convert BGR image to HSV for processing
        HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        threshOut = cv2.inRange(HSV, (autoHueMin.value, autoSatMin.value, autoValMin.value),
                                (autoHueMax.value, autoSatMax.value, autoValMax.value))
        ksize = int(2 * round(autoBlurRadius.value) + 1)
        blurOut = cv2.blur(threshOut, (ksize, ksize))
        im2, cnts, hierarchy = cv2.findContours(
            blurOut, mode=mode, method=method)
        contours = []
        for contour in cnts:
            if cv2.contourArea(contour) > 1000:
                contours.append(contour)
        cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)
        if len(contours) > 0:
            sd.putBoolean("lineDetected", True)
            largest = largestContour(contours)
            rect = cv2.minAreaRect(largest)
            cv2.imshow("blur", blurOut)
            points = cv2.boxPoints(rect)
            points = np.int0(points)
            points = order_points(points)
            upperMidpoint = [(points[2][0] + points[3][0]) / 2,
                             (points[2][1] + points[3][1]) / 2]
            lowerMidpoint = [(points[0][0] + points[1][0]) / 2,
                             (points[0][1] + points[1][1]) / 2]
            y = (upperMidpoint[1] - lowerMidpoint[1])
            x = (upperMidpoint[0] - lowerMidpoint[0])
            lineX = 160 - lowerMidpoint[0]
            print(lineX)
            sd.putNumber("lineX", lineX)
            slopeAngle = math.atan2(y, x)
            print(slopeAngle)
            sd.putNumber("slope", slopeAngle)
            cv2.line(img=frame, pt1=(int(upperMidpoint[0]), int(upperMidpoint[1])), pt2=(int(
                lowerMidpoint[0]), int(lowerMidpoint[1])), color=(255, 0, 0), thickness=4, lineType=8)
            cv2.imshow("centerLine", frame)
        else:
            sd.putBoolean("lineDetected", False)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        print("no camera")
        break
