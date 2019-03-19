import networktables
from ftplib import FTP
from networktables import NetworkTables, NetworkTablesInstance
import pathfinder as pf
import cscore
from cscore import CameraServer, VideoSource
import cv2
import numpy as np
import math
import time as t
import os
import threading
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
    distance = math.fabs(heightOfTargetFromCamera / math.tan(pitch))

    return distance
# Uses trig and focal length of camera to find yaw.
# Link to further explanation: https://docs.google.com/presentation/d/1ediRsI-oR3-kwawFJZ34_ZTlQS2SDBLjZasjzZ-eXbQ/pub?start=false&loop=false&slide=id.g12c083cffa_0_298
def calculateYaw(pixelX, centerX, hFocalLength):
    yaw = math.atan((pixelX - centerX) / hFocalLength)
    return yaw


# Link to further explanation: https://docs.google.com/presentation/d/1ediRsI-oR3-kwawFJZ34_ZTlQS2SDBLjZasjzZ-eXbQ/pub?start=false&loop=false&slide=id.g12c083cffa_0_298
def calculatePitch(pixelY, centerY, vFocalLength):
    pitch = math.atan((pixelY - centerY) / vFocalLength)
    # Just stopped working have to do this:
    pitch *= -1
    return pitch
#image size ratioed to 16:9
image_width = 320
image_height = 240
# start NetworkTables
ntinst = NetworkTablesInstance.getDefault()
#Name of network table - this is how it communicates with robot. IMPORTANT
NetworkTables.initialize(server='10.25.77.1')
networkTable = NetworkTables.getTable('Vision')

#Setting up CS stuff
cs = CameraServer.getInstance()
cs.enableLogging()
print(cscore.getUsbCameraPath(1))
print("^^path")
#todo: find camera by path, lifecam doesn't have IDs so do by path
camera = cs.startAutomaticCapture(name="cam0", path='/dev/v4l/by-path/platform-tegra-xhci-usb-0:3.3:1.0-video-index0')
#camera = cs.startAutomaticCapture(dev=0)

camera.setResolution(320, 240)
print("set resolution successfully")
cap = cs.getVideo()
print("got cvSink")

#Send video back to Dashboard
outputStream = cs.putVideo("Name", 320, 240)

#More efficient?
img = np.zeros(shape=(240, 320, 3), dtype=np.uint8)
#Start grip pipeline for processing
pipeline = GripPipeline()

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
VISION_TARGET_HEIGHT = 24.573#lower one

start = t.time()
num_frames = 0
#cs.waitForever()
print("got to start of loop")
while True:
    	# Tell the CvSink to grab a frame from the camera and put it
	# in the source image.  If there is an error notify the output.
	time,img = cap.grabFrame(img)
	print("grabbed frame")
	screenHeight, screenWidth, _ = img.shape
	centerX = (screenWidth / 2) - .5
	centerY = (screenHeight / 2) - .5
	if time == 0:
		# Send the output the error.
		outputStream.notifyError(cap.stream.getError());
		# skip the rest of the current iteration
		continue
	pipeline.process(img)
	contours = pipeline.filter_contours_output
	cntsSorted = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
	if len(cntsSorted) >= 2:
		largestTwoContours = cntsSorted[:2]
		center = [0,0,0,0,0,0]
		x1,y1,w1,h1 = cv2.boundingRect(largestTwoContours[0])
		x2,y2,w2,h2 = cv2.boundingRect(largestTwoContours[1])
		i = 0
		for c in largestTwoContours:
			M = cv2.moments(c)
			try:
				cX = int(M["m10"] / M["m00"])
				cY = int(M["m01"] / M["m00"])
			except:
				print("divby0")
				cX = 0
				cY = 0
			cv2.circle(img, (cX, cY), 7, (255, 0, 0), -1)
			x,y,w,h= cv2.boundingRect(c)
			cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
			center[i] = cX
			center[i+1] = cY
			i=i+2
		center[4] = int((center[0]+center[2])/2)
		center[5] = int((center[1]+center[3])/2)
		calculatedAngle1 = calculateYaw(center[0],centerX,H_FOCAL_LENGTH)
		print("angle1: " + str(calculatedAngle1))
		calculatedDistance1 = calculateDistance(CAMERA_HEIGHT,VISION_TARGET_HEIGHT,calculatePitch(center[1],centerY,V_FOCAL_LENGTH))
		print("Distance1: " + str(calculatedDistance1) + " inches")
		calculatedAngle2 = calculateYaw(center[2],centerX,H_FOCAL_LENGTH)
		print("angle2: " + str(calculatedAngle2))
		calculatedDistance2 = calculateDistance(CAMERA_HEIGHT,VISION_TARGET_HEIGHT,calculatePitch(center[3],centerY,V_FOCAL_LENGTH))
		print("Distance2: " + str(calculatedDistance2) + " inches")
		calculatedAngleCenter = calculateYaw(center[4],centerX,H_FOCAL_LENGTH)
		calculatedDistanceCenter = calculateDistance(CAMERA_HEIGHT,VISION_TARGET_HEIGHT,calculatePitch(center[5],centerY,V_FOCAL_LENGTH))
		print(calculatedDistanceCenter)
		print(calculatedAngleCenter)
		try:
			worldX1 = calculatedDistance1*math.sin(calculatedAngle1)
			worldY1 = calculatedDistance1*math.cos(calculatedAngle1)
			worldX2 = calculatedDistance2*math.sin(calculatedAngle2)
			worldY2 = calculatedDistance2*math.cos(calculatedAngle2)
			theta = math.atan2(worldY2-worldY1,worldX2-worldX1)
			worldCenterX = calculatedDistanceCenter * math.sin(calculatedAngleCenter)
			worldCenterY = calculatedDistanceCenter * math.cos(calculatedAngleCenter)
			print(worldCenterX)
			print(worldCenterY)
			print(theta)
		except:
			print("random nonetype")
		try:
			points = [pf.Waypoint(0, 0, math.radians(0)),pf.Waypoint(worldCenterX, worldCenterY, theta)]
			info, trajectory = pf.generate(points, pf.FIT_HERMITE_CUBIC, pf.SAMPLES_HIGH,
			dt=0.05, # 50ms
			max_velocity=24.0,
			max_acceleration=10.0,
			max_jerk=60.0
				)
			pf.serialize("newTrajectory.bin",trajectory)
			with FTP("roboRIO-2577-frc.local") as ftp:
				ftp.login()
				ftp.storbinary('STOR newTrajectory.bin',open("newTrajectory.bin",mode='rb'))
		except:
			print("pathfinder exception")
#outputStream.putFrame(img)
num_frames +=1
end = t.time()
print("fps: " + str(num_frames/(end - start)))
