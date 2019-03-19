import cv2
import numpy as np
import math
import time
import os
import pathfinder as pf
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
	# Tell the CvSink to grab a frame from the camera and put it
	# in the source image.  If there is an error notify the output.
	ts,img = cap.read()
	#print("grabbed frame")
	screenHeight, screenWidth, _ = img.shape
	centerX = (screenWidth / 2) - .5
	centerY = (screenHeight / 2) - .5
	if time == 0:
		# Send the output the error.
		outputStream.notifyError(cap.stream.getError());
		# skip the rest of the current iteration
		continue
	pipeline.process(img)
	num_frames = num_frames + 1
	contours = pipeline.filter_contours_output
	cntsSorted = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
	if len(cntsSorted) >= 2:
		largestTwoContours = cntsSorted[:2]
		center = []
		x1,y1,w1,h1 = cv2.boundingRect(largestTwoContours[0])
		cv2.rectangle(img,(x1,y1),(x1+w1,y1+h1),(0,255,0),2)
		x2,y2,w2,h2 = cv2.boundingRect(largestTwoContours[1])
		cv2.rectangle(img,(x2,y2),(x2+w2,y2+h2),(0,255,0),2)
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
			center.append(cX)
			center.append(cY)
		center.append(int((center[0]+center[2])/2))
		center.append(int((center[1]+center[3])/2))
		calculatedAngle1 = calculateYaw(center[0],centerX,H_FOCAL_LENGTH)
		#print("angle1: " + str(calculatedAngle1))
		calculatedDistance1 = calculateDistance(CAMERA_HEIGHT,VISION_TARGET_HEIGHT,calculatePitch(center[1],centerY,V_FOCAL_LENGTH))
		#print("Distance1: " + str(calculatedDistance1) + " inches")
		calculatedAngle2 = calculateYaw(center[2],centerX,H_FOCAL_LENGTH)
		#print("angle2: " + str(calculatedAngle2))
		calculatedDistance2 = calculateDistance(CAMERA_HEIGHT,VISION_TARGET_HEIGHT,calculatePitch(center[3],centerY,V_FOCAL_LENGTH))
		#print("Distance2: " + str(calculatedDistance2) + " inches")
		calculatedAngleCenter = calculateYaw(center[4],centerX,H_FOCAL_LENGTH)
		calculatedDistanceCenter = calculateDistance(CAMERA_HEIGHT,VISION_TARGET_HEIGHT,calculatePitch(center[5],centerY,V_FOCAL_LENGTH))
		#print(calculatedAngleCenter)
		try:
			worldX1 = calculatedDistance1*math.sin(calculatedAngle1)
			worldY1 = calculatedDistance1*math.cos(calculatedAngle1)
		except:
			print("issues with contour 1")
		try:
			worldX2 = calculatedDistance2*math.sin(calculatedAngle2)
			worldY2 = calculatedDistance2*math.cos(calculatedAngle2)
		except:
			print("issues with contour 2")
		try:
			theta = math.atan2(worldY2-worldY1,worldX2-worldX1)
		except:
			print("Theta Calculation error")
		worldCenterX = calculatedDistanceCenter * math.sin(calculatedAngleCenter)
		worldCenterY = calculatedDistanceCenter * math.cos(calculatedAngleCenter)
		print("X distance from target: " + str(worldCenterX))
		print("Y distance from target: " + str(worldCenterY))
		print("angle to center: " + str(theta) + " radians")
		try:
			points = [pf.Waypoint(0, 0, math.radians(0)),pf.Waypoint(worldCenterX, worldCenterY, theta)]
			info, trajectory = pf.generate(points, pf.FIT_HERMITE_CUBIC, pf.SAMPLES_HIGH,
			dt=0.05, # 50ms
			max_velocity=24.0,
			max_acceleration=10.0,
			max_jerk=60.0
				)
			pf.serialize("newTrajectory.bin",trajectory)
		except:
			print("pathfinder exception")
		try:
			with FTP("roboRIO-2577-frc.local") as ftp:
				ftp.login()
				ftp.storbinary('STOR newTrajectory.bin',open("newTrajectory.bin",mode='rb'))
		except:
			print("ftp exception")
	else:
		print("didn't find two contours")
	cv2.imshow("image",img)
	if cv2.waitKey(25) & 0xFF == ord('q'):
		break
end = time.time()
print("fps: " + str(num_frames/(end - start)))
cap.release()
cv2.destroyAllWindows()
print("Done!")
