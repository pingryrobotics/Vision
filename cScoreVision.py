import networktables
from networktables import NetworkTables, NetworkTablesInstance
from cscore import CameraServer, VideoSource
import cv2
import numpy as np
import math
import time as t
import os
import threading
from grip import GripPipeline
class WebcamVideoStream:
    def __init__(self, camera, cameraServer, frameWidth, frameHeight, name="WebcamVideoStream"):
        # initialize the video camera stream and read the first frame
        # from the stream

        #Automatically sets exposure to 0 to track tape
        self.webcam = camera
        self.webcam.setExposureManual(0)
        #Some booleans so that we don't keep setting exposure over and over to the same value
        self.autoExpose = False
        self.prevValue = self.autoExpose
        #Make a blank image to write on
        self.img = np.zeros(shape=(frameWidth, frameHeight, 3), dtype=np.uint8)
        #Gets the video
        self.stream = cameraServer.getVideo()
        (self.timestamp, self.img) = self.stream.grabFrame(self.img)

        # initialize the thread name
        self.name = name

        # initialize the variable used to indicate if the thread should
        # be stopped
        self.stopped = False

    def start(self):
        # start the thread to read frames from the video stream
        t = Thread(target=self.update, name=self.name, args=())
        t.daemon = True
        t.start()
        return self

    def update(self):
        # keep looping infinitely until the thread is stopped
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return
            #Boolean logic we don't keep setting exposure over and over to the same value
            if self.autoExpose:
                if(self.autoExpose != self.prevValue):
                    self.prevValue = self.autoExpose
                    self.webcam.setExposureAuto()
            else:
                if (self.autoExpose != self.prevValue):
                    self.prevValue = self.autoExpose
                    self.webcam.setExposureManual(0)
            #gets the image and timestamp from cameraserver
            (self.timestamp, self.img) = self.stream.grabFrame(self.img)

    def read(self):
        # return the frame most recently read
        return self.timestamp, self.img

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True
    def getError(self):
        return self.stream.getError()
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
    distance = math.fabs(heightOfTargetFromCamera / math.tan(math.radians(pitch)))

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
def main():
	# start NetworkTables
    ntinst = NetworkTablesInstance.getDefault()
    #Name of network table - this is how it communicates with robot. IMPORTANT
    networkTable = NetworkTables.getTable('ChickenVision')

	#Setting up CS stuff
	cs = CameraServer.getInstance()
    cs.enableLogging()

    #todo: find camera by path, lifecam doesn't have IDs so do by path
    camera = cs.startAutomaticCapture(name="cam0", path='/dev/v4l/by-path/some-path-here')
    camera.setResolution(320, 240)

    cap = WebcamVideoStream(camera,cs,image_width,image_height).start()

    #Send video back to Dashboard
    outputStream = cs.putVideo("Name", 320, 240)

    #More efficient?
    img = np.zeros(shape=(240, 320, 3), dtype=np.uint8)

    #Start grip pipeline for processing
    pipeline = GripPipeline()


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
	VISION_TARGET_HEIGHT = 24.573#lower one

	start = t.time()
	num_frames = 0
    while True:
    	# Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        time,img = cap.read()
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
        largestTwoContours = cntsSorted[2:]
        # (optional) send some image back to the dashboard
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
			cv2.circle(img, (cX, cY), 7, (255, 0, 0), -1)
			x,y,w,h= cv2.boundingRect(c)
			cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
			center[0] += cX
			center[1] += cY
		center[0] = int(center[0]/2)
		center[1] = int(center[1]/2)
		cv2.circle(img, (center[0], center[1]), 7, (0, 0, 255), -1)
		#calculatedAngle = math.atan((distFromCenterPx)/h) - math.pi/4
		calculatedAngle = calculateYaw(center[0],centerX,H_FOCAL_LENGTH)
		print("angle: " + str(calculatedAngle))
		calculatedDistance = calculateDistance(CAMERA_HEIGHT,VISION_TARGET_HEIGHT,calculatePitch(center[0],centerY,V_FOCAL_LENGTH))
		print("Distance: " + str(calculatedDistance) + " inches")
        cs._publishTable.putNumber("angle", )
        outputStream.putFrame(img)
        num_frames +=1
	end = t.time()
	#print("fps: " + str(num_frames/(end - start)))
