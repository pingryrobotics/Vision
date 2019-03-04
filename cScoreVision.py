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
    camera = cs.startAutomaticCapture(name="cam0", path='/dev/v4l/by-id/some-path-here')
    camera.setResolution(320, 240)

    #Start getting video feed for CV
    cvSink = cs.getVideo()

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
	VISION_TARGET_HEIGHT = 24.573#fix, correct values on tx1

	start = t.time()
	num_frames = 0
    while True:
    	# Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        time, img = cvSink.grabFrame(img)
        screenHeight, screenWidth, _ = img.shape
		centerX = (screenWidth / 2) - .5
		centerY = (screenHeight / 2) - .5
        if time == 0:
            # Send the output the error.
            outputStream.notifyError(cvSink.getError());
            # skip the rest of the current iteration
            continue
       	pipeline.process(img)
		contours = pipeline.filter_contours_output
		cntsSorted = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)

        # (optional) send some image back to the dashboard
        cs._publishTable.putNumber("angle", 4)
        outputStream.putFrame(img)
        num_frames +=1
	end = t.time()
	#print("fps: " + str(num_frames/(end - start)))
