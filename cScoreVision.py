import networktables
from networktables import NetworkTables
from cscore import CameraServer
import cv2
import numpy as np
import math
import time as t
import os
import threading
from grip import GripPipeline

def main():
	#start network tables stuff, cscore __main__ handles most of initialization
	networktables.startClientTeam(2577)

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
	VISION_TARGET_HEIGHT = 22#fix, correct values on tx1

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


        # (optional) send some image back to the dashboard
        cs._publishTable.putNumber("angle", 4)
        outputStream.putFrame(img)
        num_frames +=1
	end = t.time()
	#print("fps: " + str(num_frames/(end - start)))