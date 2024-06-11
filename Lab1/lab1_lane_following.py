''' lab1_lane_following.py

This walkthrough covers the topics of camera calibration,
feature detection, and lane following

'''

from pal.utilities.vision import Camera2D
from pal.products.qcar import QCar
from pal.utilities.math import *
from pal.utilities.gamepad import LogitechF710
from hal.utilities.image_processing import ImageProcessing

import time
import numpy as np
import cv2
import math


## Timing Parameters and methods
sampleRate = 60
sampleTime = 1/sampleRate
print('Sample Time: ', sampleTime)

# Additional parameters
counter 	= 0
imageWidth  = 1640
imageHeight = 820
cameraID 	= '3'

#Setting Filter
steeringFilter = Filter().low_pass_first_order_variable(25, 0.033)
next(steeringFilter)
dt = 0.033


## Initialize the CSI cameras
myCam = Camera2D(cameraId=cameraID, frameWidth=imageWidth, frameHeight=imageHeight, frameRate=sampleRate)

## QCar and Gamepad Initialization
myCar = QCar()
gpad  = LogitechF710()



def control_from_gamepad(LB, RT, leftLateral, A):
    #----- 2A IMPLEMENT GAMEPAD CONTROLS -----#	






	command = np.array()
	return command
    #----------------------------------------#


# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
## Main Loop
try:
	while True:
		start = time.time()
		# Capture RGB Image from CSI
		myCam.read()
		
		# Crop out a piece of the RGB to improve performance
		croppedRGB = myCam.imageData[524:674, 0:820]

        #---- 1 ISOLATE COLOURS WITH BINARY ----#





        #----------------------------------------#

		# Display the RGB (Original) as well as the Binary in 1/4th resolution for speed
		cv2.imshow('My RGB image', cv2.resize(myCam.image_data, (410, 205) ) )
		# cv2.imshow('My RGB image', cropped_rgb )
		cv2.imshow('My Binary image', cv2.resize(binaryImage, (410, 75) ))


        #---- 3a CALCULATE STEERING CONTROL ----#





        #----------------------------------------#

		# Write steering to qcar
		new = gpad.read()
		QCarCommand = control_from_gamepad(gpad.buttonLeft, gpad.trigger, gpad.leftJoystickX, gpad.buttonA)
		
    
        #------ 2b,3b INITIALIZE QCAR COMMAND ------#






		#-----------------------------------------#


        #-------- 2c WRITE MOTOR COMMAND --------#




		#----------------------------------------#
		
		cv2.waitKey(1)
		end = time.time()
		dt = end - start



except KeyboardInterrupt:
	print("User interrupted!")

finally:
	# Terminate camera and QCar
	myCam.terminate()
	myCar.terminate()
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

