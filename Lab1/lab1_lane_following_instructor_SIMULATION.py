''' lab1_lane_following.py

This walkthrough covers the topics of camera calibration,
feature detection, and lane following

'''


from pal.products.qcar import QCarCameras
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
streamInfo      = [3, "RGB"]
frameRate       = 30  
enableCameras   = [False, False, False, False]
enableCameras[streamInfo[0]] = True

#Setting Filter
steeringFilter = Filter().low_pass_first_order_variable(25, 0.033)
next(steeringFilter)
dt = 0.033

#Command Coefficients
STEERING_COEF = 0.5
THROTTLE_COEF = 0.1


# Initialize CSI cameras
frontCSI = QCarCameras(
			frameWidth  = imageWidth,
			frameHeight = imageHeight,
			frameRate   = frameRate,
			enableRight = enableCameras[0],
			enableBack  = enableCameras[1],
			enableLeft  = enableCameras[2],
			enableFront = enableCameras[3]
		)


## QCar and Gamepad Initialization
myCar = QCar()
gpad  = LogitechF710()



def control_from_gamepad(LB, RT, leftLateral, A):

	if LB == 1:
			if A == 1 :
				throttle = THROTTLE_COEF* (RT-0.5) #going backward
				steering = leftLateral * 0.5
			else:
				throttle = THROTTLE_COEF *-(RT-0.5) #going forward
				steering = leftLateral * 0.5
	else:
		throttle = 0
		steering  = 0

	command = np.array([throttle, steering])
	return command

touch = 0

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
## Main Loop
try:
	while True:
		start = time.time()
		# Capture RGB Image from CSI
		frontCSI.readAll()
		image = frontCSI.csi[3].imageData

		
		# Crop out a piece of the RGB to improve performance
		croppedRGB = image[524:674, 0:820]

        #---- 1 ISOLATE COLOURS WITH BINARY ----#
		hsvBuf = cv2.cvtColor(croppedRGB, cv2.COLOR_BGR2HSV)
		binaryImage = ImageProcessing.binary_thresholding(frame= hsvBuf,
													lowerBounds=np.array([10, 0, 0]),
													upperBounds=np.array([40, 255, 255]))
		

		
        #----------------------------------------#

		# Display the RGB (Original) as well as the Binary in 1/4th resolution for speed
		cv2.imshow('crop rgB image', cv2.resize(croppedRGB, (410, 205) ) )
		cv2.imshow('My Binary image', cv2.resize(binaryImage, (410, 75) ))

		


        #---- 3.1 CALCULATE STEERING CONTROL ----#
		
		slope, intercept = ImageProcessing.find_slope_intercept_from_binary(binary=binaryImage)

		rawSteering = 1.5*(slope - 0.3419) + (1/150)*(intercept+5)
		steering = steeringFilter.send((np.clip(rawSteering, -0.5, 0.5), dt))
		
		if steering > 2 or steering < -2:
			steeringFilter = Filter().low_pass_first_order_variable(25, 0.033)
			next(steeringFilter)
			touch = 1

		print(steering)
        #----------------------------------------#

		# Write steering to qcar
		new = gpad.read()
		QCarCommand = control_from_gamepad(gpad.buttonLeft, gpad.trigger, gpad.leftJoystickX, gpad.buttonA)
		
    
        #------ 3.2 INITIALIZE QCAR COMMAND ------#
		if gpad.buttonX == 1:
			if math.isnan(steering):
				QCarCommand[1] = 0
			else:
				QCarCommand[1] = steering
			QCarCommand[0] = QCarCommand[0]*np.cos(steering)
		#-----------------------------------------#


        #-------- 2.2 WRITE MOTOR COMMAND --------#
		LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])
		myCar.read_write_std(QCarCommand[0],QCarCommand[1],LEDs)
		#----------------------------------------#
		
		cv2.waitKey(1)
		end = time.time()
		dt = end - start



except KeyboardInterrupt:
	print("User interrupted!")
	print(touch)

finally:
	# Terminate camera and QCar
	frontCSI.terminate()
	myCar.terminate()
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

