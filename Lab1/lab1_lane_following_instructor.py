''' lab1_lane_following.py

This walkthrough covers the topics of camera calibration,
feature detection, and lane following

'''

from pal.products.qcar import QCarCameras, IS_PHYSICAL_QCAR
from hal.utilities.image_processing import ImageProcessing
from pal.utilities.math import *
import time
import numpy as np
import cv2
import math

## Timing Parameters and methods
sampleRate = 60
sampleTime = 1/sampleRate
print('Sample Time: ', sampleTime)

#Setting Filter
steeringFilter = Filter().low_pass_first_order_variable(25, 0.033)
next(steeringFilter)
dt = 0.033

## Initialize the CSI cameras
myCam = Camera2D(cameraId=cameraID, frameWidth=imageWidth, frameHeight=imageHeight, frameRate=sampleRate)

## QCar Initialization
myCar = QCar()


class CameraCalibrator():


    def __init__(self):


        self.NUMBER_IMAGES = 15

        self.imageSize      = [820,410]
        self.chessboardDim  = [7,9]
        self.frameRate      = 30
        self.boxSize        = 0.02 #squares on calibration images ad 20mm
        self.sampleRate     = 1/self.frameRate
        self.calibFinished  = False
        self.CSICamIntrinsics = np.eye(3, 3, dtype=np.float32)
        self.CSIDistParam = np.ones((1,5),dtype= np.float32)

        self.camCalibTool = ImageProcessing()

        self.SimulationTime = 15
    

#PART 1
    def calibrate(self):

        savedImages = []
        imageCount = 0
        
        while self.calibFinished == False:
            startTime = time.time()

            self.frontCSI.readAll()
            endTime = time.time()

            image = self.frontCSI.csi[3].imageData
            computationTime = endtime-startTime
            sleepTime = self.sampleRate \
                - (computationTime % self.sampleRate)
            
            cv2.imshow("Camera Feed", image)


            #STUDENT CODE HERE
            for i in range(15):
                imageCount += 1
                print("saving Image #: ", imageCount)
                grayImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                savedImages.append(grayImage)

                if imageCount == self.NUMBER_IMAGES:
                    print("Implement calibration for CSI camera images: ")

                    print("Camera calibration for front csi")
                    self.CSICamIntrinsics = np.eye(3, 3, dtype=np.float32)
                    self.CSIDistParam = np.ones((1, 5), dtype=np.float32)

                    # Quanser's use of OpenCV for camera calibration
                    self.CSICamIntrinsics, self.CSIDistParam = \
                        self.camCalibTool.calibrate_camera(
                            savedImages,
                            self.chessboardDim,
                            self.boxSize)
                    

                    text = "CSI camera intrinsic matrix at resolution {} is:"
                    print(text.format(self.imageSize[:]))
                    print(self.CSICamIntrinsics)

                    text = ("CSI camera distortion parameters "
                        + "at resolution {} are: ")
                    print(text.format(self.imageSize[:]))
                    print(self.CSIDistParam)
                    self.calibFinished = True
        #STUDENT CODE HERE ^

        print("CSI Camera Calibrated")
        cv2.destroyAllWindows()
            
    
    def stop_cameras(self):
        # Stopping the image feed for both cameras
        self.frontCSI.terminate()


#PART 2
def line_detection(self):
    currentTime = 0
    t0 = time.time()
    
    while (currentTime < self.SimulationTime):
        LoopStartTime = time.time()
        currentTime = time.time()-t0

        self.frontCSI.readAll()
        endTime = time.time()
        image = self.frontCSI.csi[3].imageData
        computationTime = endTime-LoopStartTime
        sleepTime = self.sampleRate[0] \
            - (computationTime % self.sampleRate[0])
        cameraIntrinsics = self.CSICamIntrinsics
        cameraDistortion = self.CSIDistParam

        undistortedImage = image

     # -------------- STUDENT CODE HERE - filter image -------------------
        #Correct distortion
        imageShape = np.shape(image)
        undistortedImage = self.camCalibTool.undistort_img(
            image,
            cameraIntrinsics,
            cameraDistortion
        )


        print("Implement image filter on distortion corrected image... ")
        filteredImage = self.camCalibTool.do_canny(undistortedImage)
        # filteredImage = self.camCalibTool.doSobel(undistortedImage)

        #Extract features
        print("Extract line information from filtered image... ")
        linesImage, lines = image, []

        linesImage, lines = self.camCalibTool.extract_lines(
            filteredImage,
            undistortedImage
        )

        # Use cv2 to display current image
        cv2.imshow("Lines Image", imageDisplayed)
        msSleepTime = int(1000*sleepTime)
        if  msSleepTime <= 0:
            msSleepTime = 1

        cv2.waitKey(msSleepTime)


#PART 3
def laneFollowing():
    try:
        while True:
            start = time.time()
            # Capture RGB Image from CSI
            myCam.read()

            # Crop out a piece of the RGB to improve performance
            croppedRGB = myCam.imageData[524:674, 0:820]

            # Convert to HSV and then threshold it for yellow
            hsvBuf = cv2.cvtColor(croppedRGB, cv2.COLOR_BGR2HSV)

            binaryImage = ImageProcessing.binary_thresholding(frame= hsvBuf,
                                                        lowerBounds=np.array([10, 50, 100]),
                                                        upperBounds=np.array([45, 255, 255]))

            # Display the RGB (Original) as well as the Binary in 1/4th resolution for speed
            # cv2.imshow('My RGB image', cv2.resize(myCam.image_data, (410, 205) ) )
            # cv2.imshow('My RGB image', cropped_rgb )
            cv2.imshow('My Binary image', cv2.resize(binaryImage, (410, 75) ))

            # Find slope and intercept of linear fit from the binary image
            slope, intercept = ImageProcessing.find_slope_intercept_from_binary(binary=binaryImage)

            # steering from slope and intercept
            rawSteering = 1.5*(slope - 0.3419) + (1/150)*(intercept+5)
            steering = steeringFilter.send((np.clip(rawSteering, -0.5, 0.5), dt))

            # Write steering to qcar
            new = gpad.read()
            QCarCommand = control_from_gamepad(gpad.buttonLeft, gpad.trigger, gpad.leftLateralAxis, gpad.buttonA)
            print(QCarCommand)
            if gpad.buttonX == 1:
                if math.isnan(steering):
                    QCarCommand[1] = 0
                else:
                    QCarCommand[1] = steering
                QCarCommand[0] = QCarCommand[0]*np.cos(steering)

            LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])
            myCar.read_write_std(QCarCommand[0],QCarCommand[1],LEDs)
            cv2.waitKey(1)
            end = time.time()
            dt = end - start


    except KeyboardInterrupt:
	    print("User interrupted!")

    finally:
        # Terminate camera and QCar
        myCam.terminate()
        myCar.terminate()


def main():
    try:
        labPhase = 0
        cameraCalibrator = CameraCalibrator()

        input("Hold checkerboard in front of camera, then press enter to take distored image...")
        
        image = self.frontCSI.csi[3].imageData
        cv2.imshow("Camera Feed", image)
        print("Displaying distorted image")

        input("Hold checkerboard in front of camera, then press enter to calibrate camera...")

        if labPhase > 0:
            try:
                cameraCalibrator.calibrate()
                if cameraCalibrator.calibFinished == True:
                    print("calibration process done, stopping cameras...")
                    cameraCalibrator.stop_camera()
            
            except KeyboardInterrupt:
                cameraCalibrator.stop_camera()

        #if labPhase > 1
        #   run line detection code

        # if labPhase > 2
        # run lane following code

    except Exception as e:
        # Catching the exception and storing the error message
        error_message = str(e)
        print("An error occurred:", error_message)    
    finally:
        if not IS_PHYSICAL_QCAR:
            import qlabs_setup
            qlabs_setup.terminate()
        input('Experiment complete. Press any key to exit...')



if __name__ == '__main__':
    main()
