''' lab1_lane_following.py

This walkthrough covers the topics of camera calibration,
feature detection, and lane following

'''

from pal.products.qcar import QCarCameras,QCarRealSense, IS_PHYSICAL_QCAR
from hal.utilities.image_processing import ImageProcessing
import time
import numpy as np
import cv2


class ImageInterpretation():

    def __init__(self):

        # Camera calibration constants:
        self.NUMBER_IMAGES = 15

        # List of variables given by students
        self.imageSize      = [820,410]      # pixel width & height of images
        self.chessboardDim  = [7,9]          # dimenstions of the checkerboard calibration imagage
        self.frameRate      = 30             # frame rate for the camera
        self.boxSize        = 0.02           # width of the boxes on checkerboard image
        self.sampleRate     = 1/self.frameRate
        self.calibFinished  = False          # boolean to signify when calibration is complete
        streamInfo=[3, "RGB"]                # CSI camera 3 (front), in RGB mode

        # List of camera intrinsic properties :
        self.CSICamIntrinsics = np.eye(3, 3, dtype=np.float32)
        # CSI camera intrinsic matrix at resolution [820, 410] is:
        # [[318.86    0.00  401.34]
        #  [  0.00  312.14  201.50]
        #  [  0.00    0.00    1.00]]


        self.CSIDistParam = np.ones((1,5),dtype= np.float32)
        # CSI camera distorion paramters at resolution [820, 410] are:
        # [[-0.9033  1.5314 -0.0173 0.0080 -1.1659]]

        # Final Image streamed by CSI
        self.streaCSI = np.zeros((self.imageSize[0],self.imageSize[1]))

        # Information for interfacing with front CSI camera
        enableCameras = [False, False, False, False]
        enableCameras[streamInfo[0]] = True

        self.frontCSI = QCarCameras(
            frameWidth  = self.imageSize[0],
            frameHeight = self.imageSize[1],
            frameRate   = self.frameRate,
            enableRight = enableCameras[0],
            enableBack  = enableCameras[1],
            enableLeft  = enableCameras[2],
            enableFront = enableCameras[3]
        )


        # Initialize calibration tool:
        self.camCalibTool = ImageProcessing()

        self.SimulationTime = 15


    # function to calculate intrinsic lens parameters / distortion
    def camera_calibration(self):

        # saving images
        savedImages = []
        imageCount = 0

        while True:
            startTime = time.time()

            # Read RGB information for front csi

            self.frontCSI.readAll()
            endTime = time.time()
            image = self.frontCSI.csi[3].imageData
            computationTime = endTime-startTime
            print("computationtime =" + str(computationTime))
            sleepTime = self.sampleRate \
                - (computationTime % self.sampleRate)
            print("sleeptime = " + str(sleepTime))


            # Use cv2 to display current image
            cv2.imshow("Camera Feed", image)

            msSleepTime = int(1000 * sleepTime)
            if  msSleepTime <= 0:
                msSleepTime = 1
            if cv2.waitKey(msSleepTime) & 0xFF == ord('q'):
                imageCount +=1
                print("saving Image #: ", imageCount)
                grayImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                savedImages.append(grayImage)

                if imageCount == self.NUMBER_IMAGES:

                    # Initialize Matrices for lens properties
                    self.CSICamIntrinsics = np.eye(3, 3, dtype=np.float32)
                    self.CSIDistParam = np.ones((1, 5), dtype=np.float32)

                    #----- 1.1 PEFORM LENS CALCUATION -----#

                    self.CSICamIntrinsics, self.CSIDistParam = \
                        self.camCalibTool.calibrate_camera(
                            savedImages,
                            self.chessboardDim,
                            self.boxSize)
                    
                    #----------------------------------------#

                    # Printed output for students
                    text = "CSI camera intrinsic matrix at resolution {} is:"
                    print(text.format(self.imageSize[:]))
                    print(self.CSICamIntrinsics)

                    text = ("CSI camera distortion parameters "
                        + "at resolution {} are: ")
                    print(text.format(self.imageSize[:]))
                    print(self.CSIDistParam)
                    break

        print("Camera calibrated!")

        self.calibFinished = True
        cv2.destroyAllWindows()

    # Function to print (and later calibrate) image
    def printImage(self):

        currentTime = 0
        t0 = time.time()

        while (currentTime < self.SimulationTime):
            LoopStartTime = time.time()
            currentTime = time.time()-t0

            self.frontCSI.readAll()
            endTime = time.time()
            image = self.frontCSI.csi[3].imageData
            computationTime = endTime-LoopStartTime
            sleepTime = self.sampleRate \
                - (computationTime % self.sampleRate)
            cameraIntrinsics = self.CSICamIntrinsics
            cameraDistortion = self.CSIDistParam



            #--------- 2.1 CALIBRATE IMAGE ---------#

            undistortedImage = image
      
            imageShape = np.shape(image)
            undistortedImage = self.camCalibTool.undistort_img(
                image,
                cameraIntrinsics,
                cameraDistortion
            )
            #----------------------------------------#
            #----- 2.2 PERFORM CANNY FILTERATION -----#

            filteredImage = self.camCalibTool.do_canny(undistortedImage)

            #----------------------------------------#
            #---------- 2.3 Extract Lines ----------#

            linesImage, lines = self.camCalibTool.extract_lines(
                filteredImage,
                undistortedImage
            )
            
            #----------------------------------------#


            imageDisplayed = image

            #Uncomment for Step 2.1
            #mageDisplayed = undistortedImage

            #Uncomment for Step 2.2
            #imageDisplayed = filteredImage

            #Uncomment for Step 2.3
            imageDisplayed = linesImage
            

            # Use cv2 to display current image
            cv2.imshow("Final Image", imageDisplayed)
            msSleepTime = int(1000*sleepTime)
            if  msSleepTime <= 0:
                msSleepTime = 1

            cv2.waitKey(msSleepTime)

    def stop_cameras(self):
        # Stopping the image feed for both cameras
        print("stopping camera")
        #self.frontCSI.terminate()
        


def main():
    try:
        cameraInterfacingLab = ImageInterpretation()

        try:
            cameraInterfacingLab.camera_calibration()
            if cameraInterfacingLab.calibFinished:
                print("calibration process done, stopping cameras...")
                cameraInterfacingLab.stop_cameras()
        except KeyboardInterrupt:
            cameraInterfacingLab.stop_cameras()

        try:
            cameraInterfacingLab.printImage()
        except KeyboardInterrupt:
            cameraInterfacingLab.stop_cameras()


    finally:
        input('Experiment complete. Press any key to exit...')


# MAIN

if __name__ == '__main__':
    print("Started")
    main()
