''' lab1_filtering.py

This walkthrough covers the image filtering
for edge detection

'''

from pal.products.qcar import QCarCameras,QCarRealSense, IS_PHYSICAL_QCAR
from hal.utilities.image_processing import ImageProcessing
import time
import numpy as np
import cv2
        


def main():
    try:
        
        # Set camera sepcifiactions
        imageSize       = [820,410] 
        streamInfo      = [3, "RGB"]
        frameRate       = 30  
        enableCameras   = [False, False, False, False]
        enableCameras[streamInfo[0]] = True

        # Initialize CSI cameras
        frontCSI = QCarCameras(
                    frameWidth  = imageSize[0],
                    frameHeight = imageSize[1],
                    frameRate   = frameRate,
                    enableRight = enableCameras[0],
                    enableBack  = enableCameras[1],
                    enableLeft  = enableCameras[2],
                    enableFront = enableCameras[3]
                )

        # Add intrinsic camera properties for calibration
        CSICamIntrinsics = np.array([
            [318.86, 0.00, 401.34],
            [0.00, 312.14, 201.50],
            [0.00, 0.00, 1.00]
        ], dtype=np.float32)

        CSIDistParam = np.array([
            [-0.9033,  1.5314, -0.0173, 0.0080, -1.1659]
            ], dtype=np.float32)


        camCalibTool = ImageProcessing()

        while True:

            # Take image
            frontCSI.readAll()
            endTime = time.time()
            image = frontCSI.csi[3].imageData

            imageShape = np.shape(image)

            # Perform Canny edge detection
            filteredImage = camCalibTool.do_canny(image)

            # Isolate lines
            linesImage, lines = camCalibTool.extract_lines(
                    filteredImage,
                    image
                )

            imageDisplayed = image

            # Use cv2 to display current image
            cv2.imshow("Final Image", imageDisplayed)


            cv2.waitKey(0.01)

    finally:
        frontCSI.terminate()
        input('Experiment complete. Press any key to exit...')


# MAIN

if __name__ == '__main__':
    print("Started")
    main()
