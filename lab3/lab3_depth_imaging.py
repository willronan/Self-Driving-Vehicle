'''lab3_depth_imaging.py

This walkthrough is designed to practice interfacing 
with the intelrealsense camera and test out its different modes

'''
import time
import cv2
from pal.products.qcar import QCarRealSense, IS_PHYSICAL_QCAR

if not IS_PHYSICAL_QCAR:
    import qlabs_setup
    qlabs_setup.setup()


#Initial Setup
runTime = 30.0 # seconds
max_distance = 2 # meters (for depth camera)

with QCarRealSense('''              ''') as myCam:
    t0 = time.time()
    while time.time() - t0 < runTime:


        #------ 1 TEST REALSENSE CAMERA -------#











        #----------------------------------------#

        cv2.waitKey(100)
