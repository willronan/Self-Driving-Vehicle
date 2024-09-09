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

with QCarRealSense(mode='RGB, Depth') as myCam:
    t0 = time.time()
    while time.time() - t0 < runTime:


        #------ 1 PEFORM LENS CALCUATION ------#
        
        myCam.read_depth()
        cv2.imshow('My Depth', myCam.imageBufferDepthPX/max_distance)


        #myCam.read_IR()

        #cv2.imshow('Left IR Camera', myCam.imageBufferIRLeft)
        #cv2.imshow('Right IR Camera', myCam.imageBufferIRRight)

        #----------------------------------------#

        cv2.waitKey(100)
