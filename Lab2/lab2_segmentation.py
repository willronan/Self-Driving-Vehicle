'''lab2_segmentation.py

In this walk through, students will
create a preprocessing algorithm to reduce
the realsense camera feed to the information
necassary to detect stop and yeild signs.
Using edge detection, signs will be identified 
and proper feedback will be displayed.

'''
from pal.utilities.vision import Camera3D

import numpy as np
import cv2
import time
import os


# Timing Parameters and methods
startTime = time.time()
def elapsed_time():
    return time.time() - startTime

sampleRate = 30.0
sampleTime = 1 / sampleRate
simulationTime = 200.0
print('Sample Time: ', sampleTime)

# Additional parameters
imageWidth = 1280
imageHeight = 720
font = cv2.FONT_HERSHEY_COMPLEX

# Initialize the RealSense camera for RGB and Depth data
myCam1 = Camera3D(mode='RGB&DEPTH', frameWidthRGB=imageWidth, frameHeightRGB=imageHeight)

    


# Performs image preproceessing

def preProcess(img):

    
    #----- 3.1 PEFORM PREPROCESSING-----#
    














    #----------------------------------------#


    #cv2.imshow('ppImag', x_put-img-here_x)
    #cv2.waitKey(1)

    ppImage = img
    return ppImage


def recognizeSign(img, ppImg):

    contours, _= cv2.findContours(ppImg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)    
    shape = "None"

    filtered_contours = []
    height, width = ppImg.shape
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if x > 0 and y > 0 and (x + w) < width and (y + h) < height:
            filtered_contours.append(contour) 

    # Clear the terminal
    os.system('cls' if os.name == 'nt' else 'clear')    

    for i, contour in enumerate(filtered_contours):
        
        #----- 3.2 PREDICT THE SHAPE -----#




        pass # <--- delete this when you fill this loop

        #----------------------------------#



        
    
    # Going through every contours found in the image. 
    for cnt in filtered_contours : 
    
        approx = cv2.approxPolyDP(cnt, 0.009 * cv2.arcLength(cnt, True), True) 
    
        # draws boundary of contours. 
        cv2.drawContours(img, [approx], 0, (0, 0, 255), 5)  
    
        # Used to flatted the array containing 
        # the co-ordinates of the vertices. 
        n = approx.ravel()  
        i = 0
    
        for j in n : 
            if(i % 2 == 0): 
                x = n[i] 
                y = n[i + 1] 
    
                # String containing the co-ordinates. 
                string = str(x) + " " + str(y)  
    
                if(i == 0): 
                    # text on topmost co-ordinate. 
                    cv2.putText(img, "Arrow tip", (x, y), 
                                    font, 0.5, (255, 0, 0))  
                else: 
                    # text on remaining co-ordinates. 
                    cv2.putText(img, string, (x, y),  
                            font, 0.5, (0, 255, 0))  
            i = i + 1

    cv2.imshow('Contours', img)
    cv2.waitKey(1)


    
try:
    while elapsed_time() < simulationTime:
        start = time.time()

        # Read the RGB and Depth data (latter in meters)
        myCam1.read_RGB()
        img = myCam1.imageBufferRGB

        ppImage = preProcess(img)
        #recognizeSign(img, ppImage) 

        end = time.time()

        # Calculate the computation time, and the time that the thread should pause/sleep for
        computationTime = end - start
        sleepTime = sampleTime - (computationTime % sampleTime)

        # Pause/sleep for sleepTime in milliseconds
        msSleepTime = int(1000 * sleepTime)
        if msSleepTime <= 0:
            msSleepTime = 1
        cv2.waitKey(msSleepTime)


except KeyboardInterrupt:
    print("User interrupted!")

finally:
    # Terminate RealSense camera object
    myCam1.terminate()
