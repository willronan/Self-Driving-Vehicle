'''lab3_segmentation.py

In this walk through, students will
create a preprocessing algorithm to reduce
the realsense camera feed to the information
necassary to detect stop and yeild signs.
Using edge detection, signs will be identified 
and proper feedback will be displayed
'''

from pal.products.qcar import QCarRealSense
from pal.products.qcar import IS_PHYSICAL_QCAR

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

# Initialize the RealSense camera for RGB and Depth datas
myCam1 = QCarRealSense(mode='RGB, Depth', frameWidthRGB=imageWidth, frameHeightRGB=imageHeight)

import quanser
 


#---------------------------------------#
#                                       #
#     Performs image preproceessing     #
#                                       #
#---------------------------------------#


def preProcess(img):


    #----- 3.1 PEFORM PREPROCESSING-----#


    blurred = cv2.GaussianBlur(img, (3, 3), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV) 

    # Define lower and upper bounds for each range of red hue
    lower_red1 = np.array([0, 80, 80])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 80, 80])
    upper_red2 = np.array([180, 255, 255])

    # Create masks for each range of red hue
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

    # Combine the masks to get the final mask
    mask = cv2.bitwise_or(mask1, mask2)

    # Close holes and remove noise
    kernel = np.ones((5, 5), np.uint8) 

    erode= cv2.erode(mask, kernel, iterations=1)  
    closing = cv2.morphologyEx(erode, cv2.MORPH_CLOSE, kernel, iterations = 11) 
    ppImage = closing 

    #----------------------------------------#

    
    cv2.imshow('ppImag', ppImage)
    cv2.waitKey(1)

    return ppImage



#---------------------------------------#
#                                       #
# Detects Signs in preprocessed image   #
#                                       #
#---------------------------------------#

def recognizeSign(img, ppImg):


    contours, _= cv2.findContours(ppImg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)    
    shape = "None"

    # Sometimes the find contours method returns a box around the entire display
    # the lines directly below elimate this contour set
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
        
        # Predict the shape based on number of vertices
        
        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        num_vertices = len(approx)
        
        # Print the shape prediction
        if num_vertices == 3:
            shape = "Yield"
            print(f"{shape} sign detected")
        elif num_vertices == 8:
            shape = "Stop"  # More vertices indicate a rounder shape
            print(f"{shape} sign detected")



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

        cv2.imshow('img', img)
            
        # Apply Preprocessing to Image
        ppImage = preProcess(img)
        recognizeSign(img, ppImage)
        

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
