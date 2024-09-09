''' lab3_sliding_window.py

This walkthrough builds a object detection a
algorithm for detecting stop signs in images
'''

import numpy as np
import cv2
import time

# Compute the mean square error of colour data in two matrices 
def meanSquareError(img1, img2):

    #----- 2.1 PEFORM ERROR CALCUATION -----#





    #----------------------------------------#
    return error

# Computes pyramids of images (starts with the original and down samples).
#-- 1.3 ADJUST SCALE & SIZE RESTRAINTS --#

def pyramid(image, scale=1.5, minSize=30, maxSize=1000):

#----------------------------------------#
    yield image
    while True:
        w = int(image.shape[1] / scale)
        image = cv2.resize(image, (w, int(image.shape[0] / scale)))
        if image.shape[0] < minSize or image.shape[1] < minSize:
            break
        if image.shape[0] > maxSize or image.shape[1] > maxSize:
            continue
        yield image


# "Slides" a window over the image.
#-- 1.3 ADJUST SCALE & SIZE RESTRAINTS --#
def sliding_window(image, windowSize, stepSize=2):

#----------------------------------------#    
    for y in range(0, image.shape[0], stepSize):
        for x in range(0, image.shape[1], stepSize):
            yield (x, y, image[y:y+windowSize[1], x:x+windowSize[1]])


def visualize_process(img, x, y, w, h):

    # Draw the rectangle around the detected area

    x1 = x -5
    y1 = y - 5
    x2 = x + w + 5
    y2 = y + h + 5

    img_copy = img.copy()
    cv2.rectangle(img_copy, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.imshow('image', img_copy)
    cv2.waitKey(1)



def main():
    try:
        # Load and preprocess images

        #-- 2.2 TEST ALGORITHM WITH OTHER PICTURES --#
        targetImage = cv2.imread("test_image1.jpg")
        #--------------------------------------------#  

        targetImage = cv2.GaussianBlur(targetImage, (15, 15), 0)
        targetImage = cv2.resize(targetImage, (500, int(targetImage.shape[0] * (500.0 / targetImage.shape[1]))))

        prototypeImg = cv2.imread('stopPrototype.png')
        prototypeImg = cv2.GaussianBlur(prototypeImg, (15, 15), 0)

        print(prototypeImg.astype("float"))

        targetImage_hsv = cv2.cvtColor(targetImage, cv2.COLOR_BGR2HSV)
        prototypeImg_hsv = cv2.cvtColor(prototypeImg, cv2.COLOR_BGR2HSV)

        # Initialize variables for the sliding window and image pyramid process
        minError = float('inf')
        maxBox = (0, 0, 0, 0)

        t0 = time.time()


        for p in pyramid(prototypeImg_hsv, minSize=50, maxSize=targetImage_hsv.shape[0]):
            for (x, y, window) in sliding_window(targetImage_hsv, windowSize=(p.shape[1], p.shape[0])):
                if window.shape[0] != p.shape[0] or window.shape[1] != p.shape[1]:
                    continue
                
                tempError = meanSquareError(p, window)

                
                #visualize_process(targetImage, x, y, p.shape[1], p.shape[0])
                

                if tempError < minError:
                    minError = tempError
                    maxBox = (x, y, p.shape[1], p.shape[0])

                

        t1 = time.time()

        # Output results
        print(f"Execution time: {t1 - t0:.2f} seconds")
        print(f"Min Error: {minError:.4f}")
        print(maxBox) 

        # Draw the rectangle around the detected area
        buff1 = 5
        (x, y, w, h) = maxBox
        x1 = x - buff1
        y1 = y - buff1
        x2 = x + w + buff1
        y2 = y + h + buff1

        cv2.rectangle(targetImage, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.imshow('image', targetImage)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    except KeyboardInterrupt:
        print("User interrupted!")



# MAIN

if __name__ == '__main__':
    print("Started")
    main()
