'''lab3_supervised_detection

This walkthough has students compare 
several supervised detection models
to choose the best one for autonomous
driveing.

'''


from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput, cudaFromNumpy, cudaConvertColor, cudaAllocMapped


from pal.utilities.vision import Camera3D

import os
import sys
import signal
import time

import numpy as np

imageWidth = 1280
imageHeight = 720


# Calculate area of detection bounding box


#----- 1.3 CALCULATE BOUNDING AREA -----#

def obj_area(left, right, top, bottom):
    length = np.abs(left - right)
    height = np.abs(top - bottom)
    area = length * height
    return area

#----------------------------------------#


# Termination procedure

def detect_signs():
    try:

        try:
            camera = Camera3D(mode='RGB&DEPTH', frameWidthRGB=imageWidth, frameHeightRGB=imageHeight)
            net = detectNet("ssd-mobilenet-v2", threshold=0.25)
            display = videoOutput("display://0")

        except Exception as e:
            print(f"Failed to initialize components: {e}")
            camera.terminate()
            display.Close()
            sys.exit(1)

        while True:

            # Resest with ever iteration
            sign_detected = False
            os.system('clear')

            try:

                # Read camera data
                camera.read_RGB()
                img = camera.imageBufferRGB

                # Preprocess image

                #------ 1.1 CONVERT TO CUDA IMAGE ------#

                bgrImg = cudaFromNumpy(img, isBGR=True)
                cudaImg = cudaAllocMapped(width=bgrImg.width, height=bgrImg.height, format='rgb8')
                cudaConvertColor(bgrImg, cudaImg)

                #----------------------------------------#

                if cudaImg is None:  # capture timeout
                    continue

                

                #----- 1.2 PERFORM OBJECT DETECTION -----#

                # Performed model detection
                detections = net.Detect(cudaImg)

                # Sort detections
                if detections:
                    for detection in detections:

                        # Check for stop signs
                        if detection.ClassID == 13:
                            
                            # Check if stop sign is in range
                            area = obj_area(detection.Left, detection.Right, detection.Top, detection.Bottom)
                            if detection.Confidence > 0.325 and area > 12000:
                                print(detection.Confidence)
                                print(area)
                                sign_detected = True
                                break

                #----------------------------------------#

                display.Render(cudaImg)
                display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))

                time.sleep(0.1)
                

            except Exception as e:
                print(f"Error during processing: {e}")
                break

    except Exception as e:
        print(f"Failed to initialize components: {e}")

    finally:
        camera.terminate()
        display.Close()
        sys.exit(0)




def main(args=None):
    detect_signs()



if __name__ == '__main__':
    main()
