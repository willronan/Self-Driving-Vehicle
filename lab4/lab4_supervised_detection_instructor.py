'''lab4_supervised_detection

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


def detect_signs():
    display = None  # Initialize display to None

    try:
        try:

            #------ 1.1 CONVERT TO CUDA IMAGE ------#

            camera = Camera3D(mode='RGB&DEPTH', frameWidthRGB=imageWidth, frameHeightRGB=imageHeight)
            net = detectNet("ssd-inception-v2", threshold=0.25)  #	trafficcamnet
            display = videoOutput("display://0") 

            #----------------------------------------#

        except Exception as e:
            print(f"Failed to initialize components: {e}")
            if camera:
                camera.terminate()
            if display:
                display.Close()
            sys.exit(1)

        while True:
            # Reset with every iteration
            os.system('clear')

            try:
                # Read camera data
                camera.read_RGB()
                img = camera.imageBufferRGB

                # Preprocess image

                #------ 1.2 CONVERT TO CUDA IMAGE ------#

                bgrImg = cudaFromNumpy(img, isBGR=True)
                cudaImg = cudaAllocMapped(width=bgrImg.width, height=bgrImg.height, format='rgb8')
                cudaConvertColor(bgrImg, cudaImg)

                #----------------------------------------#

                if cudaImg is None:  # capture timeout
                    continue

                #----- 1.3 PERFORM OBJECT DETECTION -----#

                # Perform model detection
                detections = net.Detect(cudaImg)

                # Sort detections
                if detections:
                    
                    for detection in detections:
                        if net.GetClassDesc(detection.ClassID) in ["road_sign", "stop sign"]:
                            print("Sign Detected!")

                #----------------------------------------#

                if display:
                    display.Render(cudaImg)
                    display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))

                time.sleep(0.1)

            except Exception as e:
                print(f"Error during processing: {e}")
                break

    except Exception as e:
        print(f"Failed to initialize components: {e}")

    finally:
        if camera:
            camera.terminate()
        if display:
            display.Close()
        sys.exit(0)





def main(args=None):
    detect_signs()



if __name__ == '__main__':
    main()
