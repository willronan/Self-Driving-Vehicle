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
    pass    # eliminates error message

#----------------------------------------#


# Termination procedure

def detect_signs():
    try:

        try:
            camera = Camera3D(mode='RGB&DEPTH', frameWidthRGB=imageWidth, frameHeightRGB=imageHeight)
            display = videoOutput("display://0")

            # detection model
            # net = detectNet("ssd-mobilenet-v2", threshold=0.25)

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


                cudaImg = img

                # Preprocess image

                #------ 1.1 CONVERT TO CUDA IMAGE ------#






                #----------------------------------------#

                if cudaImg is None:  # capture timeout
                    continue

                

                #----- 1.2 PERFORM OBJECT DETECTION -----#







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
