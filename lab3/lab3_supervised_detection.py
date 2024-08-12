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


def detect_signs():
    display = None  # Initialize display to None

    try:
        try:

            #------ 1.1 CONVERT TO CUDA IMAGE ------#



            pass # <=== clears empty try block error, can be deleted

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






                #----------------------------------------#

                if cudaImg is None:  # capture timeout
                    continue

                #----- 1.3 PERFORM OBJECT DETECTION -----#







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
