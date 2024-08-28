'''
ROS Sign Detection Node

Uses Intel RealSense RGB-D
camera. ssd-mobilenet-v2 
supervised object detection
model used to detect sign.
Handles distance based on 
size of the bounding box.

'''

# ROS imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Jetson imports
from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput, cudaFromNumpy, cudaConvertColor, cudaAllocMapped

# Quanser imports
from pal.utilities.vision import Camera3D

# Standard imports
import os
import sys
import signal
import time

# Advanced imports
import numpy as np

class SignDetection(Node):

    def __init__(self):

        # Create publisher & set camera parameters
        super().__init__('sign_detection')
        self.publisher_ = self.create_publisher(String, 'detection', 10)
        self.imageWidth = 1280
        self.imageHeight = 720
        self.initialize_components()

        # Prepare system exit procedure
        signal.signal(signal.SIGINT, self.signal_handler)

        self.detect_signs()

    # Setup Intel Realsense camera
    def initialize_components(self):
        try:
            self.camera = Camera3D(mode='RGB&DEPTH', frameWidthRGB=self.imageWidth, frameHeightRGB=self.imageHeight)
            self.net = detectNet("ssd-mobilenet-v2", threshold=0.20)
            #self.display = videoOutput("display://0")  # 'my_video.mp4' for file
        except Exception as e:
            print(f"Failed to initialize components: {e}")
            self.cleanup()
            sys.exit(1)
    
    # Calculate area of detection bounding box
    def obj_area(self, left, right, top, bottom):
        length = np.abs(left - right)
        height = np.abs(top - bottom)
        area = length * height
        return area


    # Termination procedure
    def cleanup(self):
        if self.camera:
            self.camera.terminate()
        #if self.display:
        #    self.display.Close()


    def detect_signs(self):
        try:


            while rclpy.ok(): # and self.display.IsStreaming()

                # Resest with ever iteration
                sign_detected = False
                os.system('clear')

                try:

                    # Read camera data
                    self.camera.read_RGB()
                    img = self.camera.imageBufferRGB

                    # Preprocess image
                    bgrImg = cudaFromNumpy(img, isBGR=True)
                    cudaImg = cudaAllocMapped(width=bgrImg.width, height=bgrImg.height, format='rgb8')
                    cudaConvertColor(bgrImg, cudaImg)

                    if cudaImg is None:  # capture timeout
                        continue

                    # Performed model detection
                    detections = self.net.Detect(cudaImg)

                    # Sort detections
                    if detections:
                        for detection in detections:

                            # Check for stop signs
                            if detection.ClassID == 13:
                                
                                # Check if stop sign is in range
                                area = self.obj_area(detection.Left, detection.Right, detection.Top, detection.Bottom)
                                if detection.Confidence > 0.2 and area > 12000:
                                    print(detection.Confidence)
                                    print(area)
                                    sign_detected = True
                                    break

                    # Publish detection information
                    msg = String()
                    if sign_detected:
                        msg.data = "Stop"
                        self.publisher_.publish(msg)
                        self.get_logger().info(f"Publishing Stop")
                    else:
                        msg.data = "Go"
                        self.publisher_.publish(msg)
                        self.get_logger().info(f"Publishing Go")

                    #self.display.Render(cudaImg)
                    #self.display.SetStatus("Object Detection | Network {:.0f} FPS".format(self.net.GetNetworkFPS()))

                    time.sleep(0.1)
                    

                except Exception as e:
                    print(f"Error during processing: {e}")
                    break

        except Exception as e:
            print(f"Failed to initialize components: {e}")

        finally:
            self.cleanup()
            sys.exit(0)



    # Safely disconnect hardware
    def signal_handler(self, sig, frame):
        print("Exiting gracefully...")
        self.cleanup()
        sys.exit(0)

    def destroy(self):
        self.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    sign_detection = SignDetection()
    rclpy.spin(sign_detection)
    sign_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
