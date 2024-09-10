'''lab4_publisher.py

This code will publish a simple
go or stop command based on keyboard
input.

'''

# Jetson Inference imports
from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput, cudaFromNumpy, cudaConvertColor, cudaAllocMapped

# ROS imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# Quanser imports
from pal.utilities.vision import Camera3D
import time

networkPath = "/home/nvidia/jetson-inference/python/training/detection/ssd/models/qsigns/"

# Gamepad Node
class Detector_Node(Node):

    def __init__(self):

        super().__init__('detector_node')

        imageWidth = 1280
        imageHeight = 720

        # Connect to remote controller & car hardware
        self.camera = Camera3D(mode='RGB&DEPTH', frameWidthRGB=imageWidth, frameHeightRGB=imageHeight)
        self.net = detectNet(model=networkPath + "ssd-mobilenet.onnx", 
            labels=networkPath + "labels.txt", input_blob="input_0", 
            output_cvg="scores", output_bbox="boxes", 
            threshold=0.2)

        self.perform_detection()


    # Detection system
    def perform_detection(self):

            
        while True:
            sign_detected = False
            # Reset with every iteration
            msg = String()


            # Read camera data
            self.camera.read_RGB()
            img = self.camera.imageBufferRGB

            # Preprocess image
            bgrImg = cudaFromNumpy(img, isBGR=True)
            cudaImg = cudaAllocMapped(width=bgrImg.width, height=bgrImg.height, format='rgb8')
            cudaConvertColor(bgrImg, cudaImg)


            if cudaImg is None:  # capture timeout
                continue


            # Perform model detection
            detections = self.net.Detect(cudaImg)

            # Sort detections
            if len(detections) == 1:
                for detection in detections:
                    msg.data = str(detection.confidence)
            elif len(detections) > 1:       
                msg.data = str("FP")
            else:
                msg.data = "None"


            # Publish drive command
            self.publisher_.publish(msg)
            self.get_logger().info(f"Publishing {msg.data} to the detection topic")


            time.sleep(0.01)
        


    # Safely disconnect hardware
    def destroy(self):
        self.camera.terminate()
        super().destroy_node()

def main(args=None):

     
    # ROS exectution pipeline
    rclpy.init(args=args)
    detector_node = Detector_Node()
    rclpy.spin(detector_node)
    detector_node.destroy()
    rclpy.shutdown()

    

if __name__ == '__main__':
    main()
