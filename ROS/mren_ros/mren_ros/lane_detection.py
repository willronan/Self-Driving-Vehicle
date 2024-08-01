'''
ROS Lane Detection Node

Isolates the lane using CV2 filtration.
Calculates the slope of the line.
Computes appropriate motor commands
using PI control.

'''


# ROS imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Jetson imports
from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput

# Quanser imports
from pal.utilities.vision import Camera2D
from pal.utilities.gamepad import LogitechF710
from pal.products.qcar import QCar
from pal.utilities.math import *
from hal.utilities.image_processing import ImageProcessing
from qcar_interfaces.msg import MotorCmd 

# Standard Imports
import time
import numpy as np
import cv2
import math
import os

# Command Coefficients
STEERING_COEF = 0.5
THROTTLE_COEF = 0.09


# Lane detection Node
class LaneDetection(Node):

    def __init__(self):
        super().__init__('lane_detection')

        # Create publisher & set camera parameters
        self.publisher_ = self.create_publisher(MotorCmd, 'steering', 10)
        self.counter = 0
        self.imageWidth = 1640
        self.imageHeight = 820
        self.cameraID = '3'
        self.display = videoOutput("display://0")
        self.camera = Camera2D(cameraId=self.cameraID, frameWidth=self.imageWidth, frameHeight=self.imageHeight, frameRate=60)

        # Connect to remote controller & car hardware
        self.gpad  = LogitechF710()
        self.car = QCar()

        # Setting Filter
        self.steeringFilter = Filter().low_pass_first_order_variable(25, 0.033)
        next(self.steeringFilter)
        self.dt = 0.033

        self.steer_car()


    # Manual Control via remote controller
    def control_from_gamepad(self, LB, RT, leftLateral, A):
        # Implement Gamepad Controls
        if LB == 1:
            if A == 1:
                throttle_axis = -THROTTLE_COEF * RT  # going backward
            else:
                throttle_axis = THROTTLE_COEF * RT  # going forward
            steering_axis = leftLateral * STEERING_COEF
        else:
            throttle_axis = 0
            steering_axis = 0

        return throttle_axis, steering_axis

    # Auto control via computer vison
    def steer_car(self):

        while True:

            # Capture RGB Image from CSI
            try:
                self.camera.read()
                croppedRGB = self.camera.imageData[524:674, 0:820]
            except Exception as e:
                self.get_logger().error(f"Error reading camera data: {e}")
                continue

            # Preprocess image & show
            try:
                hsvBuf = cv2.cvtColor(croppedRGB, cv2.COLOR_BGR2HSV)
                binaryImage = ImageProcessing.binary_thresholding(
                    frame=hsvBuf,
                    lowerBounds=np.array([10, 55, 80]),
                    upperBounds=np.array([40, 255, 200])
                )
                cv2.imshow('Crop RGB Image', croppedRGB)
                cv2.imshow('Binary Image', binaryImage)
            except Exception as e:
                self.get_logger().error(f"Error processing image: {e}")
                continue

            # Calculate slope and compute PI control
            try:
                slope, intercept = ImageProcessing.find_slope_intercept_from_binary(binary=binaryImage)
                rawSteering = 1.5 * (slope - 0.3419) + (1 / 150) * (intercept + 5)
                auto_steering = self.steeringFilter.send((np.clip(rawSteering, -0.5, 0.5), self.dt))
            except Exception as e:
                self.get_logger().error(f"Error calculating steering: {e}")
                continue

            # Read drive command from gamepad
            new = self.gpad.read()
            throttle, steering= self.control_from_gamepad(self.gpad.buttonLeft, self.gpad.trigger, self.gpad.leftJoystickX, self.gpad.buttonA)

            # If selected, use autodrive
            if self.gpad.buttonX == 1:
                if math.isnan(auto_steering):
                    steering = 0
                else:
                    steering = auto_steering
                throttle = throttle * np.cos(auto_steering)

            # Publish drive command
            msg = MotorCmd()
            msg.throttle = float(throttle)
            msg.steer = float(steering)
            self.publisher_.publish(msg)
            self.get_logger().info(f"Publishing Motor Cmd")

            cv2.waitKey(1)

    # Safely disconnect hardware
    def destroy(self):
        self.camera.Close()
        self.car.terminate()  
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    lane_detection = LaneDetection()
    rclpy.spin(lane_detection)
    lane_detection.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
