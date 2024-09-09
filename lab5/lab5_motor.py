'''
ROS Motor Control Node

Takes in feedback from all
other nodes. Based on sensor
information, executes
correstponding drive commands,

'''

# ROS imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Quanser imports
from pal.products.qcar import QCar

# Standard imports
import sys
import time
import numpy as np


class Motor(Node):

    def __init__(self):

        # Create subscriber & add subscriptions
        super().__init__('motor')
        self.steering_subscription = self.create_subscription(String, 'gamepad', self.steering_callback, 10)
        self.lidar_subscription = self.create_subscription(String, 'lidar', self.lidar_callback, 10)
        
        # prevent unused variable warning
        self.steering_subscription
        self.lidar_subscription

        # Initialize car
        self.LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        self.car = QCar()

        # Flags and counters
        self.arrivedFlag = False

    # Handle lidar detections
    def lidar_callback(self, msg):

        #-- SET LEDS ACCORDING TO RENDEZVOUS STATUS --#


        pass
        #---------------------------------------------#
        

    # Handle steering/motor command
    def steering_callback(self, msg):

        throttle, steer = map(float, msg.data.split(','))

        print(msg.data)

        # If the car is not in the rendezvous accept drive commands
        self.car.read_write_std(throttle, steer, self.LEDs)



    # Safely disconnect hardware
    def destroy(self):
        self.LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        self.car.read_write_std(0, 0, self.LEDs)
        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)
    motor = Motor()
    rclpy.spin(motor)
    motor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()