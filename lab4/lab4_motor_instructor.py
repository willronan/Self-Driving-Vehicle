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
        self.stop_flag = False
        self.ignore_flag = False
        self.ignore_timer = 0
        self.front_obstacle_flag = False
        self.rear_obstacle_flag = False


    # Handle lidar detections
    def lidar_callback(self, msg):

        print(f"Lidar says {msg.data}")
        if msg.data == "Front&Rear":
            self.front_obstacle_flag = True
            self.rear_obstacle_flag = True
        elif msg.data == "Front":
            self.front_obstacle_flag = True
        elif msg.data == "Rear":
            self.rear_obstacle_flag = True
        elif msg.data == "None":
            self.front_obstacle_flag = False
            self.rear_obstacle_flag = False
    

    # Handle steering/motor command
    def steering_callback(self, msg):

        print("hello")

        throttle, steer = map(float, msg.data.split(','))

        #---------- 3 HANDLE WARNINGS ----------#

        print(msg.data)

        # If the command is forwards, and there is no front obstacle, go forwards
        if throttle > 0 and not self.front_obstacle_flag:
            self.car.read_write_std(throttle, steer, self.LEDs)
        # If the command is backwards, and there is no rear obstacle, go backwards
        elif throttle < 0 and not self.rear_obstacle_flag:
            self.car.read_write_std(throttle, steer, self.LEDs)
        # Otherwise stop
        else:
            self.car.read_write_std(0, steer, self.LEDs)

        #----------------------------------------#
            

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