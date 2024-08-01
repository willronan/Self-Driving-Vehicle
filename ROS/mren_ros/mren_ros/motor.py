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
from qcar_interfaces.msg import MotorCmd 

# Standard imports
import sys
import time
import numpy as np


class Motor(Node):

    def __init__(self):

        # Create subscriber & add subscriptions
        super().__init__('motor')
        self.detection_subscription = self.create_subscription(String, 'detection', self.detection_callback, 10)
        self.steering_subscription = self.create_subscription(MotorCmd, 'steering', self.steering_callback, 10)
        self.lidar_subscription = self.create_subscription(String, 'lidar', self.lidar_callback, 10)
        
        # prevent unused variable warning
        self.detection_subscription  
        self.steering_subscription
        self.lidar_subscription

        # Initialize car
        self.LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        self.car = QCar()

        # Flags and counters
        self.stop_flag = False
        self.ignore_flag = False
        self.ignore_timer = 0
        self.danger_flag = False


    # Handle sign detection
    def detection_callback(self, msg):
        if msg.data == "Stop" and self.ignore_flag == False:
            self.stop_flag = True
        elif msg.data == "Go" or self.ignore_flag == True:
            self.stop_flag = False
        else: 
            print("Camera not detected")
            time.sleep(3)
        time.sleep(0.1)

        self.get_logger().info('A : "%s" was detected' % msg.data)

    # Handle lidar detections
    def lidar_callback(self, msg):

        print(f"Lidar says {msg.data}")
        if msg.data == "Danger":
            self.danger_flag = True
            self.get_logger().info('Danger!')
        elif msg.data == "Safe":
            self.danger_flag = False
            self.get_logger().info('Safe to proceed')

    # Handle steering/motor command
    def steering_callback(self, msg):

        if self.stop_flag == False and self.danger_flag == False:
            if self.ignore_flag == True:
                if time.time() - self.ignore_timer > 3:
                    self.ignore_flag = False
            self.car.read_write_std(msg.throttle, msg.steer, self.LEDs)
            self.get_logger().info('Motor controls processed')
        else:
            self.car.read_write_std(0, 0, self.LEDs)
            if self.stop_flag == True:
                time.sleep(3)
                self.ignore_flag = True
                self.ignore_timer = time.time()
            self.get_logger().info('Motor controls processed')

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