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


class Motor_Node(Node):

    def __init__(self):

        #------- 2.3 INIITALIZE SUBSCRIBER -------#

        # Create subscriber & add subscriptions
        super().__init__('motor_node')


        self.gamepad_subscription = self.create_subscription(String, 'gamepad', self.gamepad_callback, 10)
        self.detection_subscription = self.create_subscription(String, 'detection', self.detection_callback, 10)

        # prevent unused variable warning
        self.gamepad_subscription  
        self.detection_subscription

        self.sign_detected_flag = False

        # Initialize car
        self.LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        self.car = QCar()

        #----------------------------------------#



    # Handle sign detection
    def gamepad_callback(self, msg):

        #----- 2.3 WRITE CALLBACK FUNCTION -----#
        if msg.data == "Stop" or self.sign_detected_flag == True:
            self.car.read_write_std(0, 0, self.LEDs)
        elif msg.data == "Go" and self.sign_detected_flag == False:
            self.car.read_write_std(0.1, 0, self.LEDs)
            

        time.sleep(0.1)

        self.get_logger().info('Told to %s' % msg.data)


    #--- 3.5 STOP SIGN DETECTION CALLBACK ---#  
    def detection_callback(self, msg):

        self.get_logger().info('Detection status is %s' % msg.data)
        if msg.data == "True":
            self.sign_detected_flag = True
        elif msg.data == "False":
            self.sign_detected_flag = False

    #----------------------------------------#

    # Safely disconnect hardware
    def destroy(self):
        self.LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        self.car.read_write_std(0, 0, self.LEDs)
        super().destroy_node()

    


def main(args=None):


    #------- 2.5 EXECUTION PIPELINE -------#

    rclpy.init(args=args)
    motor_node = Motor_Node()
    rclpy.spin(motor_node)
    motor_node.destroy_node()
    rclpy.shutdown()

    #----------------------------------------#


if __name__ == '__main__':
    main()