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


class Motor_Node(Node):

    def __init__(self):

        #------- 2.3 INIITALIZE SUBSCRIBER -------#





        pass # <=== clears empty method error, can be deleted
        #----------------------------------------#



    # Handle sign detection
    def gamepad_callback(self, msg):

        #----- 2.3 WRITE CALLBACK FUNCTION -----#



        pass # <=== clears empty method error, can be deleted
        #----------------------------------------#






    #--- 3.5 STOP SIGN DETECTION CALLBACK ---#  






    #----------------------------------------#

    # Safely disconnect hardware
    def destroy(self):
        self.LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        self.car.read_write_std(0, 0, self.LEDs)
        super().destroy_node()

    


def main(args=None):


    #------- 2.5 EXECUTION PIPELINE -------#





    pass # <=== clears empty method error, can be deleted

    #----------------------------------------#


if __name__ == '__main__':
    main()