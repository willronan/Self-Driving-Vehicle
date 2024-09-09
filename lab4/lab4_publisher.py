'''lab4_publisher.py

This code will publish a simple
go or stop command based on keyboard
input.

'''

# ROS imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# Quanser imports
from pal.utilities.gamepad import LogitechF710
from pal.utilities.math import *

import time

# Gamepad Node
class Gamepad_Node(Node):

    def __init__(self):

        #------- 2.1 INIITALIZE PUBLISHER -------#





        pass # <=== clears empty method error, can be deleted
        #----------------------------------------#



    # Auto control via computer vison
    def drive_car(self):

        while True:

            #--------- 2.2 PUBLISH TO TOPIC ---------#






            #----------------------------------------#

            time.sleep(0.1)


    # Safely disconnect hardware
    def destroy(self):
        super().destroy_node()

def main(args=None):

     
    # ROS exectution pipeline
    rclpy.init(args=args)
    gamepad_node = Gamepad_Node()
    rclpy.spin(gamepad_node)
    gamepad_node.destroy()
    rclpy.shutdown()

    

if __name__ == '__main__':
    main()
