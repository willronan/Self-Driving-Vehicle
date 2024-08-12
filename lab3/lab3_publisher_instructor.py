'''lab3_publisher.py

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

        super().__init__('gamepad_node')

        # Create publisher 
        self.publisher_ = self.create_publisher(String, 'gamepad', 10)

        # Connect to remote controller & car hardware
        self.gpad  = LogitechF710()

        self.drive_car()

        #----------------------------------------#



    # Auto control via computer vison
    def drive_car(self):

        while True:

            #--------- 2.2 PUBLISH TO TOPIC ---------#

            # Read drive command from gamepad
            self.gpad.read()
            
            msg = String()

            if self.gpad.buttonA == 1:
                msg.data = "Go"
            else:
                msg.data = "Stop"
                

            # Publish drive command
            self.publisher_.publish(msg)
            self.get_logger().info(f"Publishing {msg.data} to the Gamepad topic")

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
