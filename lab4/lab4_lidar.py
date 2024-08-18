'''
ROS LiDAR Detection Node

Intiates a LiDAR sensor and
begins collecting data. 
Uses DBSCAN clustering to
count the number of surrounding
obsticals. Provides feedback if
obstical in collision distance.

'''

# ROS imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Quanser imports
from pal.products.qcar import QCar
from pal.utilities.lidar import Lidar

# Standard imports
import sys
import numpy as np
import signal
import time
import os
from enum import Enum


# Advanced imports 
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt

class Obstical(Enum):
    FRONT_AND_REAR = 1
    FRONT = 2
    REAR = 3
    NONE = 0

class RPLidar(Node):

    def __init__(self):

        #------ 1.1.1 INITIALIZE LIDAR NODE ------#



        #----------------------------------------#

        # Prepare system exit procedure 
        signal.signal(signal.SIGINT, self.signal_handler)

        self.detect_obstacles()


    # check if any detected object is a hazard
    def check_danger_zones(self, obstacles):

        #-------- 1.4 CHECK DANGER ZONES --------#
        
        pass

        #----------------------------------------#
        


    def detect_obstacles(self):
        try:
            while True:

                
                #---------- 1.2 SCAN & PLOT ----------#


                #----------- 1.2 LOCATE CLUSTERS -----------#



                #--------------------------------------------#


                #------------ 1.2 PUBLISH WARNING -----------#


                #--------------------------------------------#

                pass
                #----------------------------------------#


        except Exception as e:
            self.get_logger().error(f"Failed to read from lidar: {e}")
            print(f"Failed to read from lidar: {e}")
        finally:
            self.lidar.terminate()  # Ensure the LiDAR is terminated
            # Except keyboard interrupt 

        

    # Handle keyboard interrupt 
    def signal_handler(self, sig, frame):
        print("Exiting gracefully...")
        self.destroy()
        sys.exit(0)


    # Safely disconnect hardware
    def destroy(self):
        self.lidar.terminate()  # Properly terminate QCar instance
        super().destroy_node()

#------ 1.1.2 SETUP ROS PIPELINE ------#



#----------------------------------------#