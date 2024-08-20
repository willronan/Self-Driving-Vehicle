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

        #------ 2.1.1 INITIALIZE LIDAR NODE ------#







        #----------------------------------------#

        self.danger_zones = [
            ((-0.1, 0.1), (0.1, 0.6)),    # <=== objects in front of the vehicle
            ((-0.1, -0.5), (0.1, -0.1))   # <=== objects behind the vehicle
        ]

        # Prepare system exit procedure 
        signal.signal(signal.SIGINT, self.signal_handler)

        self.detect_obstacles()


    # check if any detected object is a hazard
    def check_danger_zones(self, obstacles):

        os.system("clear")

        # Coordinates of danger zones
        front_x_min, front_y_min = self.danger_zones[0][0]
        front_x_max, front_y_max = self.danger_zones[0][1]
        rear_x_min, rear_y_min = self.danger_zones[1][0]
        rear_x_max, rear_y_max = self.danger_zones[1][1]

        # Flags for obstacle detections
        in_danger_zone = [False, False]

        #-------- 2.4 CHECK DANGER ZONES --------#






        #----------------------------------------#

        # Return corresponding warning code
        if in_danger_zone[0] and in_danger_zone[1]:
            print("Front and rear")
            return Obstical.FRONT_AND_REAR
        elif in_danger_zone[1]:
            print("Rear")
            return Obstical.REAR
        elif in_danger_zone[0]:
            print("Front")
            return Obstical.FRONT
        else:
            print("None")
            return Obstical.NONE
        


    def detect_obstacles(self):
        try:

            fig, ax = plt.subplots()
            plt.show(block=False)


            while True:

                
             #---------- 2.2 SCAN & PLOT ----------#




                #--------- 2.3 LOCATE CLUSTERS ---------#
                obstacles = []


                #----------------------------------------#

                # Check for obstacles in danger zone
                obstical_detection = self.check_danger_zones(obstacles)


                #--------- 2.5 PUBLISH WARNING ---------#


                #----------------------------------------#


                # Add an arrow to indicate vehicle position and direction
                # arrowprops = dict(facecolor='red', shrink=0.05, width=1.0, headwidth=2.0)
                # ax.annotate('', xy=(0, 1), xytext=(0, 0), arrowprops=arrowprops)



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

#------ 2.1.2 SETUP ROS PIPELINE ------#





#----------------------------------------#