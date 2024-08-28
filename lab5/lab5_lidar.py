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


class RPLidar(Node):

    def __init__(self):


        super().__init__('lidar')

        # Create publisher 
        self.publisher_ = self.create_publisher(String, 'lidar', 10)


        self.numMeasurements 	     = 360	# Points
        self.lidarMeasurementMode 	 = 2    # Long range mode
        self.lidarInterpolationMode  = 0    # No interpolation

        # Connect to LiDAR sensor
        self.lidar = Lidar(
            type='RPLidar',
            numMeasurements=self.numMeasurements,
            rangingDistanceMode=self.lidarMeasurementMode,
            interpolationMode=self.lidarInterpolationMode
        )

        # Prepare system exit procedure 
        signal.signal(signal.SIGINT, self.signal_handler)

        self.detect_obstacles()



    #-- DETECT OBSTACLES SEND WARNING MESSAGES --#














    #---------------------------------------------#


        

    # Handle keyboard interrupt 
    def signal_handler(self, sig, frame):
        print("Exiting gracefully...")
        self.destroy()
        sys.exit(0)


    # Safely disconnect hardware
    def destroy(self):
        self.lidar.terminate()  # Properly terminate QCar instance
        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)
    rplidar = RPLidar()
    rclpy.spin(rplidar)
    rplidar.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
