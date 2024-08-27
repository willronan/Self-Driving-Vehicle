'''
ROS LiDAR KMeans Node

This node uses LiDAR sensors with unsupervised
learning in the form of K-Means clustering to
predict the location of 4 posts around the sensor.
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
import os
import time

# Advanced imports
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull, Delaunay


class RPLidarKMeans(Node):

    def __init__(self):
        super().__init__('lidar_kmeans')

        # Create publisher
        self.publisher_ = self.create_publisher(String, 'lidar', 10)

        self.numMeasurements = 360  # Points
        self.lidarMeasurementMode = 2  # Long range mode
        self.lidarInterpolationMode = 0  # No interpolation

        # Connect to LiDAR sensor
        self.lidar = Lidar(
            type='RPLidar',
            numMeasurements=self.numMeasurements,
            rangingDistanceMode=self.lidarMeasurementMode,
            interpolationMode=self.lidarInterpolationMode
        )

        # Prepare system exit procedure
        signal.signal(signal.SIGINT, self.signal_handler)

        self.fig, self.ax = plt.subplots()
        plt.show(block=False)

        self.detect_rendezvous()

    #-- DETECT RENDEZVOUS WITH RESPECT TO CAR --#














    #--------------------------------------------#

    # Handle system exits
    def signal_handler(self, sig, frame):
        self.get_logger().info("Exiting gracefully...")
        self.destroy()
        sys.exit(0)

    # Terminate hardware processes
    def destroy(self):
        self.lidar.terminate()
        super().destroy_node()


# ROS pipeline
def main(args=None):
    rclpy.init(args=args)
    rplidar_kmeans = RPLidarKMeans()
    rclpy.spin(rplidar_kmeans)
    rplidar_kmeans.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
