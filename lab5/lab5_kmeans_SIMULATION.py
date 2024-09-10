'''
LiDAR KMeans Standalone Script

This script uses LiDAR sensors with unsupervised
learning in the form of K-Means clustering to
predict the location of 4 posts around the sensor.
'''

# Quanser imports
from pal.products.qcar import QCar
from pal.products.qcar import QCarLidar

# Standard imports
import sys
import numpy as np
import signal
import time

# Advanced imports
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull, Delaunay


class RPLidarKMeans:

    def __init__(self):
        self.numMeasurements = 360  # Points
        self.lidarMeasurementMode = 2  # Long range mode
        self.lidarInterpolationMode = 0  # No interpolation

        # Connect to LiDAR sensor
        self.lidar = QCarLidar(
            numMeasurements=self.numMeasurements,
            rangingDistanceMode=self.lidarMeasurementMode,
            interpolationMode=self.lidarInterpolationMode
        )

        self.LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        self.car = QCar()

        # Prepare system exit procedure
        signal.signal(signal.SIGINT, self.signal_handler)

        self.fig, self.ax = plt.subplots()
        plt.show(block=False)

        self.detect_rendezvous()

    #-- DETECT RENDEZVOUS WITH RESPECT TO CAR --#











    # Control LEDs on with:
    # self.LEDs = np.array([1, 1, 1, 1, 1, 1, 1, 1]) / self.LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    # self.car.read_write_std(0, 0, self.LEDs)

    #--------------------------------------------#

    # Handle system exits
    def signal_handler(self, sig, frame):
        print("Exiting gracefully...")
        self.destroy()
        sys.exit(0)

    # Terminate hardware processes
    def destroy(self):
        self.lidar.terminate()


if __name__ == '__main__':
    RPLidarKMeans()
