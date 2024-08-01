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

# Advanced imports 
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN



class RPLidar(Node):


    def __init__(self):
        super().__init__('lidar')

        # Create publisher & set LiDAR parameters
        self.publisher_ = self.create_publisher(String, 'lidar', 10)
        self.numMeasurements 	     = 360	# Points
        self.lidarMeasurementMode 	 = 2
        self.lidarInterpolationMode  = 0
        self.danger_zone = ((-0.2, 0.05), (0.2, 0.6))

        # Connect to LiDAR sensor
        self.lidar = Lidar(
            type='RPLidar',
            numMeasurements=self.numMeasurements,
            rangingDistanceMode=self.lidarMeasurementMode,
            interpolationMode=self.lidarInterpolationMode
        )

        # Create plot to visualize data
        #self.ax = plt.subplot(111, projection='polar')
        #plt.show(block=False)

        # Prepare system exit procedure 
        signal.signal(signal.SIGINT, self.signal_handler)

        self.detect_obsticals()


    # Perform obstical detection
    def detect_obsticals(self):

        try:
            while True:

                # Preprocess data
                self.lidar.read()
                distances = self.lidar.distances.flatten()
                angles = np.linspace(0, 2 * np.pi, self.numMeasurements)

                # Convert to cartisian coordinates
                x = distances * np.cos(angles)
                y = distances * np.sin(angles)
                data = np.vstack((x, y)).T

                # Apply clustering algorithm
                db = DBSCAN(eps=0.2, min_samples=5).fit(data)  # Adjust eps and min_samples as needed
                labels = db.labels_
                unique_labels = set(labels) - {-1}


                # Reset for each scan
                os.system("clear")
                num_objects = 0
                danger = False

                # Process data
                for label in unique_labels:

                    # Group data point clusters 
                    cluster_indices = np.where(labels == label)
                    cluster = data[cluster_indices]
                    
                    # Calculate distance of obstical
                    cluster_distances = distances[cluster_indices]
                    cluster_distance = np.mean(cluster_distances)


                    # Determine if obstical is in colision distance
                    if self.is_in_danger_zone(cluster):
                        danger = True
                        
                    # Count relevant obsticals 
                    if self.filter_clusters(cluster, cluster_distance):
                        continue
                    num_objects += 1

                # Publish LiDAR status
                if danger:
                    msg = String()
                    msg.data = "Danger"
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Publishing Danger")
                else:
                    msg = String()
                    msg.data = "Safe"
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Publishing Safe")

                print(f"Number of objects detected: {num_objects}")


                # Plot data
                #plt.scatter(data[:, 0], data[:, 1], c=labels, cmap='viridis')
                #plt.xlabel('X')
                #plt.ylabel('Y')
                #plt.title('Lidar Data Clustering')
                #plt.draw()
                #plt.pause(0.01)
                #plt.clf()

        # Except keyboard interrupt 
        except Exception as e:
            self.get_logger().error(f"Failed to read from lidar: {e}")
        


    # Filter clusters into relavent obsticals
    def filter_clusters(self, cluster, distance):

        # Remove walls
        if len(cluster) > 50:  
            return True

        # Remove obsticals farther than 2 m
        if distance > 2.0 or distance < 0.05:  # senses the front of car as cluster, hence ' < 0.05'
            return True
        
        return False


    # Check for obsticals within collision distance
    def is_in_danger_zone(self, cluster):
        x_min, y_min = self.danger_zone[0]
        x_max, y_max = self.danger_zone[1]
        x = cluster[:, 0]
        y = cluster[:, 1]
        
        in_danger_zone = np.any((x_min <= x) & (x <= x_max) & (y_min <= y) & (y <= y_max))
        return in_danger_zone


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