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

        super().__init__('lidar')

        # Create publisher & set LiDAR parameters
        self.publisher_ = self.create_publisher(String, 'lidar', 10)
        self.numMeasurements 	     = 360	# Points
        self.lidarMeasurementMode 	 = 2
        self.lidarInterpolationMode  = 0
        self.danger_zones = [
            ((-0.1, 0.1), (0.1, 0.6)),    # <=== objects in front of the vehicle
            ((-0.1, -0.5), (0.1, -0.1))   # <=== objects behind the vehicle
        ]

        # Connect to LiDAR sensor
        self.lidar = Lidar(
            type='RPLidar',
            numMeasurements=self.numMeasurements,
            rangingDistanceMode=self.lidarMeasurementMode,
            interpolationMode=self.lidarInterpolationMode
        )

        #----------------------------------------#

        # Prepare system exit procedure 
        signal.signal(signal.SIGINT, self.signal_handler)

        self.detect_obstacles()


    # check if any detected object is a hazard
    def check_danger_zones(self, obstacles):

        #-------- 1.4 CHECK DANGER ZONES --------#
        
        # Coordinates of danger zones
        front_x_min, front_y_min = self.danger_zones[0][0]
        front_x_max, front_y_max = self.danger_zones[0][1]
        rear_x_min, rear_y_min = self.danger_zones[1][0]
        rear_x_max, rear_y_max = self.danger_zones[1][1]

        # Flags for obstacle detections
        in_danger_zone = [False, False]

        # Check if any detection is in a danger zone
        for detection in obstacles:
            x = detection["cluster"][:, 0]
            y = detection["cluster"][:, 1]

            if np.any((front_x_min <= x) & (x <= front_x_max) & (front_y_min <= y) & (y <= front_y_max)):
                in_danger_zone[0] = True
            if np.any((rear_x_min <= x) & (x <= rear_x_max) & (rear_y_min <= y) & (y <= rear_y_max)):
                in_danger_zone[1] = True

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
            while True:

                
                #---------- 1.2 SCAN & PLOT ----------#


                fig, ax = plt.subplots()
                plt.show(block=False)


                # Preprocess data
                self.lidar.read()
                distances = self.lidar.distances.flatten()
                angles = np.linspace(0, 2 * np.pi, self.numMeasurements)

                # Convert to cartesian coordinates
                x = distances * np.cos(angles)
                y = distances * np.sin(angles)
                data = np.vstack((x, y)).T


                #--------- 1.2 LOCATE CLUSTERS ---------#

                # Apply clustering algorithm
                db = DBSCAN(eps=0.2, min_samples=5).fit(data)  # Adjust eps and min_samples as needed
                labels = db.labels_
                unique_labels = set(labels) - {-1}

                # Reset for each scan
                os.system("clear")

                # List to store relevant clusters
                obstacles = []

                # Process data
                for label in unique_labels:
                    # Group data point clusters 
                    cluster_indices = np.where(labels == label)
                    cluster = data[cluster_indices]
                    
                    # Calculate distance of obstacle
                    cluster_distances = distances[cluster_indices]
                    cluster_distance = np.mean(cluster_distances)

                    if cluster_distance < 2 and cluster_distance > 0.5:
                        obstacles.append({
                            "cluster": cluster,
                            "distance": cluster_distance,
                            "label": label
                        })

                #----------------------------------------#

                # Check for obstacles in danger zone
                obstical_detection = self.check_danger_zones(obstacles)


                #--------- 1.2 PUBLISH WARNING ---------#
                msg = String()
                if obstical_detection == Obstical.FRONT_AND_REAR:
                    msg.data = "Front&Rear"
                    self.publisher_.publish(msg)
                    self.get_logger().info("Front&Rear")
                elif obstical_detection == Obstical.FRONT:
                    msg.data = "Front"
                    self.publisher_.publish(msg)
                    self.get_logger().info("Front")
                elif obstical_detection == Obstical.REAR:
                    msg.data = "Rear"
                    self.publisher_.publish(msg)
                    self.get_logger().info("Rear")
                elif obstical_detection == Obstical.NONE:
                    msg.data = "None"
                    self.publisher_.publish(msg)
                    self.get_logger().info("None")
                else:
                    self.get_logger().info("Unknown obstacle detection status")

                #----------------------------------------#

                # Plot data
                ax.clear()  # Clear the previous plot
                ax.scatter(data[:, 0], data[:, 1], c=labels, cmap='viridis')
                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_title('Lidar Data Clustering')

                # Add an arrow to indicate vehicle position and direction
                arrowprops = dict(facecolor='red', shrink=0.05, width=1.0, headwidth=3.0)
                ax.annotate('', xy=(0, 1), xytext=(0, 0), arrowprops=arrowprops)

                plt.draw()
                plt.pause(0.01)

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

def main(args=None):

    rclpy.init(args=args)
    rplidar = RPLidar()
    rclpy.spin(rplidar)
    rplidar.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#----------------------------------------#