'''
ROS LiDAR KMeans Node

This node uses LiDAR sensors with unsupervised
learning in the form of K-Means clustering to
predict the location of 4 posts around the sensor.
'''

# Quanser imports
from pal.products.qcar import QCar
from pal.products.qcar import QCarLidar
from pal.utilities.gamepad import LogitechF710

# Standard imports
import sys
import numpy as np
import signal
import os
import time
from enum import Enum

# Advanced imports
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt

class Obstical(Enum):
    FRONT_AND_REAR = 1
    FRONT = 2
    REAR = 3
    NONE = 0

STEERING_COEF = 0.5
THROTTLE_COEF = 0.09

class ObstacleAvoidance:

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
        self.gpad  = LogitechF710()
        self.LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        self.car = QCar()

        # Prepare system exit procedure
        signal.signal(signal.SIGINT, self.signal_handler)

        self.fig, self.ax = plt.subplots()
        plt.show(block=False)

        self.front_obstacle_flag = False
        self.rear_obstacle_flag = False

        self.danger_zones = [
            ((0.1, -0.1), (0.4, 0.1)),    # <=== objects in front of the vehicle
            ((-0.4, -0.1), (-0.1, 0.1))   # <=== objects behind the vehicle
        ]

        self.drive()



    def drive(self):

        throttle = 0
        steer = 0

    
        while True:

            try:
                # Preprocess data
                self.lidar.read()
                distances = self.lidar.distances.flatten()
                angles = np.linspace(0, 2 * np.pi, self.numMeasurements)

                # Convert to cartesian coordinates
                x = distances * np.cos(angles)
                y = distances * np.sin(angles)
                data = np.vstack((x, y)).T

                # Apply clustering algorithm
                db = DBSCAN(eps=0.2, min_samples=5).fit(data)  # Adjust eps and min_samples as needed
                labels = db.labels_
                unique_labels = set(labels) - {-1}



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

                    if cluster_distance < 2 and cluster_distance > 0.05:
                        obstacles.append({
                            "cluster": cluster,
                            "distance": cluster_distance,
                            "label": label
                        })


                # Check for obstacles in danger zone
                obstical_detection = self.check_danger_zones(obstacles)

                if obstical_detection == Obstical.FRONT_AND_REAR:
                    self.front_obstacle_flag = True
                    self.rear_obstacle_flag = True
                elif obstical_detection == Obstical.FRONT:
                    self.front_obstacle_flag = True
                elif obstical_detection == Obstical.REAR:
                    self.rear_obstacle_flag = True
                elif obstical_detection == Obstical.NONE:
                    self.front_obstacle_flag = False
                    self.rear_obstacle_flag = False
                else:
                    print("Unknown obstacle detection status")


                # Plot data
                self.ax.clear()  # Clear the previous plot
                
                self.ax.scatter(data[:, 0], data[:, 1], c=labels, cmap='viridis')

                self.ax.set_xlabel('X')
                self.ax.set_ylabel('Y')
                self.ax.set_title('Lidar Data Clustering')

                # Add an arrow to indicate vehicle position and direction
                arrowprops = dict(facecolor='red', shrink=0.05, width=1.0, headwidth=2.0)
                self.ax.annotate('', xy=(0, 1), xytext=(0, 0), arrowprops=arrowprops)

                plt.draw()
                plt.pause(0.01)

            except Exception as e:
                print(f"Failed to read from lidar: {e}")
            finally:
                self.destroy()



            new = self.gpad.read()
            throttle, steering = self.control_from_gamepad(self.gpad.buttonLeft, self.gpad.trigger, self.gpad.leftJoystickX, self.gpad.buttonA)
            if throttle > 0 and not self.front_obstacle_flag:
                self.car.read_write_std(throttle, steer, self.LEDs)
            # If the command is backwards, and there is no rear obstacle, go backwards
            elif throttle < 0 and not self.rear_obstacle_flag:
                self.car.read_write_std(throttle, steer, self.LEDs)
            # Otherwise stop
            else:
                self.car.read_write_std(0, steer, self.LEDs)


    # Manual Control via remote controller
    def control_from_gamepad(self, LB, RT, leftLateral, A):
        # Implement Gamepad Controls
        if LB == 1:
            if A == 1:
                throttle_axis = -THROTTLE_COEF * RT  # going backward
            else:
                throttle_axis = THROTTLE_COEF * RT  # going forward
            steering_axis = leftLateral * STEERING_COEF
        else:
            throttle_axis = 0
            steering_axis = 0

        return throttle_axis, steering_axis


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

        # Check if any detection is in a danger zone
        for obstacle in obstacles:
            x = obstacle["cluster"][:, 0]
            y = obstacle["cluster"][:, 1]

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


    # Handle system exits
    def signal_handler(self, sig, frame):
        self.destroy()
        sys.exit(0)

    # Terminate hardware processes
    def destroy(self):
        self.lidar.terminate()
        self.car.terminate()  




if __name__ == '__main__':
    ObstacleAvoidance()