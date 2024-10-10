# Standard imports
import numpy as np
import signal
import sys
from enum import Enum
from sklearn.cluster import DBSCAN
from pal.products.qcar import QCarLidar

# Placeholder for Lidar class (replace this with the actual import if needed)
# from pal.utilities.lidar import Lidar

class Obstical(Enum):
    FRONT_AND_REAR = 1
    FRONT = 2
    REAR = 3
    NONE = 0

class LidarDetector:

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


        self.danger_zones = [
            ((-0.1, 0.1), (0.1, 0.7)),    # Objects in front of the vehicle
            ((-0.1, -0.7), (0.1, -0.1))   # Objects behind the vehicle
        ]

        # Prepare system exit procedure
        signal.signal(signal.SIGINT, self.signal_handler)

        self.detect_obstacles()



    def check_danger_zones(self, obstacles):
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

    def detect_obstacles(self):
        try:
            while True:
                # Simulate LiDAR data read (replace with actual lidar.read())
                # distances = self.lidar.distances.flatten()
                distances = np.random.uniform(0, 2, self.numMeasurements)  # Example data
                angles = np.linspace(0, 2 * np.pi, self.numMeasurements)

                # Convert to cartesian coordinates
                x = distances * np.cos(angles)
                y = distances * np.sin(angles)
                data = np.vstack((x, y)).T

                # Apply clustering algorithm
                db = DBSCAN(eps=0.2, min_samples=5).fit(data)
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

                    if 0.05 < cluster_distance < 2:
                        obstacles.append({
                            "cluster": cluster,
                            "distance": cluster_distance,
                            "label": label
                        })

                # Check for obstacles in danger zones
                self.check_danger_zones(obstacles)

        except Exception as e:
            print(f"Failed to read from lidar: {e}")
        finally:
            # Safely disconnect hardware
            self.terminate_lidar()

    def terminate_lidar(self):
        # Placeholder for proper LiDAR termination (replace with actual termination)
        pass

    def signal_handler(self, sig, frame):
        print("Exiting gracefully...")
        self.terminate_lidar()
        sys.exit(0)

if __name__ == '__main__':
    LidarDetector()
