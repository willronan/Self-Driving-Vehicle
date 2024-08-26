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

        self.detect_obstacles()

    def filter_data(self, data, min_distance=0.05, max_distance=0.8):

        
        if data.shape[0] == 0:
            raise ValueError("No data points available to filter.")
        
        distances = np.linalg.norm(data, axis=1)
        mask = (distances >= min_distance) & (distances <= max_distance)
        filtered_data = data[mask]

        if filtered_data.shape[0] == 0:
            raise ValueError("No data points left after filtering.")

        return filtered_data

    def cluster_and_find_centroids(self, data, k=4):

        
        if data.shape[0] < k:
            raise ValueError(f"Not enough data points to form {k} clusters.")
        
        kmeans = KMeans(n_clusters=k, random_state=0).fit(data)
        centroids = kmeans.cluster_centers_

        if len(centroids) < 4:
            raise ValueError("Not enough clusters found to form a quadrilateral.")

        return kmeans.labels_, centroids

    def calculate_directions(self, centroids):
        directions = np.arctan2(centroids[:, 1], centroids[:, 0])  # Angle in radians
        return np.degrees(directions)  # Convert to degrees
    



    def check_rendezvous(self, centroids):
        # Check if there are fewer than 3 points
        if len(centroids) < 3:
            return False

        # Compute the convex hull of the centroids
        hull = ConvexHull(centroids)

        # Check if the origin (0,0) is inside the convex hull
        def is_point_inside_polygon(polygon, point):
            hull = Delaunay(polygon)
            return hull.find_simplex(point) >= 0
        
        msg = String()
        if is_point_inside_polygon(centroids[hull.vertices], [0, 0]):
            msg.data = "true"
            self.publisher_.publish(msg)
            self.get_logger().info(f"Publishing {msg.data}")
        else:
            msg.data = "false"
            self.publisher_.publish(msg)
            self.get_logger().info(f"Publishing {msg.data}")

        



    def detect_obstacles(self):
        try:
            x_min, x_max = -2.0, 2.0
            y_min, y_max = -2.0, 2.0
            
            while rclpy.ok():


                self.lidar.read()
                distances = self.lidar.distances.flatten()
                angles = np.linspace(0, 2 * np.pi, self.numMeasurements)

                x = distances * np.cos(angles)
                y = distances * np.sin(angles)
                data = np.vstack((x, y)).T

                if data.shape[0] == 0:
                    self.get_logger().warn("No data points received from LiDAR.")
                    continue

                try:
                    filtered_data = self.filter_data(data)
                except ValueError as e:
                    self.get_logger().warn(f"Filtering error: {e}")
                    continue

                try:
                    labels, centroids = self.cluster_and_find_centroids(filtered_data, k=4)
                except ValueError as e:
                    self.get_logger().warn(f"Clustering error: {e}")
                    continue

                directions = self.calculate_directions(centroids)
                for i, direction in enumerate(directions):
                    self.get_logger().info(f"Post {i + 1} direction: {direction:.2f} degrees")

                self.check_rendezvous(centroids)

                self.ax.clear()
                self.ax.scatter(filtered_data[:, 0], filtered_data[:, 1], c='gray', s=1, label='Detected Points')
                self.ax.scatter(centroids[:, 0], centroids[:, 1], c='blue', s=50, marker='x', label='Centroids')

                if len(centroids) == 4:
                    sorted_indices = np.argsort(np.arctan2(centroids[:, 1], centroids[:, 0]))
                    sorted_vertices = centroids[sorted_indices]
                    sorted_vertices = np.vstack((sorted_vertices, sorted_vertices[0]))
                    self.ax.plot(sorted_vertices[:, 0], sorted_vertices[:, 1], 'r-')

                for centroid, direction in zip(centroids, directions):
                    self.ax.annotate(f'{direction:.1f}°', xy=centroid, textcoords='offset points', xytext=(0, 10), ha='center')


                

                self.ax.set_xlim(x_min, x_max)
                self.ax.set_ylim(y_min, y_max)
                self.ax.set_aspect('equal')
                arrowprops = dict(facecolor='red', shrink=0.05, width=1.0, headwidth=3.0)
                self.ax.annotate('', xy=(0, 1), xytext=(0, 0), arrowprops=arrowprops)
                self.ax.set_xlabel('X')
                self.ax.set_ylabel('Y')
                self.ax.set_title('Lidar Data Clustering')
                self.ax.legend()

                self.fig.canvas.draw()
                plt.pause(0.1)

        except Exception as e:
            self.get_logger().error(f"Failed to read from lidar: {e}")
        finally:
            self.lidar.terminate()

    def signal_handler(self, sig, frame):
        self.get_logger().info("Exiting gracefully...")
        self.destroy()
        sys.exit(0)

    def destroy(self):
        self.lidar.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    rplidar_kmeans = RPLidarKMeans()
    rclpy.spin(rplidar_kmeans)
    rplidar_kmeans.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
