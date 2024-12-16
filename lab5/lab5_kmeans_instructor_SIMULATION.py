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
#if sklearn is not found
# use "pip install scikit-learn" in terminal
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

    # Remove unwanted datapoints
    def filter_data(self, data, min_distance=0.05, max_distance=0.8):

        if data.shape[0] == 0:
            raise ValueError("No data points available to filter.")

        distances = np.linalg.norm(data, axis=1)
        mask = (distances >= min_distance) & (distances <= max_distance)
        filtered_data = data[mask]

        if filtered_data.shape[0] == 0:
            raise ValueError("No data points left after filtering.")

        return filtered_data

    # Perform kmeans clustering and return the data
    def cluster_and_find_centroids(self, data, k=4):

        if data.shape[0] < k:
            raise ValueError(f"Not enough data points to form {k} clusters.")

        kmeans = KMeans(n_clusters=k, random_state=0).fit(data)
        centroids = kmeans.cluster_centers_

        if len(centroids) < 4:
            raise ValueError("Not enough clusters found to form a quadrilateral.")

        return kmeans.labels_, centroids

    # For additional graphical information, print the angle of the centroids
    def calculate_directions(self, centroids):
        directions = np.arctan2(centroids[:, 1], centroids[:, 0])  # Angle in radians
        return np.degrees(directions)  # Convert to degrees

    # Uses a convex hull to see if the origin is within the scanned quadrilateral
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

        if is_point_inside_polygon(centroids[hull.vertices], [0, 0]):
           self.LEDs = np.array([1, 1, 1, 1, 1, 1, 1, 1])
           self.car.read_write_std(0, 0, self.LEDs)
        else:
            self.LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
            self.car.read_write_std(0, 0, self.LEDs)

    # Use the LiDAR sensor to create mapping and perform KMeans and rendezvous check
    def detect_rendezvous(self):
        try:
            x_min, x_max = -2.0, 2.0
            y_min, y_max = -2.0, 2.0

            while True:
                self.lidar.read()
                distances = self.lidar.distances.flatten()
                angles = np.linspace(0, 2 * np.pi, self.numMeasurements)

                x = distances * np.cos(angles)
                y = distances * np.sin(angles)
                data = np.vstack((x, y)).T

                if data.shape[0] == 0:
                    print("No data points received from LiDAR.")
                    continue

                if len(data) == 0:
                    raise ValueError("Data cannot be empty. Please provide valid data.")

                filtered_data = self.filter_data(data)

                try:
                    labels, centroids = self.cluster_and_find_centroids(filtered_data, k=4)
                except ValueError as e:
                    print(f"Clustering error: {e}")
                    continue

                directions = self.calculate_directions(centroids)
                for i, direction in enumerate(directions):
                    print(f"Post {i + 1} direction: {direction:.2f} degrees")

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
            print(f"Failed to read from lidar: {e}")
        finally:
            self.lidar.terminate()

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
