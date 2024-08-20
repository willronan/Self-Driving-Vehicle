# Standalone LiDAR Detection Script for Quadrilateral Area Calculation

# Standard imports
import numpy as np
import matplotlib.pyplot as plt
import time
import os
import sys
import signal
from enum import Enum
from sklearn.cluster import KMeans

import traceback

# Quanser imports (Make sure these libraries are installed and accessible)
from pal.products.qcar import QCar
from pal.utilities.lidar import Lidar

class Obstical(Enum):
    FRONT_AND_REAR = 1
    FRONT = 2
    REAR = 3
    NONE = 0

class RPLidar:

    def __init__(self):


        #-------- 1.1 SETUP LIDAR SENSOR ---------#


        # Initialize LIDAR
        self.numMeasurements = 720  # Points
        self.lidarMeasurementMode = 2
        self.lidarInterpolationMode = 0

        # Connect to LIDAR sensor
        self.lidar = Lidar(
            type='RPLidar',
            numMeasurements=self.numMeasurements,
            rangingDistanceMode=self.lidarMeasurementMode,
            interpolationMode=self.lidarInterpolationMode
        )

        #----------------------------------------#

        # Prepare system exit procedure
        signal.signal(signal.SIGINT, self.signal_handler)

        # Initialize plot
        self.fig, self.ax = plt.subplots()
        self.fig.show()
        self.fig.canvas.draw()

    def filter_data(self, data, distance_threshold=1.0, min_distance=0.05):
        """Filter data points based on a distance threshold and a minimum distance."""
        distances = np.sqrt(np.sum(data**2, axis=1))  # Calculate the distance from the origin
        filtered_data = data[(distances < distance_threshold) & (distances > min_distance)]
        return filtered_data


    def cluster_and_find_centroids(self, data, k=4):
        """Apply K-Means clustering to find centroids."""

        #-------- 1.3 SETUP LIDAR SENSOR --------#

        kmeans = KMeans(n_clusters=k, random_state=0).fit(data)
        centroids = kmeans.cluster_centers_
        return kmeans.labels_, centroids
    
        #----------------------------------------#

    def calculate_quadrilateral_area(self, vertices):
        """Calculate the area of a quadrilateral given its vertices using the Shoelace formula."""
        if len(vertices) != 4:
            raise ValueError("There must be exactly 4 vertices to calculate area of a quadrilateral.")
        
        x = vertices[:, 0]
        y = vertices[:, 1]
        area = 0.5 * abs(
            x[0]*y[1] + x[1]*y[2] + x[2]*y[3] + x[3]*y[0] - 
            (y[0]*x[1] + y[1]*x[2] + y[2]*x[3] + y[3]*x[0])
        )
        return area

    def detect_obstacles(self):
        try:
            while True:

                #-------- 1.2 PERFORM SCAN --------#

                # Preprocess data
                self.lidar.read()

                distances = self.lidar.distances.flatten()
                angles = np.linspace(0, 2 * np.pi, self.numMeasurements)

                # Convert to cartesian coordinates
                x = distances * np.cos(angles)
                y = distances * np.sin(angles)
                
                data = np.vstack((x, y)).T

                #----------------------------------------#

                # Filter data
                filtered_data = self.filter_data(data)

                # Apply clustering algorithm
                if filtered_data.size > 4:  # Check if filtered_data is not empty
                    labels, centroids = self.cluster_and_find_centroids(filtered_data, k=4)



                    # Calculate the area of the quadrilateral
                    try:
                        area = self.calculate_quadrilateral_area(centroids)
                        print(f"Area of the quadrilateral formed by the posts: {area:.2f} square units")
                    except ValueError:
                        print("Not enough centroids to calculate area.")

                    # Clear the previous plot
                    self.ax.clear()

                    # Plot data
                    self.ax.scatter(filtered_data[:, 0], filtered_data[:, 1], c='gray', s=1, label='Detected Points')
                    self.ax.scatter(centroids[:, 0], centroids[:, 1], c='blue', s=50, marker='x', label='All Centroids')
                    self.ax.scatter(centroids[:, 0], centroids[:, 1], c='red', s=100, marker='o', label='Selected Pillars')

                    # Connect the centroids with lines
                    if len(centroids) == 4:
                        # Sort the vertices to form a closed quadrilateral
                        sorted_indices = np.argsort(np.arctan2(centroids[:, 1], centroids[:, 0]))
                        sorted_vertices = centroids[sorted_indices]
                        sorted_vertices = np.vstack((sorted_vertices, sorted_vertices[0]))  # Close the polygon
                        self.ax.plot(sorted_vertices[:, 0], sorted_vertices[:, 1], 'r-')

                    # Add an arrow to indicate vehicle position and direction
                    arrowprops = dict(facecolor='red', shrink=0.05, width=1.0, headwidth=3.0)
                    self.ax.annotate('', xy=(0, 1), xytext=(0, 0), arrowprops=arrowprops)

                    self.ax.set_xlabel('X')
                    self.ax.set_ylabel('Y')
                    self.ax.set_title('Lidar Data Clustering')
                    self.ax.legend()


                else:
                    print("No data available for clustering.")

                # Draw and pause
                self.fig.canvas.draw()
                plt.pause(0.1)

        except Exception as e:
            print(f"Failed to read from lidar: {e}")
            traceback.print_exc()
        finally:
            self.lidar.terminate()  # Ensure the LiDAR is terminated

    # Handle keyboard interrupt 
    def signal_handler(self, sig, frame):
        print("Exiting gracefully...")
        self.destroy()
        sys.exit(0)

    # Safely disconnect hardware
    def destroy(self):
        self.lidar.terminate()  # Properly terminate QCar instance

# Main function to run the standalone script
def main():
    rplidar = RPLidar()
    rplidar.detect_obstacles()

if __name__ == '__main__':
    main()
