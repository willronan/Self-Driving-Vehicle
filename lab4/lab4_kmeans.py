'''lab4_kmeans.py

This walkthough uses lidar sensors
with unsupervised learning in the
form of k-means clustering to predict
the location of 4 posts around the 
sensor

'''


import numpy as np
import matplotlib.pyplot as plt
import time
import os
import sys
import signal
from enum import Enum
from sklearn.cluster import KMeans

from pal.products.qcar import QCar
from pal.utilities.lidar import Lidar


class RPLidar:

    def __init__(self):

        #-------- 1.1 Intialize the LiDAR -------#






        #----------------------------------------#
       
        signal.signal(signal.SIGINT, self.signal_handler)

        self.fig, self.ax = plt.subplots()
        self.fig.show()
        self.fig.canvas.draw()


    def filter_data(self, data, min_distance=0.05, max_distance=1.0):
        if data.shape[0] == 0:
            raise ValueError("No data points available to filter.")
        
        distances = np.linalg.norm(data, axis=1)
        print(f"Min distance before filtering: {distances.min():.4f}, Max distance before filtering: {distances.max():.4f}")
        mask = (distances >= min_distance) & (distances <= max_distance)
        filtered_data = data[mask]

        if filtered_data.shape[0] == 0:
            raise ValueError("No data points left after filtering.")

        return filtered_data
    

    def cluster_and_find_centroids(self, data, k=4):
        if data.shape[0] < k:
            raise ValueError(f"Not enough data points to form {k} clusters.")
        

        #--- 1.3 Perform k means algorithm ---#






    
        #----------------------------------------#


    def calculate_directions(self, centroids):
        directions = np.arctan2(centroids[:, 1], centroids[:, 0])  # Angle in radians
        return np.degrees(directions)  # Convert to degrees
    

    def detect_obstacles(self):
        try:
            # Define fixed limits for the plot
            x_min, x_max = -2.0, 2.0
            y_min, y_max = -2.0, 2.0
            
            while True:

                #--- 1.2 Convert to polar coordinates ---#







                #----------------------------------------#

                if data.shape[0] == 0:
                    print("No data points received from LiDAR.")
                    continue

                try:
                    filtered_data = self.filter_data(data)
                except ValueError as e:
                    print(f"Filtering error: {e}")
                    continue

                try:
                    labels, centroids = self.cluster_and_find_centroids(filtered_data, k=4)
                except ValueError as e:
                    print(f"Clustering error: {e}")
                    continue

                unique_labels = np.unique(labels)
                cluster_sizes = [np.sum(labels == label) for label in unique_labels]
                
                if len(unique_labels) < 4:
                    print("Not enough unique clusters to identify four largest clusters.")
                    continue

                largest_labels = np.argsort(cluster_sizes)[-4:]
                largest_centroids = centroids[largest_labels]

                os.system("clear")

                try:
                    directions = self.calculate_directions(largest_centroids)
                    for i, (centroid, direction) in enumerate(zip(largest_centroids, directions)):
                        print(f"Post {i + 1} has direction {direction:.2f} degrees")
                except ValueError as e:
                    print(f"Direction calculation error: {e}")
                    continue

                self.ax.clear()


                # Plotting 
                self.ax.scatter(filtered_data[:, 0], filtered_data[:, 1], c='gray', s=1, label='Detected Points')
                self.ax.scatter(centroids[:, 0], centroids[:, 1], c='blue', s=50, marker='x', label='All Centroids')
                self.ax.scatter(largest_centroids[:, 0], largest_centroids[:, 1], c='red', s=100, marker='o', label='Selected Pillars')

                if len(largest_centroids) == 4:
                    sorted_indices = np.argsort(np.arctan2(largest_centroids[:, 1], largest_centroids[:, 0]))
                    sorted_vertices = largest_centroids[sorted_indices]
                    sorted_vertices = np.vstack((sorted_vertices, sorted_vertices[0]))
                    self.ax.plot(sorted_vertices[:, 0], sorted_vertices[:, 1], 'r-')

                for centroid, direction in zip(largest_centroids, directions):
                    self.ax.annotate(f'{direction:.1f}°', xy=centroid, textcoords='offset points', xytext=(0,10), ha='center')

                # Set fixed axis limits and aspect ratio
                self.ax.set_xlim(x_min, x_max)
                self.ax.set_ylim(y_min, y_max)
                self.ax.set_aspect('equal')

                # Add an arrow to indicate vehicle position and direction
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


    def signal_handler(self, sig, frame):
        print("Exiting gracefully...")
        self.destroy()
        sys.exit(0)


    def destroy(self):
        self.lidar.terminate()


def main():
    rplidar = RPLidar()
    rplidar.detect_obstacles()

if __name__ == '__main__':
    main()
