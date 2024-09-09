'''
ROS Motor Control Node

Takes in feedback from all
other nodes. Based on sensor
information, executes
correstponding drive commands,

'''

# ROS imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Quanser imports
from pal.products.qcar import QCar

# Standard imports
import sys
import time
import numpy as np


class Motor(Node):

    def __init__(self):

        # Create subscriber & add subscriptions
        super().__init__('motor')
        self.detection_subscription = self.create_subscription(String, 'detection', self.detection_callback, 10)
        self.steering_subscription = self.create_subscription(String, 'steering', self.steering_callback, 10)
        self.lidar_subscription = self.create_subscription(String, 'lidar', self.lidar_callback, 10)
        
        # prevent unused variable warning
        self.detection_subscription  
        self.steering_subscription
        self.lidar_subscription

        # Initialize car
        self.LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        self.car = QCar()

        # Flags and counters
        self.stop_flag = False
        self.ignore_flag = False
        self.ignore_timer = 0
        self.front_obstacle_flag = False
        self.rear_obstacle_flag = False


    # Handle sign detection
    def detection_callback(self, msg):
        if msg.data == "Stop" and self.ignore_flag == False:
            self.stop_flag = True
        elif msg.data == "Go" or self.ignore_flag == True:
            self.stop_flag = False
        else: 
            print("Camera not detected")
            time.sleep(3)
        time.sleep(0.1)

        self.get_logger().info('A : "%s" was detected' % msg.data)

    # Handle lidar detections
    def lidar_callback(self, msg):

        #------- SET FRONT/REAR OBSTACLE FLAGS -------#





            pass
        #---------------------------------------------#


    

    # Handle steering/motor command
    def steering_callback(self, msg):

        ''' The current logic uses the steering topic
            message to write a drive command to the motor.
            If a stop sign is detect, the vehicle stops, 
            waits for 3 seconds, then drives through, 
            ignoring the stop sign.
            
            Add to the logic so that the vehicle cannot drive
            forward if forward/backwards if the front/rear 
            obstacle flags are set:'''
        
        throttle, steer = map(float, msg.data.split(','))

        # No stop sign / already waited, ignoring stop sign
        if self.stop_flag == False:
            # If stop sign is being ignored, stop ignoring after 3 s
            if self.ignore_flag == True:
                if time.time() - self.ignore_timer > 3:
                    self.ignore_flag = False
            # If the command is forwards, and there is no front obstacle, go forwards
            self.car.read_write_std(throttle, steer, self.LEDs)
   
        # If there is a stop sign
        else:
            # Stop
            self.car.read_write_std(0, 0, self.LEDs)
            # Wait 3 s, then continue, ignorin the stop sign 
            if self.stop_flag == True:
                time.sleep(3)
                self.ignore_flag = True
                self.ignore_timer = time.time()
            self.get_logger().info('Motor controls processed')

    # Safely disconnect hardware
    def destroy(self):
        self.LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        self.car.read_write_std(0, 0, self.LEDs)
        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)
    motor = Motor()
    rclpy.spin(motor)
    motor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()