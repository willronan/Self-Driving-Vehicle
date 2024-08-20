'''
ROS Lane Detection Node

Isolates the lane using CV2 filtration.
Calculates the slope of the line.
Computes appropriate motor commands
using PI control.

'''


# ROS imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# Quanser imports
from pal.utilities.gamepad import LogitechF710
from pal.products.qcar import QCar


# Command Coefficients
STEERING_COEF = 0.5
THROTTLE_COEF = 0.09


# Lane detection Node
class Gamepad_Node(Node):

    def __init__(self):
        super().__init__('gamepad')

        # Create publisher & set camera parameters
        self.publisher_ = self.create_publisher(String, 'gamepad', 10)

        # Connect to remote controller & car hardware
        self.gpad  = LogitechF710()
        self.car = QCar()

        self.steer_car()


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

    # Auto control via computer vison
    def steer_car(self):

        while True:

            # Read drive command from gamepad
            new = self.gpad.read()
            throttle, steering= self.control_from_gamepad(self.gpad.buttonLeft, self.gpad.trigger, self.gpad.leftJoystickX, self.gpad.buttonA)

            # Publish drive command
            msg = String()
            msg.data = f"{throttle},{steering}"
            self.publisher_.publish(msg)
            self.get_logger().info(f"Publishing Motor Cmd {msg.data}")


    # Safely disconnect hardware
    def destroy(self):
        self.car.terminate()  
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    gamepad_node = Gamepad_Node()
    rclpy.spin(gamepad_node)
    gamepad_node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
