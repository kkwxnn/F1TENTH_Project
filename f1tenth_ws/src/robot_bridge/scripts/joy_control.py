#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import numpy as np

class JoyControl(Node):
    def __init__(self):
        super().__init__('joy_control')
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()
        self.v_max = 2.5 #2.0
        self.w_max = np.tan(0.698) / 0.263

    def joy_callback(self, msg):
        # Use axes[1] for x direction (forward/backward)
        self.twist.linear.x = msg.axes[1] * self.v_max

        # Control steering with axes[4] (left) and axes[5] (right)
        left_turn = msg.axes[4]  # Left turn input
        right_turn = msg.axes[5]  # Right turn input

        # Calculate the angular.z by subtracting right turn from left turn, scaling with w_max
        self.twist.angular.z = (left_turn - right_turn) * self.w_max

        # Publish the Twist message to control the car
        self.pub_cmd.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    joy_control = JoyControl()
    rclpy.spin(joy_control)
    joy_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
