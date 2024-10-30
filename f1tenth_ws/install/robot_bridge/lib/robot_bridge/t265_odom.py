#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Empty
from nav_msgs.msg import Odometry
import robot_class
import numpy as np
import time

class T265Pub(Node):

    def __init__(self):
        super().__init__('t265_odom_publisher')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.create_subscription(Odometry, '/camera/pose/sample', self.odom_callback, qos_policy)
        self.create_subscription(Odometry, '/odom', self.wheel_odom_callback, qos_policy)
        self.create_subscription(Empty, "calibrate_odom", self.cal_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 't265_odom', 10)
        self.robot = robot_class.robot_class()
        self.T_t265_base = self.robot.create_transformation( -0.30, 0.0, 0.0 )
        self.data = []
        self.max_data = 2000
        self.i = 0
        self.state = "calibration"
        self.mean = []
        self.max_value = []
        self.cov = []
        self.isMove = False
        self.timeoout = time.time()
        self.vx_wheel = 0.0
    
    def wheel_odom_callback(self, msg:Odometry):
        if abs( msg.twist.twist.linear.x ) >= 0.001 or abs( msg.twist.twist.angular.z ) >= 0.001:
            self.isMove = True
            self.timeoout = time.time()
            self.vx_wheel = msg.twist.twist.linear.x
        else:
            if time.time() - self.timeoout >= 2.0:
                self.isMove = False
                
    def cal_callback(self, msg):
        self.state = "calibration"
        self.i = 0
    def odom_callback(self, msg:Odometry):
        # print(msg)
        stamp = self.get_clock().now().to_msg()
        T = self.robot.pose_stamped_to_matrix(msg.pose)
        T_new = T @ self.T_t265_base
        pose = self.robot.matrix_to_pose_stamped(T_new, "odom", stamp)
        cov_pose = 0.05
        twist_cov = 0.1
        min_vx = 0.035
        if self.state == "calibration":
            if self.i < self.max_data:
                buffer = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z, 
                        msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]
                self.data.append( buffer )
                self.i = self.i + 1
            else:
                data_array = np.array(self.data)
                mean = np.mean( data_array, 0 )
                max_value = np.max( data_array, 0)
                cov = np.cov( data_array.T )
                
                self.mean = mean
                self.max_value = max_value
                self.cov = cov
                self.state = "send_data"
                # print("cov")
                # print(self.cov)
                # print("mean")
                # print(self.mean)
        elif self.state == "send_data":
            
            # print(msg.twist.twist.angular.x)
            msg.child_frame_id = "base_footprint"
            msg.header.frame_id = "odom"
            msg.header.stamp = stamp
            msg.twist.twist.linear.x = msg.twist.twist.linear.x - self.mean[0]
            msg.twist.twist.linear.y = msg.twist.twist.linear.y - self.mean[1]
            msg.twist.twist.linear.z = msg.twist.twist.linear.z - self.mean[2]
            msg.twist.twist.angular.x = msg.twist.twist.angular.x - self.mean[3]
            msg.twist.twist.angular.y = msg.twist.twist.angular.y - self.mean[4]
            msg.twist.twist.angular.z = msg.twist.twist.angular.z - self.mean[5]
            # print("threshold = ", self.cov[0,0] * 6 )
            # print(msg.twist.twist.linear.x )
            
            
            if abs(msg.twist.twist.linear.x) <= 0.1:
                # msg.twist.twist.linear.x = 0.0
                msg.twist.twist.linear.x = self.vx_wheel
            if abs(msg.twist.twist.angular.z) <= 0.008:
                msg.twist.twist.angular.z = 0.0
            
            # if abs(msg.twist.twist.linear.x) > 0.008 and abs(msg.twist.twist.linear.x) < 0.05:
            #     msg.twist.twist.linear.x = self.vx_wheel
            
            if self.isMove == False:
                msg.twist.twist.linear.x = 0.0
                msg.twist.twist.angular.z = 0.0
            # msg.twist.twist.linear.x = msg.twist.twist.linear.x
            msg.pose.pose.position = pose.pose.position
            msg.pose.pose.orientation = pose.pose.orientation
            msg.pose.covariance = [ cov_pose, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, cov_pose, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, cov_pose, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, cov_pose, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, cov_pose, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, cov_pose]
            msg.twist.covariance = [ twist_cov, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, twist_cov, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, twist_cov, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, twist_cov, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, twist_cov, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, twist_cov]
            # msg.twist.covariance = [ self.cov[0,0], 0.0, 0.0, 0.0, 0.0, 0.0,
            #                         0.0, self.cov[1,1], 0.0, 0.0, 0.0, 0.0,
            #                         0.0, 0.0, self.cov[2,2], 0.0, 0.0, 0.0,
            #                         0.0, 0.0, 0.0, self.cov[3,3], 0.0, 0.0,
            #                         0.0, 0.0, 0.0, 0.0, self.cov[4,4], 0.0,
            #                         0.0, 0.0, 0.0, 0.0, 0.0, self.cov[5,5]]
            # msg.twist.covariance = self.cov
            # print(msg.twist.twist.linear)
            # print(T_new)
            self.odom_pub.publish(msg)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = T265Pub()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()