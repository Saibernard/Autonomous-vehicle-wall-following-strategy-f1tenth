#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import time
from math import isfinite

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers
        self.pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.sub = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)

        # TODO: set PID gains
        self.kp = 5.0
        self.kd = 1.0
        self.ki = 0.001 #1.0 #5/1e4

        # TODO: store history
        self.integral = 0.0
        self.prev_error = 0.0
        self.error = 0.0

        # TODO: store any necessary values you think you'll need
        self.dist_des = 1.0
        self.L = 1.0
        self.prev_time = time.time()
        self.dt = 0.0
        self.i = 0.0

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        #TODO: implement
        index = int(np.floor((angle - range_data.angle_min) / range_data.angle_increment))
        return range_data.ranges[index]

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        #TODO:implement
        angle_b = np.pi/2
        angle_a = np.pi/4
        while (not isfinite(self.get_range(range_data, angle_a))) or (angle_a == angle_b):
            print('skipping a')
            angle_a += 0.02

        while (not isfinite(self.get_range(range_data, angle_b))) or (angle_a == angle_b):
            print('skipping b')
            angle_b += 0.02

        a = self.get_range(range_data, angle_a)
        b = self.get_range(range_data, angle_b)
        theta = angle_b - angle_a
        alpha = np.arctan2((a * np.cos(theta) - b), (a * np.sin(theta)))
        D_t = b * ( np.cos(alpha))
        D_t_plus_1 = D_t + self.L * np.sin(alpha)
        error_t_plus_1 = self.dist_des - D_t_plus_1
        self.prev_error = self.error
        self.error = error_t_plus_1
        return error_t_plus_1

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        angle = 0.0
        # TODO: Use kp, ki & kd to implement a PID controller
        self.integral = self.prev_error * self.dt
        p_term = self.kp * error
        d_term = self.kd * ((error - self.prev_error)/self.dt)
        i_term = self.ki*self.integral
        angle = -(p_term + d_term + i_term)
        abs_angle = abs(angle)
        # print(f'p_term{p_term}  ', f'i_term{i_term}  ', f'd_term{d_term}')
        # print(f'steering_angle: {angle}')
        if 0 <= abs_angle < 10*(np.pi/180):
            velocity = 1.5
        elif 10*(np.pi/180) <= abs_angle < 20*(np.pi/180):
            velocity = 1.0
        else:
            velocity = 0.5
        # print(f'velocity:{velocity}')
        drive_msg = AckermannDriveStamped()
        # TODO: fill in drive message and publish
        drive_msg.drive.steering_angle = angle #radians
        drive_msg.drive.speed = velocity  #m/s
        self.pub.publish(drive_msg)

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        error = self.get_error(msg, self.dist_des) # TODO: replace with error calculated by get_error()
        velocity = 0.0 # TODO: calculate desired car velocity based on error
        curr_time = time.time()
        self.dt = curr_time - self.prev_time
        # print(f'dt:{self.dt}')
        self.pid_control(error, velocity)  # TODO: actuate the car with PID
        self.prev_time = curr_time


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()