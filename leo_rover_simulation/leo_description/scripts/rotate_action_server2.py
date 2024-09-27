#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from leo_description.action import Rotate
from rclpy.action import ActionServer

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sin, cos
import threading
import time

# Import euler_from_quaternion
from tf_transformations import euler_from_quaternion

class RotateActionServer(Node):

    def __init__(self):
        super().__init__('rotate_action_server')

        # Create an action server
        self._action_server = ActionServer(
            self,
            Rotate,
            'rotate',
            self.execute_callback)

        # Publisher to send velocity commands
        self._cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        # Subscriber to receive odometry data
        self._odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        self.current_yaw = 0.0

    def odom_callback(self, msg):
        # Extract yaw from quaternion
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w]

        # Use tf_transformations to convert quaternion to Euler angles
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw

        print("CURRENT YAW!!", self.current_yaw)
        # Optional: Add logging to verify current_yaw
        #self.get_logger().info(f'Current yaw updated: {self.current_yaw:.4f}')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        target_angle = goal_handle.request.target_angle
        angular_speed = 0.2  # radians per second

        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = angular_speed if target_angle > 0 else -angular_speed

        initial_yaw = self.current_yaw

        target_yaw = initial_yaw + target_angle

        # Normalize target_yaw to [-pi, pi]
        target_yaw = atan2(sin(target_yaw), cos(target_yaw))

        while rclpy.ok():
            # Check for cancellation request
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled')
                twist_msg.angular.z = 0.0
                self._cmd_vel_publisher.publish(twist_msg)
                goal_handle.canceled()
                return Rotate.Result(success=False)

            self._cmd_vel_publisher.publish(twist_msg)

            current_yaw = self.current_yaw

            # Calculate the angle rotated so far
            #print("Current angle:", current_yaw, initial_yaw)
            angle_rotated = current_yaw - initial_yaw
            angle_rotated = atan2(sin(angle_rotated), cos(angle_rotated))

            # Publish feedback
            feedback_msg = Rotate.Feedback()
            feedback_msg.current_angle = angle_rotated
            goal_handle.publish_feedback(feedback_msg)

            # Add logging to verify angle_rotated
            self.get_logger().info(f'Feedback: current_angle = {angle_rotated:.4f}')

            # Calculate the remaining angle
            remaining_angle = target_yaw - current_yaw
            remaining_angle = atan2(sin(remaining_angle), cos(remaining_angle))

            # Check if the rotation is complete
            if abs(remaining_angle) < 0.01:
                self.get_logger().info('Rotation complete')
                break

            # Sleep for a short duration
            time.sleep(0.1)

        # Stop the robot
        twist_msg.angular.z = 0.0
        self._cmd_vel_publisher.publish(twist_msg)

        goal_handle.succeed()
        result = Rotate.Result()
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = RotateActionServer()
    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
