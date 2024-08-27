#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool

import numpy as np
import cv2

class AprilTagPoseSubscriber(Node):
    def __init__(self):
        super().__init__('apriltag_pose_subscriber')
        self.subscription = self.create_subscription( msg_type=AprilTagDetectionArray,
                                                    topic='detections',
                                                    callback=self.listener_callback,
                                                    qos_profile=10
        )
        self.camera_info_sub = self.create_subscription( msg_type=CameraInfo,
                                                    topic='front_camera/camera_info',
                                                    callback=self.set_camera_matrices,
                                                    qos_profile=10)

        self.object_grabbed_pub = self.create_publisher(msg_type=Bool,
                                                  topic='object_grabbed',
                                                  qos_profile=1)
        # self.camera_matrix = np.array([
        #      [467.74270306499267, 0.0, 320.0],
        #      [0.0, 467.74270306499267, 240.0],
        #      [0.0, 0.0, 1.0],
        #     ])
        # Neutral/default camera matrix (identity matrix for no distortion)
        self.camera_matrix = np.eye(3, dtype=np.float32)
        # self.T_closed = np.array([
        #                         [0.00854252, -0.99995771, -0.00340785, -0.091],
        #                          [-0.17625282,  0.00184893, -0.98434319, 0.1],
        #                          [0.98430786,  0.00900941, -0.17622957, 0.375],
        #                          [0, 0, 0, 1]])
        self.gripper_threshold = 0.15
        self.gripper_state = 'closed'
        self.unknown_state_start_time = None

        self.unknown_start_time = None
        self.unknown_threshold = 1
        self.T_open = np.array([
                                [-0.00283829, -0.70956404, -0.70463524, 0.0015],
                                [-0.98288965, -0.12779666,  0.13264968, 0.08],
                                [-0.18417348,  0.69295519, -0.69706043, 0.355],
                                 [0, 0, 0, 1]])
        self.T_closed = np.array([
                                [0.00456548, -0.99984658,  0.0169108, -0.05],
                                [-0.98564349, -0.0016451,   0.16883186, 0.07],
                                [-0.16877813, -0.01743882, -0.98549979, 0.4],
                                [0, 0, 0, 1],
                                ])


# Neutral/default distortion coefficients (zero distortion)
        self.dist_coeffs = np.zeros(5, dtype=np.float32)  # or np.zeros(8, dtype=np.float32) depending on the model

        tag_size = 0.03  # in meters

# 3D points in the world (tag corners, counterclockwise from top-left)
        self.object_points = np.array([
            [-tag_size / 2, -tag_size / 2, 0],
            [ tag_size / 2, -tag_size / 2, 0],
            [ tag_size / 2,  tag_size / 2, 0],
            [-tag_size / 2,  tag_size / 2, 0]
        ], dtype=np.float32)

    def set_camera_matrices(self, camera_info):
        self.camera_matrix = np.array(camera_info.k).reshape(3,3)
        self.dist_coeffs = np.array(camera_info.d)
    
    def listener_callback(self, msg):
        if msg.detections:
            for detection in msg.detections:
                if detection.id == 101:
                    tag_size = 0.03
                else:
                    tag_size = 0.04

                self.object_points = np.array([
                    [-tag_size / 2, -tag_size / 2, 0],
                    [ tag_size / 2, -tag_size / 2, 0],
                    [ tag_size / 2,  tag_size / 2, 0],
                    [-tag_size / 2,  tag_size / 2, 0]
                ], dtype=np.float32)

                # Assuming corners are provided in the apriltag_msgs/Point format (x, y)
                image_points = np.array([
                    [detection.corners[0].x, detection.corners[0].y],
                    [detection.corners[1].x, detection.corners[1].y],
                    [detection.corners[2].x, detection.corners[2].y],
                    [detection.corners[3].x, detection.corners[3].y]
                ], dtype=np.float32)
                # Use solvePnP to estimate the pose
                success, rvec, tvec = cv2.solvePnP(
                        self.object_points, image_points,
                        self.camera_matrix, self.dist_coeffs)
                rvec, _ = cv2.Rodrigues(rvec)

                if success and detection.id == 101:
                    # rvec is the rotation vector, tvec is the translation vector
                    # self.get_logger().info(f"Rotation Vector: {rvec}")
                    # self.get_logger().info(f"Translation Vector: {tvec}")
                    T_current = np.eye(4)
                    T_current[:3, :3] = rvec
                    T_current[:3, 3] = tvec.squeeze()
                    # self.get_logger().info(f"Tag ID: {detection.id}")
                    # self.get_logger().info(f"Rotation Vector: {rvec}")
                    difference_open = np.linalg.norm(T_current - self.T_open, 'fro')
                    difference_closed = np.linalg.norm(T_current - self.T_closed, 'fro')

                    if difference_open < self.gripper_threshold:
                        new_state = 'open'
                    elif difference_closed < self.gripper_threshold:
                        new_state = 'closed'
                    else:
                        new_state = 'unknown'
                    self.update_state(new_state, msg.header.stamp.sec)
                    grab_msg = Bool
                    if self.gripper_state == 'holding':
                        grab_msg = True
                    else:
                        grab_msg = False
                    self.object_grabbed_pub.publish(grab_msg)
                    # self.get_logger().info(f"Translation Vector: {tvec}")
                    # self.get_logger().info(f"Transformation matrix: {T_current}")
                    # self.get_logger().info(f"New state: {new_state}")
                    # self.get_logger().info(f"Gripper state: {self.gripper_state}")
                    # self.get_logger().info(f"Gripper state: {msg.header.stamp.sec}")

    def update_state(self, new_state, current_timestamp: Time):
        # Get the current ROS2 time
        # current_time = self.get_clock().now()
        # self.get_logger().info(f"Current time: {current_time}")
        # self.get_logger().info(f"time_stamp: {header_stamp}")

        if new_state == 'unknown' and self.gripper_state != 'holding':
            if self.gripper_state != 'unknown':
                self.unknown_state_start_time = current_timestamp
                self.gripper_state = 'unknown'
            elif (current_timestamp - self.unknown_state_start_time) >= self.unknown_threshold:
                self.gripper_state = 'holding'
        elif new_state == 'open':
            self.gripper_state = 'open'
        elif new_state == 'closed':
            self.gripper_state = 'closed'
        else:
            if self.gripper_state == 'holding':
                if new_state == 'open' or new_state == 'closed':
                    self.gripper_state = new_state

def main():
    rclpy.init()
    node = AprilTagPoseSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
