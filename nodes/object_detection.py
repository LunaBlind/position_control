#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo

import numpy as np
import cv2

class AprilTagPoseSubscriber(Node):
    def __init__(self):
        super().__init__('apriltag_pose_subscriber')
        self.subscription = self.create_subscription(
            msg_type=AprilTagDetectionArray,
            topic='detections',
            callback=self.listener_callback,
            qos_profile=10
        )
        self.camera_info_sub = self.create_subscription(
                msg_type=CameraInfo,
                topic='front_camera/camera_info',
                callback=self.set_camera_matrices,
                qos_profile=10)
        # self.camera_matrix = np.array([
        #      [467.74270306499267, 0.0, 320.0],
        #      [0.0, 467.74270306499267, 240.0],
        #      [0.0, 0.0, 1.0],
        #     ])
        # Neutral/default camera matrix (identity matrix for no distortion)
        self.camera_matrix = np.eye(3, dtype=np.float32)

# Neutral/default distortion coefficients (zero distortion)
        self.dist_coeffs = np.zeros(5, dtype=np.float32)  # or np.zeros(8, dtype=np.float32) depending on the model

        tag_size = 0.02  # in meters

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
                    self.get_logger().info(f"Tag ID: {detection.id}")
                    self.get_logger().info(f"Rotation Vector: {rvec.flatten()}")
                    self.get_logger().info(f"Translation Vector: {tvec.flatten()}")


def main():
    rclpy.init()
    node = AprilTagPoseSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
