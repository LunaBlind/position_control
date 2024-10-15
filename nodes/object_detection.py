#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import rclpy.time
from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Pose

import numpy as np
import cv2

import tf_transformations
import tf2_geometry_msgs
from tf2_ros import TransformListener, Buffer

class AprilTagPoseSubscriber(Node):
    def __init__(self):
        super().__init__('apriltag_pose_subscriber')
        self.subscription = self.create_subscription( msg_type=AprilTagDetectionArray,
                                                    topic='detections',
                                                    callback=self.listener_callback,
                                                    qos_profile=10
        )
        self.camera_info_sub = self.create_subscription( msg_type=CameraInfo,
                                                    # topic='front_camera/camera_info',
                                                    topic='synchronized_camera_info',
                                                    callback=self.set_camera_matrices,
                                                    qos_profile=10)

        self.object_grabbed_pub = self.create_publisher(msg_type=Bool,
                                                  topic='object_grabbed',
                                                  qos_profile=1)

        self.object_pose_pub = self.create_publisher(msg_type=PoseStamped,
                                                  topic='object_pose',
                                                  qos_profile=1)

        self.object_pose_camera_frame_pub = self.create_publisher(msg_type=PoseStamped,
                                                  topic='object_pose_camera_frame',
                                                  qos_profile=1)
        # Initialize the tf2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Neutral/default camera matrix (identity matrix for no distortion)
        self.camera_matrix = np.eye(3, dtype=np.float32)

        self.gripper_threshold = 0.25
        self.gripper_state = 'closed'
        self.unknown_state_start_time = None

        self.unknown_start_time = None
        self.unknown_threshold = 4

        # Define hardcoded open and closed gripper poses found in rviz2
        gripper_open_position = np.array([-0.018, 0.097, 0.356])
        gripper_open_orientation = np.array([-0.59, -0.255, -0.296, 0.7])
        self.T_open = np.eye(4)
        # Convert quaternion to rotation matrix
        rotation_matrix = tf_transformations.quaternion_matrix(gripper_open_orientation)[:3, :3]  # 3x3 rotation matrix
        self.T_open[:3, :3] = rotation_matrix
        self.T_open[:3, 3] = gripper_open_position.squeeze()

        gripper_closed_position = np.array([-0.0457, 0.0985, 0.379])
        gripper_closed_orientation = np.array([-0.454, -0.444, -0.546, 0.545])
        self.T_closed = np.eye(4)
        # Convert quaternion to rotation matrix
        rotation_matrix = tf_transformations.quaternion_matrix(gripper_closed_orientation)[:3, :3]  # 3x3 rotation matrix
        self.T_closed[:3, :3] = rotation_matrix
        self.T_closed[:3, 3] = gripper_closed_position.squeeze()

        # Neutral/default distortion coefficients (zero distortion)
        self.dist_coeffs = None  # or np.zeros(8, dtype=np.float32) depending on the model

    def set_camera_matrices(self, camera_info):
        self.camera_matrix = np.array(camera_info.k).reshape(3,3)
        self.dist_coeffs = np.array(camera_info.d)
    
    def listener_callback(self, msg):
        if self.gripper_state == 'unknown':
            if (msg.header.stamp.sec - self.unknown_state_start_time) >= self.unknown_threshold:
                self.gripper_state = 'holding'
                self.publish_gripper_state()
        if msg.detections:
            for detection in msg.detections:
                tag_size = 0.02
                if detection.id == 101:
                    tag_size = 0.035

                object_points = np.array([
                    [-tag_size / 2, tag_size / 2, 0],
                    [ tag_size / 2, tag_size / 2, 0],
                    [ tag_size / 2,  -tag_size / 2, 0],
                    [-tag_size / 2,  -tag_size / 2, 0]
                ], dtype=np.float32)

                # Assuming corners are provided in the apriltag_msgs/Point format (x, y)
                image_points = np.array([
                    [detection.corners[0].x, detection.corners[0].y],
                    [detection.corners[1].x, detection.corners[1].y],
                    [detection.corners[2].x, detection.corners[2].y],
                    [detection.corners[3].x, detection.corners[3].y]
                ], dtype=np.float32)
                # Use solvePnP to estimate the pose
                # cv2.SOLVEPNP_IPPE_SQUARE is specificially for tags 
                success, rvec, tvec = cv2.solvePnP(
                        object_points, image_points,
                        self.camera_matrix, self.dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE)

                if success == True:
                    rotation_matrix, _ = cv2.Rodrigues(rvec)
                    # quaternion = tf_transformations.quaternion_from_matrix(np.hstack((rotation_matrix, tvec.reshape(3, 1))))
                    
                    # self.get_logger().info(f"Quaternion: {quaternion}")
                    # current_gripper_pose = self.create_pose_msg(tvec, quaternion)
                    T_current = np.eye(4)
                    T_current[:3, :3] = rotation_matrix
                    T_current[:3, 3] = tvec.squeeze()

                    if detection.id == 100:
                        # self.get_logger().info(f"Transformation matrix: {T_current}")
                        self.transform_matrix_to_pose(T_current)

                    # rvec is the rotation vector, tvec is the translation vector
                    # self.get_logger().info(f"Rotation Vector: {rvec}")
                    # self.get_logger().info(f"Translation Vector: {tvec}")
                    # self.get_logger().info(f"Tag ID: {detection.id}")
                    # self.get_logger().info(f"Rotation Vector: {rvec}")
                    if detection.id == 101:
                        difference_open = np.linalg.norm(T_current - self.T_open, 'fro')
                        difference_closed = np.linalg.norm(T_current - self.T_closed, 'fro')
                        # self.get_logger().info(f"Current Transform: {T_current}")
                        # self.get_logger().info(f"Open Transform: {self.T_open}")
                        # self.get_logger().info(f"Closed Transform: {self.T_closed}")
                        # self.get_logger().info(f"difference open: {difference_open}")
                        # self.get_logger().info(f"difference closed: {difference_closed}")

                        if difference_open < self.gripper_threshold:
                            new_state = 'open'
                        elif difference_closed < self.gripper_threshold:
                            new_state = 'closed'
                        else:
                            new_state = 'unknown'
                        self.get_logger().info(f"gripper stae: {new_state}")
                        # self.get_logger().info(f"Time: {msg.header.stamp.sec}")
                        self.update_state(new_state, msg.header.stamp.sec)
                        self.publish_gripper_state()
                    # self.get_logger().info(f"Translation Vector: {tvec}")
                    # self.get_logger().info(f"New state: {new_state}")
                    # self.get_logger().info(f"Gripper state: {self.gripper_state}")
                    # self.get_logger().info(f"Gripper state: {msg.header.stamp.sec}")

    def update_state(self, new_state, current_timestamp: Time):
        if new_state == 'unknown' and self.gripper_state != 'holding':
            if self.gripper_state != 'unknown':
                self.unknown_state_start_time = current_timestamp
                self.gripper_state = 'unknown'
            elif (current_timestamp - self.unknown_state_start_time) >= self.unknown_threshold:
                self.get_logger().info(f"time diff: {current_timestamp - self.unknown_state_start_time}")
                self.gripper_state = 'holding'
        elif new_state == 'open':
            self.gripper_state = 'open'
        elif new_state == 'closed':
            self.gripper_state = 'closed'
        else:
            if self.gripper_state == 'holding':
                if new_state == 'open' or new_state == 'closed':
                    self.gripper_state = new_state
    def publish_gripper_state(self):
        grab_msg = Bool()
        if self.gripper_state == 'holding':
            grab_msg.data = True
        else:
            grab_msg.data = False
        self.object_grabbed_pub.publish(grab_msg)


    def create_pose_msg(self, tvec, quaternion):
        # Create a Pose message
        pose = Pose()

        # Set position from the translation vector (tvec)
        pose.position.x = tvec[0]
        pose.position.y = tvec[1]
        pose.position.z = tvec[2]

        # Set orientation from the quaternion
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        return pose

    def transform_matrix_to_pose(self, transformation_matrix):
        # Extract the translation (x, y, z) from the transformation matrix
        translation = tf_transformations.translation_from_matrix(transformation_matrix)
        x, y, z = translation

        # Extract the rotation (quaternion) from the transformation matrix
        quaternion = tf_transformations.quaternion_from_matrix(transformation_matrix)
        qx, qy, qz, qw = quaternion

        # Create a Pose message
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        # Get the current time
        now = rclpy.time.Time()
        # self.get_logger().info(f"Tag Pose in front_camera Frame: {pose.position}")
        pose_msg = PoseStamped()
        pose_msg.pose = pose

            # Set the header
        pose_msg.header.stamp = now.to_msg()  # Current time
        pose_msg.header.frame_id = "bluerov00/front_camera"  # Reference frame
        # pose_msg.header.stamp = now
        self.object_pose_camera_frame_pub.publish(pose_msg)

        # Transform the pose from the tag frame to the world frame
        transform = self.tf_buffer.lookup_transform('bluerov00/base_link',  # Target frame
                                                    # msg.header.frame_id,  # Source frame
                                                    'bluerov00/front_camera',  # Source frame
                                                    # 'front_camera_link',  # Source frame
                                                    now)  # Time

        # Apply the transform to get the pose in the world frame
        pose_base_link = tf2_geometry_msgs.do_transform_pose(pose, transform)
        # self.get_logger().info(f"Tag Pose in base_link Frame: {pose_base_link.position}")

        # Transform the pose from the tag frame to the world frame
        transform = self.tf_buffer.lookup_transform('map',  # Target frame
                                                    # msg.header.frame_id,  # Source frame
                                                    'bluerov00/front_camera',  # Source frame
                                                    # 'front_camera_link',  # Source frame
                                                    now)  # Time

        # Apply the transform to get the pose in the world frame
        pose_world = tf2_geometry_msgs.do_transform_pose(pose, transform)

        # Now you have the pose in the world frame
        # self.get_logger().info(f"Tag Pose in Map Frame: {pose_world.position}")
        pose_msg = PoseStamped()
        # pose_msg.pose = pose_base_link
        pose_msg.pose = pose_world

            # Set the header
        pose_msg.header.stamp = now.to_msg()  # Current time
        pose_msg.header.frame_id = "map"  # Reference frame
        # pose_msg.header.frame_id = "bluerov00/base_link"  # Reference frame
        # pose_msg.header.stamp = now
        self.object_pose_pub.publish(pose_msg)

        # return pose

def main():
    rclpy.init()
    node = AprilTagPoseSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
