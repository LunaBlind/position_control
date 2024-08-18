#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from hippo_msgs.msg import ActuatorSetpoint, Float64Stamped
from std_msgs.msg import  Bool
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf_transformations import euler_from_quaternion

import rclpy.time


class YawController(Node):

    def __init__(self):
        super().__init__(node_name='yaw_controller')
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST,
                         depth=1)

        # default value for the yaw setpoint
        self.setpoint_yaw = math.pi / 2.0
        self.setpoint_pitch = 0
        self.setpoint_roll = 0

        self.p_gain_yaw = 0.25  # turned out to be a good value
        self.p_gain_pitch = 0.2  # turned out to be a good value
        self.p_gain_roll = 0.2  # turned out to be a good value

        self.yaw_offset = 0
        self.pitch_offset = -0.35
        self.roll_offset = 0.025

        self.vision_pose_sub = self.create_subscription(
            msg_type=PoseWithCovarianceStamped,
            topic='vision_pose_cov',
            callback=self.on_vision_pose,
            qos_profile=qos)
        self.setpoint_sub = self.create_subscription(Float64Stamped,
                                                     topic='~/setpoint',
                                                     callback=self.on_setpoint,
                                                     qos_profile=qos)

        self.object_grabbed_sub = self.create_subscription(msg_type=Bool,
                                                  topic='object_grabbed',
                                                  callback=self.adjust_offsets,
                                                  qos_profile=1)

        self.torque_pub = self.create_publisher(msg_type=ActuatorSetpoint,
                                                topic='torque_setpoint',
                                                qos_profile=1)

    def adjust_offsets(self, msg):
        if msg.data == True:
            # weight of object 125g -> ~7g -0.01 offset
            self.pitch_offset = -0.52

        else:
            self.pitch_offset = -0.35

    def wrap_pi(self, value: float):
        """Normalize the angle to the range [-pi; pi]."""
        if (-math.pi < value) and (value < math.pi):
            return value
        range = 2 * math.pi
        num_wraps = math.floor((value + math.pi) / range)
        return value - range * num_wraps

    def on_setpoint(self, msg: Float64Stamped):
        self.setpoint = self.wrap_pi(msg.data)

    def on_vision_pose(self, msg: PoseWithCovarianceStamped):
        # get the vehicle orientation expressed as quaternion
        q = msg.pose.pose.orientation
        # convert the quaternion to euler angles
        (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw = self.wrap_pi(yaw)
        pitch = self.wrap_pi(pitch)
        roll = self.wrap_pi(roll)

        control_output = self.compute_control_output(yaw, pitch, roll)
        timestamp = rclpy.time.Time.from_msg(msg.header.stamp)
        self.publish_control_output(control_output, timestamp)

    def compute_control_output(self, yaw, pitch, roll):
        # very important: normalize the angle error!
        error_yaw = self.wrap_pi(self.setpoint_yaw - yaw)
        error_pitch = self.wrap_pi(self.setpoint_pitch - pitch)
        error_roll = self.wrap_pi(self.setpoint_roll - roll)

        control_output = [self.p_gain_pitch * error_pitch + self.pitch_offset,
                          self.p_gain_yaw * error_yaw,
                          self.p_gain_roll * error_roll + self.roll_offset]
        return control_output

    def publish_control_output(self, control_output: list,
                               timestamp: rclpy.time.Time):
        msg = ActuatorSetpoint()
        msg.header.stamp = timestamp.to_msg()
        msg.ignore_x = False
        msg.ignore_y = False  # pitch is rotation around y
        msg.ignore_z = False  # yaw is the rotation around the vehicle's z axis

        msg.y = control_output[0]
        msg.z = control_output[1]
        msg.x = control_output[2]
        self.torque_pub.publish(msg)


def main():
    rclpy.init()
    node = YawController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
