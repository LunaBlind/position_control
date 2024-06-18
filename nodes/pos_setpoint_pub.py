#!/usr/bin/env python3
"""
This node computes a square-wave setpoint for the depth controller, i.e.
the setpoint jumps between two different depth values with a set duration.
You can change this code to try out other setpoint functions, e.g. a sin wave.
"""
import rclpy
from geometry_msgs.msg import PoseStamped, PointStamped
from rclpy.node import Node

import rclpy.parameter
import rclpy.time

import numpy as np

class PosSetpointNode(Node):

    def __init__(self):
        super().__init__(node_name='pos_setpoint_publisher')

        self.start_time = self.get_clock().now()

        # change these parameters to adjust the setpoints
        # ... or change implementation details below to achieve other setpoint
        # functions.
        self.setpoints = [[1.0, 1.0, -0.5],
                          [0.5, 2.0, -0.2],
                          [1.0, 1.0, -1.2],
                          [1.0, 2.0, -0.8],
                          ]
        self.current_position = np.zeros(3)
        self.epsilon = 0.2
        self.index = 0

        self.pos_sub = self.create_subscription(msg_type=PoseStamped,
                                                  topic='position_estimate',
                                                  callback=self.set_current_position,
                                                  qos_profile=1)

        self.pos_setpoint_pub = self.create_publisher(msg_type=PointStamped,
                                                        topic='pos_setpoint',
                                                        qos_profile=1)
        self.timer = self.create_timer(timer_period_sec=1 ,
                                       callback=self.timer_callback)

    def timer_callback(self):
        if self.index >= len(self.setpoints):
            self.get_logger().info("All setpoints reached")
            self.timer.cancel()
            return

        setpoint = self.setpoints[self.index]

        now = self.get_clock().now()
        self.publish_setpoint(setpoint=setpoint, now=now)

        error = setpoint - self.current_position

        # set new setpoint if close to current setpoint
        if np.linalg.norm(error) < self.epsilon:
            self.get_logger().info(f"Reached setpoint: {setpoint}")
            self.index += 1
            if self.index >= len(self.setpoints):
                self.get_logger().info("All setpoints reached")
                self.timer.cancel()
                return
 
    def set_current_position(self, pose_msg: PoseStamped):
        current_robot_pos = pose_msg.pose.position
        self.current_position = np.array([current_robot_pos.x,
                                         current_robot_pos.y,
                                         current_robot_pos.z])

    def publish_setpoint(self, setpoint: list, now: rclpy.time.Time) -> None:
        msg = PointStamped()
        msg.point.x = setpoint[0]
        msg.point.y = setpoint[1]
        msg.point.z = setpoint[2]
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pos_setpoint_pub.publish(msg)


def main():
    rclpy.init()
    node = PosSetpointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()

