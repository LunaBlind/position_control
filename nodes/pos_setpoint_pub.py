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
from enum import Enum, auto


class State(Enum):
    UNSET = auto()
    INIT = auto()
    IDLE = auto()
    MOVE_TO_START = auto()
    EN_ROUTE = auto()


class PosSetpointNode(Node):

    def __init__(self):
        super().__init__(node_name='pos_setpoint_publisher')

        self.start_time = self.get_clock().now()
        self.state = State.MOVE_TO_START

        # change these parameters to adjust the setpoints
        # ... or change implementation details below to achieve other setpoint
        # functions.
        self.setpoints = [[0.0, 0.0, -0.5],
                          [1.5, 0.0, -0.5],
                          [1.5, 1.5, -0.5],
                          [0.0, 1.5, -0.5],
                          [0.0, 0.0, -0.5],
                          ]
        self.waypoints = []
        self.current_waypoint = np.zeros(3)
        self.num_of_waypoints = 5


        self.current_position = np.zeros(3)
        self.current_setpoint = self.setpoints[0]
        self.previous_setpoint = np.zeros(3)
        self.epsilon = 0.3
        self.index = 0
        self.waypoint_index = 0
        self.sleep_timer = 0

        # self.pos_sub = self.create_subscription(msg_type=PoseStamped,
        #                                           topic='position_estimate',
        #                                           callback=self.set_current_position,
        #                                           qos_profile=1)

        self.pos_sub = self.create_subscription(msg_type=PoseStamped,
                                                  topic='ground_truth/pose',
                                                  callback=self.set_current_position,
                                                  qos_profile=1)

        self.pos_setpoint_pub = self.create_publisher(msg_type=PointStamped,
                                                        topic='pos_setpoint',
                                                        qos_profile=1)
        self.timer = self.create_timer(timer_period_sec=1 ,
                                       callback=self.timer_callback)

    def timer_callback(self):
        if self.state == State.MOVE_TO_START:
            self.index = 0
            self.current_setpoint = self.setpoints[self.index]
            self.previous_setpoint = self.setpoints[self.index]
            now = self.get_clock().now()
            self.publish_setpoint(setpoint=self.current_setpoint, now=now)

            error = self.current_setpoint - self.current_position

            # set new setpoint if close to current setpoint
            if np.linalg.norm(error) < self.epsilon:
                self.get_logger().info(f"Reached startpoint: {self.current_setpoint}")
                self.index += 1
                self.state = State.INIT
                return

        if self.state == State.INIT:
            if self.sleep_timer >=2:
                self.current_setpoint = self.setpoints[self.index]

                # self.create_square_path()

                self.create_circular_path()

                now = self.get_clock().now()
                self.current_waypoint = self.waypoints[self.waypoint_index]
                self.publish_setpoint(setpoint=self.current_waypoint, now=now)

                self.state = State.EN_ROUTE
            self.sleep_timer += 1

        if self.state == State.EN_ROUTE:
            self.current_waypoint = self.waypoints[self.waypoint_index]
            error = self.current_waypoint - self.current_position

            # set new setpoint if close to current setpoint
            if np.linalg.norm(error) < self.epsilon * 2:
                # self.get_logger().info(f"Reached waypoint: {self.current_waypoint}")
                self.waypoint_index += 1
                now = self.get_clock().now()
                self.publish_setpoint(setpoint=self.current_waypoint, now=now)
                # self.get_logger().info(f"En Route move setpoint: {self.current_waypoint}")
                if self.waypoint_index >= len(self.waypoints):
                    # self.get_logger().info(f"Setpoint reached: {self.current_setpoint}")
                    self.waypoint_index = 0
                    self.index += 1
                    self.previous_setpoint = self.current_setpoint
                    self.sleep_timer = 0
                    self.state = State.INIT
                    if self.index >= len(self.setpoints):
                        self.get_logger().info("All setpoints reached")
                        # self.timer.cancel()
                        self.state = State.MOVE_TO_START
                        return

    def create_square_path(self):
        """Creates equidistant points on a line between the current point 
        and the goal"""
        self.waypoints = np.linspace(self.previous_setpoint,
                                     self.current_setpoint,
                                     num=self.num_of_waypoints + 1, endpoint=True)

    def create_circular_path(self):
        """Creates equidistant points on a half circle between the current point 
        and the goal"""
        distance = np.array(self.current_setpoint) - self.previous_setpoint
        radius = np.linalg.norm(distance[:-1],ord=2)/2 
        center_point = self.previous_setpoint + distance/2

        angle = np.arctan2(distance[1], distance[0])
        # TODO look into why it needs to be the wrong way around
        thetas = np.linspace(np.pi, 0, num=self.num_of_waypoints + 1, endpoint=True) 
        waypoints = []

        for theta in thetas:
            waypoint = center_point + (np.array([radius * np.cos(angle-theta), radius * np.sin(angle-theta ), 0]))
            waypoints.append(waypoint)
        self.waypoints = waypoints
 
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

