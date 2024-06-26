#!/usr/bin/env python3
"""
This node computes a square-wave setpoint for the depth controller, i.e.
the setpoint jumps between two different depth values with a set duration.
You can change this code to try out other setpoint functions, e.g. a sin wave.
"""
import rclpy
from geometry_msgs.msg import PoseStamped, PointStamped
from rcl_interfaces.msg import SetParametersResult


from hippo_msgs.msg import DepthStamped
from rclpy.node import Node

import rclpy.parameter
import rclpy.time

import numpy as np
from enum import Enum, auto


class State(Enum):
    INIT = auto()
    IDLE = auto()
    MOVE_TO_START = auto()
    EN_ROUTE = auto()
    TRANSPORT = auto()
    LIFT = auto()
    LOWER = auto()
    GRASP = auto()
    DROP = auto()
    DRIVE_LOOPS = auto()


class PosSetpointNode(Node):

    def __init__(self):
        super().__init__(node_name='pos_setpoint_publisher')

        self.start_time = self.get_clock().now()
        self.state = State.MOVE_TO_START
        self.previous_state = State.MOVE_TO_START

        # change these parameters to adjust the setpoints
        # ... or change implementation details below to achieve other setpoint
        # functions.

        # # square setpoints 
        # self.setpoints = [[0.5, 1.0, -0.5],
        #                   [1.5, 1.0, -1.5],
        #                   [1.5, 2.5, -1.5],
        #                   [0.5, 2.5, -0.5],
        #                   [0.5, 1.0, -1.5],
        #                   ]

        # # diagonal setpoints 
        # self.setpoints = [[1.0, 0.0, -0.5],
        #                   [2.0, 1.0, -0.5],
        #                   [1.0, 2.0, -0.5],
        #                   [0.0, 1.0, -0.5],
        #                   [1.0, 0.0, -0.5],
        #                   ]

        self.starting_point = [0.5, 1.0, -0.5]
        self.object_point = [0.7, 3.3, -0.9]
        self.goal_point = [1.3, 3.3, -0.9]

        self.waypoints = []
        self.current_waypoint = np.zeros(3)
        self.num_of_waypoints = 20


        self.current_position = np.zeros(3)
        self.current_setpoint = self.starting_point
        self.previous_setpoint = np.zeros(3)
        self.init_params()
        self.index = 0
        self.waypoint_index = 0

        self.pos_sub = self.create_subscription(msg_type=PoseStamped,
                                                  topic='position_estimate',
                                                  callback=self.set_current_position,
                                                  qos_profile=1)


        # self.pos_sub = self.create_subscription(msg_type=PoseStamped,
        #                                           topic='ground_truth/pose',
        #                                           callback=self.set_current_position,
        #                                           qos_profile=1)

        self.pos_setpoint_pub = self.create_publisher(msg_type=PointStamped,
                                                        topic='pos_setpoint',
                                                        qos_profile=1)

        self.pos_error_pub = self.create_publisher(msg_type=PointStamped,
                                                        topic='pos_error',
                                                        qos_profile=1)

        self.euclidian_pos_error_pub = self.create_publisher(msg_type=DepthStamped,
                                                  topic='euclidian_pos_error',
                                                  qos_profile=1)

        self.timer = self.create_timer(timer_period_sec=1/10 ,
                                       callback=self.timer_callback)


    def init_params(self):
        self.declare_parameters(
            namespace='',
            parameters=[
                ('epsilon.en_route', rclpy.parameter.Parameter.Type.DOUBLE),
                ('epsilon.goal', rclpy.parameter.Parameter.Type.DOUBLE),
            ],
        )
        param = self.get_parameter('epsilon.en_route')
        self.get_logger().info(f'{param.name}={param.value}')
        self.epsilon_en_route = param.value

        param = self.get_parameter('epsilon.goal')
        self.get_logger().info(f'{param.name}={param.value}')
        self.epsilon_goal = param.value

        self.add_on_set_parameters_callback(self.on_params_changed)

    def on_params_changed(self, params ):
        param: rclpy.parameter.Parameter
        for param in params:
            self.get_logger().info(f'Try to set [{param.name}] = {param.value}')
            if param.name == 'epsilon.en_route':
                self.epsilon_en_route = param.value
            if param.name == 'epsilon.goal':
                self.epsilon_goal = param.value
            else:
                continue
        return SetParametersResult(successful=True, reason='Parameter set')

    def timer_callback(self):
        if self.state == State.MOVE_TO_START:
            # self.index = 0
            self.current_setpoint = self.starting_point
            self.previous_setpoint = self.starting_point
            self.publish_setpoint(setpoint=self.current_setpoint)

            error = self.current_setpoint - self.current_position
            self.publish_error(error)
            mean_squared_error = np.linalg.norm(error, ord=2)

            # set new setpoint if close to current setpoint
            if mean_squared_error < self.epsilon_goal:
                self.get_logger().info(f"Reached startpoint: {self.current_setpoint}")
                # self.index += 1
                self.state = State.INIT

        if self.state == State.INIT:
            self.current_setpoint = self.object_point

            self.create_straight_path()
            # self.create_circular_path()

            self.current_waypoint = self.waypoints[self.waypoint_index]
            self.publish_setpoint(setpoint=self.current_waypoint)
            self.previous_state = State.INIT
            self.state = State.EN_ROUTE


        if self.state == State.EN_ROUTE:
            self.current_waypoint = self.waypoints[self.waypoint_index]
            self.publish_setpoint(setpoint=self.current_waypoint)
            error = self.current_waypoint - self.current_position
            self.publish_error(error)
            mean_squared_error = np.linalg.norm(error, ord=2)

            if self.waypoint_index == len(self.waypoints) - 1:
                if mean_squared_error < self.epsilon_goal:
                    self.get_logger().info(f"Setpoint reached: {self.current_setpoint}")
                    self.waypoint_index = 0
                    # self.index += 1
                    self.previous_setpoint = self.current_setpoint
                    if self.previous_state == State.INIT:
                        self.state = State.GRASP
                    elif self.previous_state == State.TRANSPORT:
                        self.state = State.LOWER
                    # if self.index >= len(self.setpoints):
                    #     self.get_logger().info("All setpoints reached")
                    #     self.state = State.MOVE_TO_START
            else:
                # set new setpoint if close to current setpoint
                if mean_squared_error < self.epsilon_en_route:
                    self.waypoint_index += 1
                    self.get_logger().info(f"Waypoint reached: {self.current_waypoint}")

        if self.state == State.GRASP:
            # Dummy state
            # publish grasp command
            # receive grasp completed
            self.state = State.LIFT

        if self.state == State.LIFT:
            self.current_setpoint += np.array([0,0,0.1])
            self.publish_setpoint(setpoint=self.current_setpoint)
            self.state = State.TRANSPORT

        if self.state == State.TRANSPORT:
            self.previous_setpoint = self.current_setpoint
            self.current_setpoint = self.goal_point + np.array([0,0,0.1])
            self.create_straight_path()
            self.state = State.EN_ROUTE
            self.previous_state = State.TRANSPORT

        if self.state == State.LOWER:
            self.current_setpoint += np.array([0,0,-0.1])
            self.publish_setpoint(setpoint=list(self.current_setpoint))
            self.state = State.DROP

        if self.state == State.DROP:
            # Dummy state
            # publish drop command
            # receive drop completed
            self.state = State.MOVE_TO_START

        # if self.state == State.DRIVE_LOOPS:
        #     self.current_setpoint = self.setpoints[self.index]
        #
        #     self.create_straight_path()
        #     # self.create_circular_path()
        #
        #     self.current_waypoint = self.waypoints[self.waypoint_index]
        #     self.publish_setpoint(setpoint=self.current_waypoint)
        #     self.previous_state = State.INIT
        #     self.state = State.EN_ROUTE

    def create_straight_path(self):
        """Creates equidistant points on a line between the current point 
        and the goal"""
        self.waypoints = np.linspace(self.previous_setpoint,
                                     self.current_setpoint,
                                     num=self.num_of_waypoints, endpoint=True)

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

    def publish_setpoint(self, setpoint: list) -> None:
        msg = PointStamped()
        msg.point.x = setpoint[0]
        msg.point.y = setpoint[1]
        msg.point.z = setpoint[2]
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pos_setpoint_pub.publish(msg)

    def publish_error(self, error: np.ndarray) -> None:
        msg = PointStamped()
        msg.point.x = error[0]
        msg.point.y = error[1]
        msg.point.z = error[2]
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pos_error_pub.publish(msg)

        euclidian_err_msg = DepthStamped()
        euclidian_err_msg.depth = np.linalg.norm(error, ord=2)
        euclidian_err_msg.header.stamp = msg.header.stamp
        self.euclidian_pos_error_pub.publish(euclidian_err_msg)


def main():
    rclpy.init()
    node = PosSetpointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()

