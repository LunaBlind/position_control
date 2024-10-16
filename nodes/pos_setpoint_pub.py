#!/usr/bin/env python3
"""
This node computes a square-wave setpoint for the depth controller, i.e.
the setpoint jumps between two different depth values with a set duration.
You can change this code to try out other setpoint functions, e.g. a sin wave.
"""
import rclpy
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Bool
from rcl_interfaces.msg import SetParametersResult


from hippo_msgs.msg import DepthStamped
from rclpy.node import Node

import rclpy.parameter
import rclpy.time

import numpy as np
from enum import Enum, auto
import time


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

        # self.distance_gripper_robot = np.array([-0.0375, 0.52, 0.05])
        self.distance_gripper_robot = np.array([-0.08, 0.58, 0.06])
        # self.distance_gripper_robot = np.array([0.42, 0.06, -0.06])
        # self.distance_gripper_robot = np.array([0.0, 0.24, 0.0])

        self.starting_point = np.array([0.79, 3.0, -0.85]) 
        self.object_point = np.array([0,0,0])
        self.object_apriltag_offset = np.array([0.025, 0, 0.04])
        self.holding_object = False


        self.waypoints = []
        self.current_waypoint = np.zeros(3)
        self.num_of_waypoints = 20


        self.current_position = np.array([1.3, 1.0, -0.5]) # start position of the bluerov in sim
        self.current_setpoint = self.starting_point
        self.previous_setpoint = np.zeros(3)
        self.init_params()
        self.index = 0
        self.waypoint_index = 0

        # self.pos_sub = self.create_subscription(msg_type=PoseStamped,
        #                                           topic='position_estimate',
        #                                           callback=self.set_current_position,
        #                                           qos_profile=1)


        self.pos_sub = self.create_subscription(msg_type=PoseStamped,
                                                  topic='ground_truth/pose',
                                                  callback=self.set_current_position,
                                                  qos_profile=1)

        self.obj_pos_sub = self.create_subscription(msg_type=PoseStamped,
                                                  topic='object_pose',
                                                  callback=self.set_object_position,
                                                  qos_profile=1)

        self.pos_setpoint_pub = self.create_publisher(msg_type=PointStamped,
                                                        topic='pos_setpoint',
                                                        qos_profile=1)

        self.pos_error_pub = self.create_publisher(msg_type=PointStamped,
                                                        topic='pos_error',
                                                        qos_profile=1)

        self.euclidian_pos_error_pub = self.create_publisher(msg_type=DepthStamped,
                                                  topic='euclidian_pos_error',
                                                  qos_profile=1)

        self.grip_command_pub = self.create_publisher(msg_type=Bool,
                                                  topic='grasp_command',
                                                  qos_profile=1)

        self.object_grabbed_sub = self.create_subscription(msg_type=Bool,
                                                  topic='object_grabbed',
                                                  callback=self.object_grabbed,
                                                  qos_profile=1)

        self.timer = self.create_timer(timer_period_sec=1/10 ,
                                       callback=self.timer_callback)


    def init_params(self):
        self.declare_parameters(
            namespace='',
            parameters=[
                ('epsilon.en_route', rclpy.parameter.Parameter.Type.DOUBLE),
                ('epsilon.goal', rclpy.parameter.Parameter.Type.DOUBLE),
                ('point.goal', rclpy.parameter.Parameter.Type.DOUBLE_ARRAY),
            ],
        )
        param = self.get_parameter('epsilon.en_route')
        self.get_logger().info(f'{param.name}={param.value}')
        self.epsilon_en_route = param.value

        param = self.get_parameter('epsilon.goal')
        self.get_logger().info(f'{param.name}={param.value}')
        self.epsilon_goal = param.value

        param = self.get_parameter('point.goal')
        self.get_logger().info(f'{param.name}={param.value}')
        self.goal_point = param.value - self.distance_gripper_robot

        self.add_on_set_parameters_callback(self.on_params_changed)

    def on_params_changed(self, params ):
        param: rclpy.parameter.Parameter
        for param in params:
            self.get_logger().info(f'Try to set [{param.name}] = {param.value}')
            if param.name == 'epsilon.en_route':
                self.epsilon_en_route = param.value
            if param.name == 'epsilon.goal':
                self.epsilon_goal = param.value
            if param.name == 'point.goal':
                self.goal_point = param.value - self.distance_gripper_robot
            else:
                continue
        return SetParametersResult(successful=True, reason='Parameter set')

    def timer_callback(self):
        if self.state == State.MOVE_TO_START:
            self.current_setpoint = self.starting_point
            self.previous_setpoint = self.current_position
            self.create_straight_path()

            self.current_waypoint = self.waypoints[self.waypoint_index]
            self.publish_setpoint(setpoint=self.current_waypoint)
            self.previous_state = State.MOVE_TO_START
            self.state = State.EN_ROUTE

        if self.state == State.INIT:
            time.sleep(5)
            self.current_setpoint = self.object_point
            self.get_logger().info(f"Object point: {self.object_point}")

            self.create_straight_path()
            grasp_msg = Bool()
            grasp_msg.data = False
            self.grip_command_pub.publish(grasp_msg)

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
                    self.waypoint_index = 0
                    self.previous_setpoint = self.current_setpoint
                    if self.previous_state == State.INIT:
                        self.state = State.GRASP
                    elif self.previous_state == State.TRANSPORT:
                        self.state = State.DROP
                    elif self.previous_state == State.MOVE_TO_START:
                        self.state = State.INIT
            else:
                # set new setpoint if close to current setpoint
                if mean_squared_error < self.epsilon_en_route:
                    self.waypoint_index += 1

        if self.state == State.GRASP:
            grasp_msg = Bool()
            grasp_msg.data = True
            self.grip_command_pub.publish(grasp_msg)
            if self.holding_object == True:
                self.state = State.LIFT

        if self.state == State.LIFT:
            self.current_setpoint += np.array([0,0,0.075])
            self.publish_setpoint(setpoint=self.current_setpoint)
            self.get_logger().info(f'{self.state}')
            time.sleep(10)
            self.state = State.TRANSPORT

        if self.state == State.TRANSPORT:
            self.previous_setpoint = self.current_setpoint
            self.current_setpoint = self.goal_point + np.array([0,0,0.05])
            # self.current_setpoint = self.goal_point 
            self.create_straight_path()
            self.get_logger().info(f'{self.state}')
            self.state = State.EN_ROUTE
            self.previous_state = State.TRANSPORT

        if self.state == State.LOWER:
            self.current_setpoint += np.array([0,0,-0.05])
            self.publish_setpoint(setpoint=list(self.current_setpoint))
            self.get_logger().info(f'{self.state}')
            time.sleep(10)
            self.state = State.DROP

        if self.state == State.DROP:
            grasp_msg = Bool()
            grasp_msg.data = False
            self.grip_command_pub.publish(grasp_msg)
            # receive drop completed
            self.get_logger().info(f'{self.state}')

            self.state = State.MOVE_TO_START

    def create_straight_path(self):
        """Creates equidistant points on a line between the current point 
        and the goal"""
        self.waypoints = []
        distance_to_current_waypoint = np.zeros(3)
        distance = np.array(self.current_setpoint) - self.previous_setpoint
        while (np.max(np.abs(distance)) > self.epsilon_en_route):
            waypoint = self.previous_setpoint + distance_to_current_waypoint + distance/2
            distance_to_current_waypoint += distance/2
            distance = self.current_setpoint - waypoint 
            self.waypoints.append(waypoint)
        self.waypoints.append(self.current_setpoint)
 
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

    def set_object_position(self, pose_msg: PoseStamped):
        object_position_in_map = pose_msg.pose.position
        object_position_in_map = np.array([object_position_in_map.x,
                                           object_position_in_map.y,
                                           object_position_in_map.z])
        self.object_point = object_position_in_map - self.distance_gripper_robot - self.object_apriltag_offset 

    def object_grabbed(self, msg):
        self.holding_object = msg.data

def main():
    rclpy.init()
    node = PosSetpointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()

