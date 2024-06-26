#!/usr/bin/env python3
"""
This node is your depth controller.
It takes as input a current depth and a given depth setpoint.
Its output is a thrust command to the BlueROV's actuators.
"""
import rclpy
import numpy as np
from hippo_msgs.msg import ActuatorSetpoint
from geometry_msgs.msg import PoseStamped, PointStamped
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from tf_transformations import euler_from_quaternion

import rclpy.time
import rclpy.parameter


class PosControlNode(Node):

    def __init__(self):
        super().__init__(node_name='pos_controller')

        self.current_setpoint = np.array([0.5, 1.0, -0.5])
        self.current_robot_pos = np.zeros(3)
        self.init_params()

        self.t_previous = self.get_clock().now()

        self.error_previous = np.zeros(3)
        self.error_dt_previous = np.zeros(3)
        self.error_integrated = np.zeros(3)

        # Same limits as in the final project controller 
        self.output_limits = np.array([-0.5,0.5])


        self.thrust_pub = self.create_publisher(msg_type=ActuatorSetpoint,
                                                topic='thrust_setpoint',
                                                qos_profile=1)

        self.setpoint_sub = self.create_subscription(msg_type=PointStamped,
                                                     topic='pos_setpoint',
                                                     callback=self.on_setpoint,
                                                     qos_profile=1)

        # self.pos_sub = self.create_subscription(msg_type=PoseStamped,
        #                                           topic='ground_truth/pose',
        #                                           callback=self.on_depth,
        #                                           qos_profile=1)
        #
        self.pos_sub = self.create_subscription(msg_type=PoseStamped,
                                                  topic='position_estimate',
                                                  callback=self.on_depth,
                                                  qos_profile=1)

        # might be interesting to compare that to the EKF value 
        # self.depth_sub = self.create_subscription(msg_type=DepthStamped,
        #                                           topic='depth',
        #                                           callback=self.on_depth,
        #                                           qos_profile=1)

    def init_params(self):
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gains_xy.p', rclpy.parameter.Parameter.Type.DOUBLE),
                ('gains_xy.i', rclpy.parameter.Parameter.Type.DOUBLE),
                ('gains_xy.d', rclpy.parameter.Parameter.Type.DOUBLE),
                ('gains_z.p', rclpy.parameter.Parameter.Type.DOUBLE),
                ('gains_z.i', rclpy.parameter.Parameter.Type.DOUBLE),
                ('gains_z.d', rclpy.parameter.Parameter.Type.DOUBLE),
                ('epsilon.goal', rclpy.parameter.Parameter.Type.DOUBLE),
            ],
        )
        param_xy = self.get_parameter('gains_xy.p')
        self.get_logger().info(f'{param_xy.name}={param_xy.value}')
        param_z = self.get_parameter('gains_z.p')
        self.get_logger().info(f'{param_z.name}={param_z.value}')
        self.p_gain = np.array([param_xy.value, param_xy.value, param_z.value])

        param_xy = self.get_parameter('gains_xy.i')
        self.get_logger().info(f'{param_xy.name}={param_xy.value}')
        param_z = self.get_parameter('gains_z.i')
        self.get_logger().info(f'{param_z.name}={param_z.value}')
        self.i_gain = np.array([param_xy.value, param_xy.value, param_z.value])

        param_xy = self.get_parameter('gains_xy.d')
        self.get_logger().info(f'{param_xy.name}={param_xy.value}')
        param_z = self.get_parameter('gains_z.d')
        self.get_logger().info(f'{param_z.name}={param_z.value}')
        self.d_gain = param_xy.value

        param = self.get_parameter('epsilon.goal')
        self.get_logger().info(f'{param.name}={param.value}')
        self.epsilon_goal = param.value

        self.add_on_set_parameters_callback(self.on_params_changed)

    def on_params_changed(self, params ):
        param: rclpy.parameter.Parameter
        for param in params:
            self.get_logger().info(f'Try to set [{param.name}] = {param.value}')
            if param.name == 'gains_xy.p':
                self.p_gain = param.value
            elif param.name == 'gains_xy.i':
                self.i_gain = param.value
            elif param.name == 'gains_xy.d':
                self.d_gain = param.value
            elif param.name == 'gains_z.p':
                self.p_gain = param.value
            elif param.name == 'gains_z.i':
                self.i_gain = param.value
            elif param.name == 'gains_z.d':
                self.d_gain = param.value
            elif param.name == 'epsilon.goal':
                self.epsilon_goal = param.value
            else:
                continue
        return SetParametersResult(successful=True, reason='Parameter set')


    def on_setpoint(self, setpoint_msg: PointStamped):
        # We received a new setpoint! Let's save it, so that we can use it as
        # soon as we receive new position data.
        self.current_setpoint[0] = setpoint_msg.point.x
        self.current_setpoint[1] = setpoint_msg.point.y
        self.current_setpoint[2] = setpoint_msg.point.z

    def on_depth(self, pos_msg: PoseStamped):
        # We received a new position message! Now we can get to action!
        current_position = pos_msg.pose.position
        yaw = np.pi/2
        # q = pos_msg.pose.orientation
        # (_,_, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        current_robot_pos = np.array([current_position.x,
                                         current_position.y,
                                         current_position.z])

        thrust = self.compute_control_output(current_robot_pos, yaw)
        # either set the timestamp to the current time or set it to the
        # stamp of `depth_msg` because the control output corresponds to this
        # point in time. Both choices are meaningful.
        # option 1:
        # now = self.get_clock().now()
        # option 2:
        timestamp = rclpy.time.Time.from_msg(pos_msg.header.stamp)
        self.publish_vertical_thrust(thrust=thrust, timestamp=timestamp)

    def publish_vertical_thrust(self, thrust: np.ndarray,
                                timestamp: rclpy.time.Time) -> None:
        msg = ActuatorSetpoint()
        # we want to set the vertical thrust exlusively. mask out xy-components.
        msg.ignore_x = False
        msg.ignore_y = False
        msg.ignore_z = False

        msg.x, msg.y, msg.z = thrust

        # Let's add a time stamp
        msg.header.stamp = timestamp.to_msg()

        self.thrust_pub.publish(msg)

    def compute_control_output(self, current_robot_pos: np.ndarray, yaw: float) -> np.ndarray:
        # TODO: Apply the PID control
        # thrust_z = 0.5  # This doesn't seem right yet...
        t_now = self.get_clock().now()
        dt_as_duration_object = t_now - self.t_previous
        dt = dt_as_duration_object = dt_as_duration_object.nanoseconds * 1e-9


        error = -current_robot_pos + self.current_setpoint
        rotation_world_robot = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                                        [np.sin(yaw), np.cos(yaw), 0],
                                         [0,0,1]])
        error = error @ rotation_world_robot

        if (np.abs(error).mean() < self.epsilon_goal):
            self.error_integrated = np.zeros(3)

        error_dt = (error - self.error_previous)/dt
        error_dt = 0.8 * self.error_dt_previous + 0.2 * error_dt
        thrust = (error * self.p_gain + error_dt * self.d_gain/dt + self.i_gain * self.error_integrated * dt)

        # only integrate error if within limits (anti-reset windup)
        thrust_direction_within_limits = self.within_limits(thrust)
        self.error_integrated[thrust_direction_within_limits] += error[thrust_direction_within_limits] * np.squeeze(dt)

        # limit output to physical limits of the robot (or as in the final project to half of that)
        lower_limit, upper_limit = self.output_limits
        np.clip(thrust, lower_limit, upper_limit)

        self.t_previous = t_now
        self.error_previous = error
        self.error_dt_previous = error_dt

        thrust[2] += -0.045

        return thrust

    def within_limits(self, output):
        """Check which indices of the output are within the specified limits."""
        lower_limit, upper_limit = self.output_limits
        within_lower = output >= lower_limit
        within_upper = output <= upper_limit
        within_limits_indices = np.where(within_lower & within_upper)[0]
        return within_limits_indices


def main():
    rclpy.init()
    node = PosControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
