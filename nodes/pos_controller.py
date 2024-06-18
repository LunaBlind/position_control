#!/usr/bin/env python3
"""
This node is your depth controller.
It takes as input a current depth and a given depth setpoint.
Its output is a thrust command to the BlueROV's actuators.
"""
import rclpy
import numpy as np
from hippo_msgs.msg import ActuatorSetpoint, DepthStamped, Float64Stamped
from geometry_msgs.msg import PoseStamped, PointStamped
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult


class PosControlNode(Node):

    def __init__(self):
        super().__init__(node_name='pos_controller')

        self.current_setpoint = np.array([2.5,-1.5, -0.5])
        self.current_robot_pos = np.zeros(3)
        self.init_params()

        self.t_previous = self.get_clock().now()
        self.error_previous = np.zeros(3)
        self.error_dt_previous = np.zeros(3)
        self.error_integrated = np.zeros(3)

        self.output_limits = np.array([-1,1])


        self.thrust_pub = self.create_publisher(msg_type=ActuatorSetpoint,
                                                topic='thrust_setpoint',
                                                qos_profile=1)

        self.setpoint_sub = self.create_subscription(msg_type=PointStamped,
                                                     topic='pos_setpoint',
                                                     callback=self.on_setpoint,
                                                     qos_profile=1)

        # self.yaw_angle_sub = self.create_subscription(msg_type=Float64Stamped,
        #                                              topic='xy_pos_setpoint',
        #                                              callback=self.on_setpoint,
        #                                              qos_profile=1)
        self.pos_sub = self.create_subscription(msg_type=PoseStamped,
                                                  topic='position_estimate',
                                                  callback=self.on_depth,
                                                  qos_profile=1)

        # self.depth_sub = self.create_subscription(msg_type=DepthStamped,
        #                                           topic='depth',
        #                                           callback=self.on_depth,
        #                                           qos_profile=1)

    def init_params(self):
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gains.p', rclpy.Parameter.Type.DOUBLE),
                ('gains.i', rclpy.Parameter.Type.DOUBLE),
                ('gains.d', rclpy.Parameter.Type.DOUBLE),
            ],
        )
        param = self.get_parameter('gains.p')
        self.get_logger().info(f'{param.name}={param.value}')
        self.p_gain = param.value

        param = self.get_parameter('gains.i')
        self.get_logger().info(f'{param.name}={param.value}')
        self.i_gain = param.value

        param = self.get_parameter('gains.d')
        self.get_logger().info(f'{param.name}={param.value}')
        self.d_gain = param.value

        self.add_on_set_parameters_callback(self.on_params_changed)

    def on_params_changed(self, params ):
        param: rclpy.Parameter
        for param in params:
            self.get_logger().info(f'Try to set [{param.name}] = {param.value}')
            if param.name == 'gains.p':
                self.p_gain = param.value
            elif param.name == 'gains.i':
                self.i_gain = param.value
            elif param.name == 'gains.d':
                self.d_gain = param.value
            else:
                continue
        return SetParametersResult(successful=True, reason='Parameter set')


    def on_setpoint(self, setpoint_msg: PointStamped):
        # We received a new setpoint! Let's save it, so that we can use it as
        # soon as we receive new depth data.
        self.current_setpoint[0] = setpoint_msg.point.x
        self.current_setpoint[1] = setpoint_msg.point.y
        self.current_setpoint[2] = setpoint_msg.point.z
        # self.get_logger().info(
        #     # f"Hi! I'm your controller running. "
        #     f'I received a setpoint of {self.current_setpoint} m.',
        #     throttle_duration_sec=1)

    def on_depth(self, pos_msg: PoseStamped):
        # We received a new depth message! Now we can get to action!
        current_position = pos_msg.pose.position
        yaw = np.pi / 2
        current_2D_pos_world = np.array([current_position.x,
                                         current_position.y,
                                         current_position.z])
        rotation_world_robot = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                                        [np.sin(yaw), np.cos(yaw), 0],
                                         [0,0,1]])

        current_robot_pos = current_2D_pos_world @ rotation_world_robot
        # current_2D_pos_world = np.array([current_position.y,-current_position.x])

        self.get_logger().info(
            # f"Hi! I'm your controller running. "
            f'I received a position of {current_robot_pos} m.',
            throttle_duration_sec=1)

        thrust = self.compute_control_output(current_robot_pos)
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

    def compute_control_output(self, current_robot_pos: np.ndarray) -> np.ndarray:
        # TODO: Apply the PID control
        # thrust_z = 0.5  # This doesn't seem right yet...
        t_now = self.get_clock().now()
        dt_as_duration_object = t_now - self.t_previous
        dt = dt_as_duration_object = dt_as_duration_object.nanoseconds * 1e-9


        error = -current_robot_pos + self.current_setpoint

        error_dt = (error - self.error_previous)/dt
        error_dt = 0.8 * self.error_dt_previous + 0.2 * error_dt
        thrust = (error * self.p_gain + error_dt * self.d_gain/dt + self.i_gain * self.error_integrated * dt)

        thrust_direction_within_limits = self.within_limits(thrust)
        self.error_integrated[thrust_direction_within_limits] += error[thrust_direction_within_limits] * np.squeeze(dt)


        lower_limit, upper_limit = self.output_limits
        np.clip(thrust, lower_limit, upper_limit)

        self.t_previous = t_now
        self.error_previous = error
        self.error_dt_previous = error_dt

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
