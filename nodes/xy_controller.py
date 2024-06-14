#!/usr/bin/env python3
"""
This node is your depth controller.
It takes as input a current depth and a given depth setpoint.
Its output is a thrust command to the BlueROV's actuators.
"""
import rclpy
from hippo_msgs.msg import ActuatorSetpoint, DepthStamped, Float64Stamped
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult


class DepthControlNode(Node):

    def __init__(self):
        super().__init__(node_name='depth_controller')

        self.current_setpoint = 0.0
        self.current_depth = 0.0
        self.init_params()
        self.t_previous = self.get_clock().now()
        self.error_previous = 0.0
        self.error_dt_previous = 0.0
        self.error_integrated = 0.0


        self.thrust_pub = self.create_publisher(msg_type=ActuatorSetpoint,
                                                topic='thrust_setpoint',
                                                qos_profile=1)

        self.setpoint_sub = self.create_subscription(msg_type=Float64Stamped,
                                                     topic='depth_setpoint',
                                                     callback=self.on_setpoint,
                                                     qos_profile=1)
        self.depth_sub = self.create_subscription(msg_type=DepthStamped,
                                                  topic='depth',
                                                  callback=self.on_depth,
                                                  qos_profile=1)

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


    def on_setpoint(self, setpoint_msg: Float64Stamped):
        # We received a new setpoint! Let's save it, so that we can use it as
        # soon as we receive new depth data.
        self.current_setpoint = setpoint_msg.data
        self.get_logger().info(
            # f"Hi! I'm your controller running. "
            f'I received a setpoint of {self.current_setpoint} m.',
            throttle_duration_sec=1)

    def on_depth(self, depth_msg: DepthStamped):
        # We received a new depth message! Now we can get to action!
        current_depth = depth_msg.depth

        # self.get_logger().info(
        #     # f"Hi! I'm your controller running. "
        #     f'I received a depth of {current_depth} m.',
        #     throttle_duration_sec=1)

        thrust = self.compute_control_output(current_depth)
        # either set the timestamp to the current time or set it to the
        # stamp of `depth_msg` because the control output corresponds to this
        # point in time. Both choices are meaningful.
        # option 1:
        # now = self.get_clock().now()
        # option 2:
        timestamp = rclpy.time.Time.from_msg(depth_msg.header.stamp)
        self.publish_vertical_thrust(thrust=thrust, timestamp=timestamp)

    def publish_vertical_thrust(self, thrust: float,
                                timestamp: rclpy.time.Time) -> None:
        msg = ActuatorSetpoint()
        # we want to set the vertical thrust exlusively. mask out xy-components.
        msg.ignore_x = True
        msg.ignore_y = True
        msg.ignore_z = False

        msg.z = thrust

        # Let's add a time stamp
        msg.header.stamp = timestamp.to_msg()

        self.thrust_pub.publish(msg)

    def compute_control_output(self, current_depth: float) -> float:
        # TODO: Apply the PID control
        # thrust_z = 0.5  # This doesn't seem right yet...
        t_now = self.get_clock().now()
        dt_as_duration_object = t_now - self.t_previous
        dt = dt_as_duration_object = dt_as_duration_object.nanoseconds * 1e-9


        error_depth = -current_depth + self.current_setpoint

        error_dt = (error_depth - self.error_previous)/dt
        error_dt = 0.8 * self.error_dt_previous + 0.2 * error_dt
        # self.get_logger().info(
        #     # f"Hi! I'm your controller running. "
        #     f'I received a depth_error of {error_depth} m.',
        #     throttle_duration_sec=1)

        # thrust_z = error_depth * self.p_gain
        thrust_z = (error_depth * self.p_gain + error_dt * self.d_gain/dt + self.i_gain * self.error_integrated * dt)

        if (thrust_z > 1.0):
            thrust_z = 1.0

        elif (thrust_z < -1.0):
            thrust_z = -1.0

        else:
            self.error_integrated += error_depth * dt

        self.t_previous = t_now
        self.error_previous = error_depth
        self.error_dt_previous = error_dt

        return thrust_z


def main():
    rclpy.init()
    node = DepthControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()