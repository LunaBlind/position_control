#!/usr/bin/env python3
"""
This node computes a square-wave setpoint for the depth controller, i.e.
the setpoint jumps between two different depth values with a set duration.
You can change this code to try out other setpoint functions, e.g. a sin wave.
"""
import rclpy
# from hippo_msgs.msg import Float64Stamped
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

from rcl_interfaces.msg import SetParametersResult
import rclpy.parameter
import rclpy.time

class PosSetpointNode(Node):

    def __init__(self):
        super().__init__(node_name='pos_setpoint_publisher')

        self.start_time = self.get_clock().now()

        # change these parameters to adjust the setpoints
        # ... or change implementation details below to achieve other setpoint
        # functions.
        self.setpoint_1 = [1.0, -2.0, -0.2]  # in m
        self.setpoint_2 = [2.0, -1.0, -1.2]  # in m
        self.duration = 10.0  # in seconds

        self.pos_setpoint_pub = self.create_publisher(msg_type=PointStamped,
                                                        topic='pos_setpoint',
                                                        qos_profile=1)
        self.timer = self.create_timer(timer_period_sec=1 / 50,
                                       callback=self.on_timer)

    def on_timer(self) -> None:
        # change this for other setpoint functions
        now = self.get_clock().now()
        time = self.start_time - now
        i = time.nanoseconds * 1e-9 % (self.duration * 2)
        if i > (self.duration):
            setpoint = self.setpoint_1
        else:
            setpoint = self.setpoint_2

        now = self.get_clock().now()
        self.publish_setpoint(setpoint=setpoint, now=now)

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

