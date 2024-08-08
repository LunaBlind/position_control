#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray, Bool

class GripperControl(Node):
    def __init__(self):
        super().__init__('gripper_control')
        self.gripper_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        self.grip_command_sub = self.create_subscription(msg_type=Bool,
                                                  topic='grasp_command',
                                                  callback=self.grip_control,
                                                  qos_profile=1)
        # self.right_finger_pub = self.create_publisher(JointTrajectory, '/right_finger_controller/joint_trajectory', 10)
        # self.timer = self.create_timer(1.0, self.timer_callback)
        self.grip_state = False

    def grip_control(self, grasp_msg):
        self.grip_state = grasp_msg.data
        pub_msg = Float64MultiArray()
        if self.grip_state:
            pub_msg.data = [0.00]
        else:
            pub_msg.data = [0.78]
        self.gripper_pub.publish(pub_msg)

            

    # def timer_callback(self):

        # msg_left = JointTrajectory()
        # msg_right = JointTrajectory()

        # msg_left.joint_names = ['left_finger_joint']
        # msg_right.joint_names = ['right_finger_joint']

        # point = JointTrajectoryPoint()
        # if self.grip_state:
        #     point.positions = [0.0]  # Open position
        # else:
        #     point.positions = [0.15]  # Closed position
        # point.time_from_start.sec = 1
        # point.time_from_start.nanosec = 0

        # msg_left.points = [point]
        # msg_right.points = [point]

        # self.left_finger_pub.publish(msg_left)
        # self.right_finger_pub.publish(msg_right)

        # self.grip_state = not self.grip_state

def main(args=None):
    rclpy.init(args=args)
    node = GripperControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
