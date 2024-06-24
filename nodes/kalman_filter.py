#!/usr/bin/env python3
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from hippo_msgs.msg import RangeMeasurement, RangeMeasurementArray
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from tf_transformations import euler_from_quaternion


class PositionKalmanFilter(Node):

    def __init__(self):
        super().__init__(node_name='position_kalman_filter')

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST,
                         depth=1)

        self.init_params()

        self.time_last_prediction = self.get_clock().now()

        # TODO Assuming state consists of position x,y,z -> Feel free to add
        # more!
        self.num_states = 3

        self.num_measurements = 4

        # initial state
        self.x0 = np.zeros((self.num_states, 1))

        # state, this will be updated in Kalman filter algorithm
        self.state = np.copy(self.x0)

        # initial state covariance - how sure are we about the state?
        # TODO initial state covariance is tuning knob
        # dimension: num states x num states
        # matrix needs to be positive definite and symmetric
        self.P0 = 0.1 * np.eye(self.num_states)

        # state covariance, this will be updated in Kalman filter algorithm
        self.P = self.P0

        # measurement noise covariance - how much noise does the measurement
        # contain?
        # TODO tuning knob
        self.range_noise_stddev = 0.1
        # dimnesion: num measurements x num measurements
        # attention, this size is varying! -> Depends on detected Tags
        self.R = (self.range_noise_stddev**2) * np.eye(self.num_measurements)

        # process noise covariance - how much noise do we add at each
        # prediction step?
        # TODO tuning knob
        # dimension: num states x num states
        # matrix needs to be positive definite and symmetric
        self.process_noise_position_stddev = 0.1
        self.Q = (self.range_noise_stddev**2) * np.eye(self.num_states)

        # TODO enter tag poses here
        # TODO in the experiment, the tags will not be in these exact positions
        # however, the relative positions between tags will be the same
        self.tag_poses = np.array([[0.7, 3.8, -0.5], [1.3, 3.8, -0.5],
                                   [0.7, 3.8, -0.9] , [1.3, 3.8, -0.9]])

        self.position_pub = self.create_publisher(msg_type=PoseStamped,
                                                  topic='position_estimate',
                                                  qos_profile=1)

        self.ranges_sub = self.create_subscription(
            msg_type=RangeMeasurementArray,
            topic='ranges',
            callback=self.on_ranges,
            qos_profile=qos)
        self.vision_pose_sub = self.create_subscription(
            msg_type=PoseWithCovarianceStamped,
            topic='vision_pose_cov',
            callback=self.on_vision_pose,
            qos_profile=qos)
        # do prediction step with 50 Hz
        self.process_update_timer = self.create_timer(
            1.0 / 50, self.on_prediction_step_timer)

    def init_params(self):
        self.declare_parameters(namespace='',
                                parameters=[('range_noise_stddev',
                                             rclpy.Parameter.Type.DOUBLE),
                                            ('process_noise_position_stddev',
                                             rclpy.Parameter.Type.DOUBLE)])
        param = self.get_parameter('range_noise_stddev')
        self.get_logger().info(f'{param.name}={param.value}')
        self.range_noise_stddev = param.value

        param = self.get_parameter('process_noise_position_stddev')
        self.get_logger().info(f'{param.name}={param.value}')
        self.process_noise_position_stddev = param.value

    def on_params_changed(self, params):
        param: rclpy.Parameter
        for param in params:
            self.get_logger().info(f'Try to set [{param.name}] = {param.value}')
            if param.name == 'range_noise_stddev':
                self.range_noise_stddev = param.value
            elif param.name == 'process_noise_position_stddev':
                self.process_noise_position_stddev = param.value
                self.Q = (self.process_noise_position_stddev**2) * np.eye(
                    self.num_states)
            else:
                continue
        return SetParametersResult(successful=True, reason='Parameter set')

    def on_ranges(self, ranges_msg: RangeMeasurementArray) -> None:
        # how many tags are detected?
        num_measurements = len(ranges_msg._measurements)

        # if no tags are detected, stop here
        if not num_measurements:
            return

        # measurement_all = np.zeros(num_measurements)
        measured_distances = np.zeros(4)

        measurement: RangeMeasurement
        for index, measurement in enumerate(ranges_msg.measurements):
            # TODO
           tag_id = measurement.id
           measured_distances[tag_id] = measurement.range
           # measured_distance = measurement.range
           # self.get_logger().info(
           #     f'The {index}. element contains the measurement of tag {tag_id} with the '
           #     f'distance of {measured_distances[tag_id]}m')
            # measurement_all[index] = measurement
            # pass

        # before the measurement update, let's do a process update
        now = self.get_clock().now()
        dt = (now - self.time_last_prediction).nanoseconds * 1e-9
        self.prediction(dt)
        self.time_last_prediction = now

        # TODO
        self.measurement_update(measured_distances)

    def on_vision_pose(self, msg: PoseWithCovarianceStamped):
        # You might want to consider the vehicle's orientation

        # get the vehicle orientation expressed as quaternion
        q = msg.pose.pose.orientation
        # convert quaternion to euler angles
        (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # TODO

    def on_prediction_step_timer(self):
        # We will do a prediction step with a constant rate
        now = self.get_clock().now()
        dt = (now - self.time_last_prediction).nanoseconds * 1e-9

        self.prediction(dt)
        self.time_last_prediction = now

        # publish the estimated pose with constant rate
        self.publish_pose_msg(state=np.copy(self.state), now=now)

    def measurement_update(self, measured_distances):
        vehicle_position = np.copy(self.state[0:3, 0])
        sensor_position = vehicle_position 
        sensor_to_tag = sensor_position - self.tag_poses
        z_est = np.linalg.norm(sensor_to_tag, axis=1)


        # self.get_logger().info(f'measured_distances:{measured_distances}')
        # self.get_logger().info(f'z_est:{z_est}')
        valid_measurement_ids = np.nonzero(measured_distances)
        y = np.zeros(4)
        y[valid_measurement_ids] = measured_distances[valid_measurement_ids]- z_est[valid_measurement_ids]

        # self.get_logger().info(f'sensor_to_tag:{sensor_to_tag.T}')
        # h_jacobian = sensor_to_tag.T/z_est
        h_jacobian = np.zeros([3,4]) 
        h_jacobian[:,valid_measurement_ids] += (sensor_to_tag.T/z_est)[:,valid_measurement_ids]
        # self.get_logger().info(f'h_jacobian:{h_jacobian}')
        R_valid_measurements = np.zeros([4,4]) 
        R_valid_measurements[:,valid_measurement_ids] += self.R[:,valid_measurement_ids]
        S = h_jacobian.T @ self.P @ h_jacobian + R_valid_measurements

        K = self.P @ h_jacobian @ np.linalg.pinv(S)

        # self.get_logger().info(f'K:{K.shape}')
        # self.get_logger().info(f'y:{y.shape}')
        # self.get_logger().info(f'y:{(K@y)}')
        # self.get_logger().info(f'self.state:{self.state.shape}')
        # self.get_logger().info(f'self.state:{self.state[:,0].T}')
        # self.get_logger().info(f'self.state:{np.squeeze(self.state).shape}')

        # self.state = ((self.state)[:,0] + (K @ y )).T
        self.state = np.asarray(self.state + (K @ y ).reshape(3,1))
        self.get_logger().info(f'self.state:{self.state.T}')

        self.P = (np.eye(self.num_states) - K@h_jacobian.T) @ self.P

        # TODO

    def prediction(self, dt: float):
        # TODO
        self.state = self.state
        self.P = self.P + self.Q

    def publish_pose_msg(self, state: np.ndarray, now: rclpy.time.Time) -> None:
        msg = PoseStamped()

        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = state[0, 0]
        msg.pose.position.y = state[1, 0]
        msg.pose.position.z = state[2, 0]

        self.position_pub.publish(msg)


def main():
    rclpy.init()
    node = PositionKalmanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
