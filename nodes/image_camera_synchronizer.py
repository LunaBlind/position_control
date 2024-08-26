#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import Header

class ImageCameraSynchronizer(Node):
    def __init__(self):
        super().__init__('image_camera_synchronizer')
        
        self.image_sub = Subscriber(self, Image, 'front_camera/image_rect')
        self.camera_info_sub = Subscriber(self, CameraInfo, 'front_camera/camera_info')

        ats = ApproximateTimeSynchronizer([self.image_sub, self.camera_info_sub], queue_size=10, slop=0.1)
        ats.registerCallback(self.sync_callback)
        
        self.synced_image_pub = self.create_publisher(Image, 'synchronized_image', 10)
        self.synced_camera_info_pub = self.create_publisher(CameraInfo, 'synchronized_camera_info', 10)

    def sync_callback(self, image, camera_info):
        # Publish the synchronized messages
        self.synced_image_pub.publish(image)
        self.synced_camera_info_pub.publish(camera_info)

def main(args=None):
    rclpy.init(args=args)
    node = ImageCameraSynchronizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
