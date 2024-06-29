#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2, socket, numpy, pickle
import os

class VideoPublisherNode(Node):
    def __init__(self):
        super().__init__('video_publisher_node')

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        ip = "10.42.0.1"
        port = 5000
        self.sock.bind((ip, port))

        self.publisher = self.create_publisher(Image, 'video_frames', 10)
        self.timer = self.create_timer(0.033, self.publish_frame)  # 30 FPS (1/30 ≈ 0.033)

        self.bridge = CvBridge()

    def publish_frame(self):
        x = self.sock.recvfrom(100000000)
        client_ip = x[1][0]
        data = x[0]
        data = pickle.loads(data)
        data = cv2.imdecode(data, cv2.IMREAD_COLOR)
        data = cv2.cvtColor(data,cv2.COLOR_BGR2RGB)

        self.get_logger().info('Publishing video frame')
        image_msg = self.bridge.cv2_to_imgmsg(data)
        self.publisher.publish(image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisherNode()
    try:
        print('hi')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
