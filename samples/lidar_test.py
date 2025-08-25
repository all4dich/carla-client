#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import PointCloud2

class LidarTopicRepublisher(Node):
    def __init__(self):
        super().__init__('lidar_topic_republisher')
        
        # QoS 설정: BEST_EFFORT로 변경
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # Subscriber: /sensor/lidar/front 토픽 구독
        self.subscription = self.create_subscription(
            PointCloud2,
            '/sensor/lidar/front',
            self.lidar_callback,
            qos
        )
        
        # Publisher: /sensing/lidar/concatenated/pointcloud 토픽으로 발행
        self.publisher = self.create_publisher(
            PointCloud2,
            '/sensing/lidar/concatenated/pointcloud',
            qos
        )
        
        self.get_logger().info('Lidar Topic Republisher Started')
        self.get_logger().info('Subscribing to /sensor/lidar/front with QoS: BEST_EFFORT, VOLATILE')
        self.get_logger().info('Publishing to /sensing/lidar/concatenated/pointcloud')

    def lidar_callback(self, msg):
        # 메시지 수신 및 발행
        self.publisher.publish(msg)
        self.get_logger().info('Received and published PointCloud2')

def main(args=None):
    rclpy.init(args=args)
    node = LidarTopicRepublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Lidar Topic Republisher')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()