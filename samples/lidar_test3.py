#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import PointCloud2
import os
import subprocess
import time

class PointCloudVisualizer(Node):
    def __init__(self):
        super().__init__('point_cloud_visualizer')
        
        # RViz2 설정 파일 생성
        self.rviz_config_path = os.path.expanduser('~/point_cloud_visualizer.rviz')
        self.create_rviz_config()
        
        # ROS2 QoS 설정
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # Subscriber: /sensing/lidar/concatenated/pointcloud 토픽 구독
        self.subscription = self.create_subscription(
            PointCloud2,
            '/sensing/lidar/concatenated/pointcloud',
            self.pointcloud_callback,
            qos
        )
        
        # RViz2 프로세스 실행
        try:
            self.rviz_process = subprocess.Popen(['ros2', 'run', 'rviz2', 'rviz2', '-d', self.rviz_config_path])
            self.get_logger().info(f'RViz2 launched with config: {self.rviz_config_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to launch RViz2: {str(e)}')
        
        self.get_logger().info('Point Cloud Visualizer Started')

    def create_rviz_config(self):
        # RViz2 설정 파일 내용
        rviz_config = """
RViz:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/PointCloud2
      Enabled: true
      Name: PointCloud2
      Topic: /sensing/lidar/concatenated/pointcloud
      Queue Size: 10
      Style: Points
      Size (m): 0.05
      Color Transformer: Intensity
      Min Intensity: 0
      Max Intensity: 1
      Decay Time: 0
      Channel Name: intensity
      Fixed Frame: base_link
      Alpha: 1.0
  Enabled: true
  Fixed Frame: base_link
  Name: ""
  Tool Manager:
    Tools:
      - Class: rviz_default_plugins/Interact
      - Class: rviz_default_plugins/MoveCamera
      - Class: rviz_default_plugins/Select
      - Class: rviz_default_plugins/FocusCamera
      - Class: rviz_default_plugins/Measure
      - Class: rviz_default_plugins/PublishPoint
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10
      Pitch: 0.314
      Yaw: 0
      Focal Point:
        X: 0
        Y: 0
        Z: 0
    Saved: []
Visualization Manager:
  Class: ""
  Displays: []
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: base_link
    Frame Rate: 30
  Name: ""
  Tools: []
  Views: []
"""
        # 설정 파일 저장
        try:
            with open(self.rviz_config_path, 'w') as f:
                f.write(rviz_config)
            self.get_logger().info(f'Created RViz2 config file: {self.rviz_config_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to create RViz2 config file: {str(e)}')

    def pointcloud_callback(self, msg):
        # 포인트 클라우드 수신 로그
        self.get_logger().info(
            f'Received PointCloud2: frame_id={msg.header.frame_id}, '
            f'points={msg.width}, timestamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}'
        )

    def destroy_node(self):
        # 소멸자: RViz2 프로세스 종료
        if hasattr(self, 'rviz_process') and self.rviz_process:
            self.rviz_process.terminate()
            self.rviz_process.wait(timeout=5.0)
            self.get_logger().info('RViz2 process terminated')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Point Cloud Visualizer')
    except Exception as e:
        node.get_logger().error(f'Error occurred: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()