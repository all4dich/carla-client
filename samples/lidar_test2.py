#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import carla
import time

class CarlaLidarPublisher(Node):
    def __init__(self):
        super().__init__('carla_lidar_publisher')
        
        # ROS2 QoS 설정
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # ROS2 Publisher: /sensor/lidar/front 토픽
        self.publisher = self.create_publisher(
            PointCloud2,
            '/sensing/lidar/concatenated/pointcloud',
            qos
        )
        
        # CARLA 클라이언트 연결 (포트 1403)
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        
        # 차량 및 LiDAR 설정
        self.spawn_vehicle_and_lidar()
        
        self.get_logger().info('Carla Lidar Publisher Started')
        self.get_logger().info('Publishing to /sensor/lidar/front with frame_id: velodyne')

    def spawn_vehicle_and_lidar(self):
        # 블루프린트 라이브러리
        blueprint_library = self.world.get_blueprint_library()
        
        # 사용 가능한 차량 블루프린트 확인
        vehicle_bps = blueprint_library.filter('vehicle.*.*')
        if not vehicle_bps:
            self.get_logger().error('No vehicle blueprints found!')
            for bp in blueprint_library.filter('vehicle.*'):
                self.get_logger().info(f'Available blueprint: {bp.id}')
            raise RuntimeError('No vehicle blueprints available')
        
        # 첫 번째 사용 가능한 차량 선택
        vehicle_bp = vehicle_bps[0]
        self.get_logger().info(f'Selected vehicle blueprint: {vehicle_bp.id}')
        
        # 스폰 포인트 선택
        spawn_points = self.world.get_map().get_spawn_points()
        if not spawn_points:
            self.get_logger().error('No spawn points available in the map!')
            raise RuntimeError('No spawn points available')
        
        spawn_point = spawn_points[0]
        self.get_logger().info(f'Spawning vehicle at: {spawn_point.location}')
        
        # 차량 소환
        self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
        self.vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))  # 초기 속도 0
        self.get_logger().info(f'Spawned vehicle: {self.vehicle.type_id}')
        
        # LiDAR 센서 설정
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', '32')
        lidar_bp.set_attribute('range', '50.0')
        lidar_bp.set_attribute('points_per_second', '100000')
        lidar_bp.set_attribute('rotation_frequency', '10.0')
        
        # LiDAR 위치 (차량 위 2m)
        lidar_transform = carla.Transform(carla.Location(x=0, y=0, z=-2))
        self.lidar = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.vehicle)
        
        # LiDAR 데이터 콜백
        self.lidar.listen(self.lidar_callback)
        self.get_logger().info('LiDAR sensor attached and listening')

    def lidar_callback(self, lidar_data):
        # LiDAR 데이터를 NumPy 배열로 변환 (CARLA 0.10.0)
        points = []
        for detection in lidar_data:
            point = detection.point  # LidarDetection의 point 속성 사용
            points.append([point.x, point.y, point.z, detection.intensity])
        points = np.array(points, dtype=np.float32)
        
        # 터미널에 포인트 수 및 첫 번째 포인트 출력
        self.get_logger().info(f'Received {len(points)} points from LiDAR')
        if len(points) > 0:
            self.get_logger().info(f'First point: x={points[0][0]}, y={points[0][1]}, z={points[0][2]}, intensity={points[0][3]}')
        
        # PointCloud2 메시지 생성
        cloud_msg = PointCloud2()
        cloud_msg.header = Header()
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        cloud_msg.header.frame_id = 'base_link'
        
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        
        # 필드 설정 (x, y, z, intensity)
        cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 16  # 4 floats * 4 bytes
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = True
        cloud_msg.data = points.tobytes()
        
        # ROS2 토픽으로 발행
        self.publisher.publish(cloud_msg)
        self.get_logger().info('Published PointCloud2 to /sensor/lidar/front')

    def destroy(self):
        # 소멸자: 차량과 LiDAR 정리
        if hasattr(self, 'lidar'):
            self.lidar.destroy()
        if hasattr(self, 'vehicle'):
            self.vehicle.destroy()
        self.get_logger().info('Cleaned up CARLA actors')

def main(args=None):
    rclpy.init(args=args)
    node = CarlaLidarPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Carla Lidar Publisher')
    except Exception as e:
        node.get_logger().error(f'Error occurred: {str(e)}')
    finally:
        node.destroy()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()