
import rclpy
from rclpy.node import Node
import numpy as np
import sys
import os
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# vl53l8cx 모듈을 찾을 수 있도록 경로 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
vl53l5cx_path = os.path.join(current_dir, "..", "vl53l5cx")
sys.path.append(vl53l5cx_path)

from vl53l8cx.data_collect import TOFSensor
from vl53l8cx.api import VL53L8CX_RESOLUTION_8X8


class VL53L8CXPublisher(Node):
    def __init__(self):
        super().__init__('vl53l8cx_publisher')

        # VL53L5CX 센서 초기화 (8x8 해상도)
        self.sensor = TOFSensor(resolution=VL53L8CX_RESOLUTION_8X8)

        # QoS 설정 (Reliable로 변경)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # PointCloud2 퍼블리셔
        self.publisher_ = self.create_publisher(PointCloud2, 'vl53l8cx/points', qos_profile)
        self.timer = self.create_timer(0.15, self.publish_pointcloud)  # 15Hz 주기로 실행

        self.frame_id = "vl53l8cx_frame"
        self.res = 8  # 8x8 해상도
        self.get_logger().info("VL53L8CX PointCloud2 Publisher Started!")

    def read_tof_data(self):
        """TOF 센서에서 실제 거리 데이터를 읽고 XYZ 좌표로 변환"""
        try:
            distance_values = self.sensor.get_data()  # 기존 visualize 코드 방식 적용
            if not distance_values: # 데이터가 안들어왔을때 3번 대기
                self.fail_count += 1
                self.get_logger().warn(f"TOF data not ready(attempt {self.fail_count}/3)")
                if self.fail_count >= 3:
                    self.get_logger().error("Failed to get TOF data 3 times, reinitializing sensor...")
                    self.sensor._initialize_driver()
                    time.sleep(0.5)  # 안정화를 위해 0.5초 대기
                    self.fail_count = 0
                return None

            self.fail_count = 0
            # 거리 데이터를 저장할 배열 초기화
            distance_value = np.full((64,), 4000, dtype=np.float32)  # 기본값 4000mm (센서 읽기 오류 대비)
            
            for i in range(len(distance_values)):
                zone = distance_values[i]['zone']
                status = distance_values[i]['Status']
                if status == 5:  # 유효한 거리 데이터만 반영
                    distance_value[zone] = distance_values[i]['Distance(mm)']

            # 기존 buf 초기화(메모리 누수 방지)
            if hasattr(self, "last_buf") and self.last_buf is not None:
                del self.last_buf
                self.last_buf = None

            # 3D 좌표 변환 (z는 실제 거리값)
            buf = np.empty((8, 8, 3), dtype=np.float32)
            flipped_data = np.fliplr(distance_value.reshape(8, 8))  # `visualize.py`와 동일하게 좌우 반전

            for i in range(8):
                for j in range(8):
                    x = (i - 3.5) * 0.05  # 가로 위치 보정
                    y = (j - 3.5) * 0.05  # 세로 위치 보정
                    z = flipped_data[i, j] / 1000.0  # mm -> m 변환
                    buf[i, j] = [x, y, z]

            self.last_buf = buf # last_buf로 저장
            return buf
        except Exception as e:
            self.get_logger().error(f"Error reading TOF data: {e}")
            return None

    def publish_pointcloud(self):
        """PointCloud2 메시지를 생성하고 퍼블리시"""
        points = self.read_tof_data()
        if points is None:
            return  # 데이터가 없으면 퍼블리시 중단

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        cloud_msg = PointCloud2(
            header=header,
            height=self.res,
            width=self.res,
            fields=[
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
            ],
            is_bigendian=False,
            is_dense=True,
            point_step=12,  # float32 (4 bytes) * 3 (x, y, z)
            row_step=12 * self.res,
            data=points.tobytes()
        )

        self.publisher_.publish(cloud_msg)
        # self.get_logger().info("Published TOF PointCloud2 data")

def main(args=None):
    rclpy.init(args=args)
    node = VL53L8CXPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
