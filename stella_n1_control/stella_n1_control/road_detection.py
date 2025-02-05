import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import torch
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy

class RoadDetectionNode(Node):
    def __init__(self):
        super().__init__('road_detection_node')

        # YOLO 모델 로드
        self.get_logger().info("YOLOv5 모델 로드 중...")
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)
        self.get_logger().info("YOLOv5 모델 로드 완료!")

        # QoS 설정 (BEST_EFFORT)
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # 카메라 데이터 구독 & 노면 상태 Publish
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, qos_profile)
        self.road_condition_pub = self.create_publisher(String, '/road_condition', 10)

        self.bridge = CvBridge()

    def image_callback(self, msg):
        self.get_logger().info("🔹 이미지 콜백 실행됨 (카메라 데이터 수신)")  # 콜백 확인 로그 추가

        try:
            # 카메라 데이터 변환
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # YOLO 실행
            results = self.model(frame)
            self.get_logger().info("🔹 YOLO 분석 완료")

            # 노면 상태 판단
            road_type = "smooth"
            for result in results.xyxy[0]:
                label = int(result[-1])
                if label in [1, 2]:  # 예: 거친 노면 레이블
                    road_type = "rough"
                elif label in [3, 4]:  # 예: 매우 거친 노면 레이블
                    road_type = "very_rough"

            # 결과 Publish
            self.get_logger().info(f'🚀 노면 상태: {road_type}')
            msg = String()
            msg.data = road_type
            self.road_condition_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"❌ 이미지 처리 중 오류 발생: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RoadDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

