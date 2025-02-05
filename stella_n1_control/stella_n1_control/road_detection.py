import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import torch
from cv_bridge import CvBridge

class RoadDetectionNode(Node):
    def __init__(self):
        super().__init__('road_detection_node')

        # YOLO 모델 로드
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)

        # 카메라 데이터 구독 & 노면 상태 Publish
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.road_condition_pub = self.create_publisher(String, '/road_condition', 10)

        self.bridge = CvBridge()

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(frame)

        # 노면 상태 판단
        road_type = "smooth"
        for result in results.xyxy[0]:
            label = int(result[-1])
            if label in [1, 2]:  # 예: 거친 노면 레이블
                road_type = "rough"
            elif label in [3, 4]:  # 예: 매우 거친 노면 레이블
                road_type = "very_rough"

        # Publish
        self.get_logger().info(f'노면 상태: {road_type}')
        msg = String()
        msg.data = road_type
        self.road_condition_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RoadDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
