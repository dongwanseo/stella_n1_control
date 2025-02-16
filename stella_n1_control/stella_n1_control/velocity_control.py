import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class VelocityControlNode(Node):
    def __init__(self):
        super().__init__('velocity_control_node')

        # 노면 상태 구독 & 속도 Publish
        self.road_condition_sub = self.create_subscription(String, '/road_condition', self.road_condition_callback, 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def road_condition_callback(self, msg):
        road_type = msg.data
        speed = self.get_speed_from_condition(road_type)

        # 속도 메시지 생성
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0  # 회전 없음
        self.vel_pub.publish(twist)

        self.get_logger().info(f'노면 상태: {road_type}, 속도: {speed:.2f} m/s')
    def destroy_node(self):
        """ 노드 종료 시 모터 정지 명령 전송 """
        twist = Twist()
        twist.linear.x = 0.0  # 정지
        twist.angular.z = 0.0
        self.vel_pub.publish(twist)  # 정지 명령 퍼블리시
        self.get_logger().info("🛑 노드 종료: 모터 정지 명령 전송 완료")
        super().destroy_node()

    def get_speed_from_condition(self, road_type):
        """ 노면 상태에 따라 속도 설정 """
        if road_type == "smooth":
            return 1.0  # 기본 속도
        elif road_type == "rough":
            return 0.6  # 감속
        elif road_type == "very_rough":
            return 0.3  # 강한 감속
        return 0.5  # 기본 감속

def main(args=None):
    rclpy.init(args=args)
    node = VelocityControlNode()
    try:
        rclpy.spin(node)  # 노드 실행
    except KeyboardInterrupt:
        node.get_logger().info("🛑 KeyboardInterrupt: 노드 종료 중...")
    finally:
        node.destroy_node()  # 노드 종료 시 모터 정지
        rclpy.shutdown()

if __name__ == '__main__':
    main()
