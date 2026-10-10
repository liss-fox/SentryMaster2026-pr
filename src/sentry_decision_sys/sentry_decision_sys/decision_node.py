import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rm_msgs.msg import RefereeData, ChassisData, DecisionToDriver, DecisionToPlanner
from nav_msgs.msg import Odometry

class DecisionNode(Node):
    def __init__(self) -> None:
        super().__init__("sentry_decision_node")
        # TF 监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 订阅话题
        self.referee_suber = self.create_subscription(
            msg_type=RefereeData,
            topic="/referee_data",
            callback=self.referee_callback,
            qos_profile=10,
        )

        self.odom_suber = self.create_subscription(
            msg_type=Odometry,
            topic="/odin1/odometry_highfreq",
            callback=self.odom_callback,
            qos_profile=10,
        )

        self.chassis_suber = self.create_subscription(
            msg_type=ChassisData,
            topic="/chassis_data",
            callback=self.chassis_callback,
            qos_profile=10,
        )

        self.planner_publer = self.create_publisher(
            msg_type= DecisionToPlanner,
            topic="/decision_to_planner",
            qos_profile=10,
        )

        self.driver_publer = self.create_publisher(
            msg_type= DecisionToDriver,
            topic="/decision_to_driver",
            qos_profile=10,
        )

        timer_period = 0.1
        self.decision_timer =  self.create_timer(timer_period, self.timer_callback)

        print("\33[32m sentry_decision_node has been built \33[0m", flush=True)

    def referee_callback(self, msg : RefereeData) -> None:
        pass

    def odom_callback(self, msg : Odometry) -> None:
        pass

    def chassis_callback(self, msg : ChassisData) -> None:
        pass

    def timer_callback(self):
        msg_to_planner = DecisionToPlanner()
        msg_to_planner.x = 1.0
        msg_to_planner.y = 2.0
        msg_to_planner.is_spinning = False
        self.planner_publer.publish(msg_to_planner)

        msg_to_driver = DecisionToDriver()
        msg_to_driver.posture = 3
        self.driver_publer.publish(msg_to_driver)

def cleanup_node(node: Node) -> None:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

def spin_node(node: Node) -> None:
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

def main(args=None) -> None:
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        spin_node(node)
    finally:
        cleanup_node(node)
