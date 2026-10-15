import rclpy
import math
import os
import yaml
from ament_index_python.packages import get_package_share_directory
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

        config_path = os.path.join(
            get_package_share_directory("sentry_decision_sys"),
            "config",
            "decision.yaml",
        )

        with open(config_path) as f:
            data = yaml.safe_load(f)

        self.points = data["points"]
        self.tasks = data["tasks"]
        self.goal_tolerance = data["goal_tolerance"]
        self.stay_margin = data["stay_margin"]
        self.use_game_time = data["use_game_time"]

        self.game_time = 300
        self.task_index = 0
        
        self.odom_x = 0.0
        self.odom_y = 0.0

        self.has_odom = False
        self.stay_start = None

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
        self.game_time = msg.game_time

    def odom_callback(self, msg : Odometry) -> None:
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.has_odom = True

    def chassis_callback(self, msg : ChassisData) -> None:
        pass

    def timer_callback(self):

        if not self.has_odom or self.task_index >= len(self.tasks):
            self.get_logger().info("\33[32m waiting or done \33[0m", throttle_duration_sec=1.0)
            return

        msg_to_planner = DecisionToPlanner()
        msg_to_driver = DecisionToDriver()
        task = self.tasks[self.task_index]
        self.get_logger().info(f"task {self.task_index}: {task['type']} -> {task['point']}", throttle_duration_sec=1.0)
        task_x, task_y = self.points[task["point"]]

        msg_to_planner.is_spinning = task["spin"]
        msg_to_planner.x = task_x
        msg_to_planner.y = task_y
        msg_to_driver.posture = task["posture"]

        if task["type"] == "move":

            if math.fabs(task_x - self.odom_x) < self.goal_tolerance and math.fabs(task_y - self.odom_y) < self.goal_tolerance:
                self.task_index += 1

        elif task["type"] == "stay":

            if self.stay_start is None:
                self.stay_start = self.get_clock().now().nanoseconds / 1e9

            time_rn = self.get_clock().now().nanoseconds / 1e9
            if time_rn - self.stay_start >= task["time"] + self.stay_margin :
                self.stay_start = None
                self.task_index += 1
        
        self.planner_publer.publish(msg_to_planner)
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
