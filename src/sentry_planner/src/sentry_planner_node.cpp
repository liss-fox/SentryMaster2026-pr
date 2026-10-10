#include "sentry_planner_node.hpp"

using namespace std::chrono_literals;

SentryPlannerNode::SentryPlannerNode(std::string name):Node(name){
    tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface()
    );
    tf_buffer->setCreateTimerInterface(timer_interface);
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    odom_suber = create_subscription<OdomMsg>("/odin1/odometry_highfreq", 10, std::bind(&SentryPlannerNode::OdomCallBack, this, std::placeholders::_1));
    chassis_data_suber = create_subscription<ChassisDataMsg>("/chassis_data", 10, std::bind(&SentryPlannerNode::ChassisInfoCallback, this, std::placeholders::_1));
    decision_suber = create_subscription<DecisionToPlannerMsg>("/decision_to_planner", 10, std::bind(&SentryPlannerNode::DecisionCallBack, this, std::placeholders::_1));

    publisher_ = this->create_publisher<PlannerToDriverMsg>("/planner_cmd", 10);
    timer_ = this->create_wall_timer(20ms, std::bind(&SentryPlannerNode::TimerCallback, this));

    info_log(name, "has been built!");
}

// 底盘信息回调
void SentryPlannerNode::ChassisInfoCallback(const ChassisDataMsg::SharedPtr msg) {
    
}

// 定位节点位置回调
void SentryPlannerNode::OdomCallBack(const OdomMsg::SharedPtr msg) {

}

void SentryPlannerNode::DecisionCallBack(const DecisionToPlannerMsg::SharedPtr msg){
    target_x = msg->x;
    target_y = msg->y;
    is_spinning = msg->is_spinning;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "Target: x=%f y=%f spin=%d", target_x, target_y, is_spinning);
}

void SentryPlannerNode::TimerCallback() {
    auto message = PlannerToDriverMsg();
    message.vx = 1.0f;
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published: '%f'", message.vx);
}
