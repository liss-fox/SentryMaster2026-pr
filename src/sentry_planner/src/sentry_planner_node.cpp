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

    publisher_ = this->create_publisher<PlannerToDriverMsg>("/planner_to_driver", 10);
    timer_ = this->create_wall_timer(20ms, std::bind(&SentryPlannerNode::TimerCallback, this));

    max_speed = this->declare_parameter<double>("max_speed", 0.5);
    goal_tolerance = this->declare_parameter<double>("goal_tolerance", 0.1);

    info_log(name, "has been built! Parameters: ", max_speed, goal_tolerance);
}

// 底盘信息回调
void SentryPlannerNode::ChassisInfoCallback(const ChassisDataMsg::SharedPtr msg) {
    
}

// 定位节点位置回调
void SentryPlannerNode::OdomCallBack(const OdomMsg::SharedPtr msg) {
    odom_x = msg->pose.pose.position.x;
    odom_y = msg->pose.pose.position.y;
    odom_yaw = tf2::getYaw(msg->pose.pose.orientation);
    has_odom = true;
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
    
    if (!has_odom){
        publisher_->publish(message);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "No odom, stop: '%f' '%f'", message.vx, message.vy);
        return;
    }
    
    double dx = target_x - odom_x;
    double dy = target_y - odom_y;
    double dist = std::hypot(dx, dy);

    if (dist < goal_tolerance){
        message.vx = 0.0;
        message.vy = 0.0;
    }
    else {
    // speed on the field
    double vx_field = dx / dist * max_speed;
    double vy_field = dy / dist * max_speed;

    // speen for the robot with respect to turning
    message.vx =  std::cos(odom_yaw) * vx_field + std::sin(odom_yaw) * vy_field;
    message.vy = -std::sin(odom_yaw) * vx_field + std::cos(odom_yaw) * vy_field;
    }

    publisher_->publish(message);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "Published: '%f' '%f'", message.vx, message.vy);
}
