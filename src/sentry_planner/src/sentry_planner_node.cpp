#include "sentry_planner_node.hpp"
#include <algorithm>

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
    spin_speed = this->declare_parameter<double>("spin_speed", 3.0);
    yaw_offset = this->declare_parameter<double>("yaw_offset", 0.0);
    heading_kp = this->declare_parameter<double>("heading_kp", 0.0);

    info_log(name, "has been built! Parameters: ", max_speed, goal_tolerance, spin_speed);
}

// 底盘信息回调
void SentryPlannerNode::ChassisInfoCallback(const ChassisDataMsg::SharedPtr msg) {
    
}

// 定位节点位置回调
void SentryPlannerNode::OdomCallBack(const OdomMsg::SharedPtr msg) {
    odom_x = msg->pose.pose.position.x;
    odom_y = msg->pose.pose.position.y;
    odom_yaw = tf2::getYaw(msg->pose.pose.orientation) + yaw_offset;
    if (!has_yaw0) { yaw0 = odom_yaw; has_yaw0 = true; }
    has_odom = true;
}

void SentryPlannerNode::DecisionCallBack(const DecisionToPlannerMsg::SharedPtr msg){
    has_target = true;
    target_x = msg->x;
    target_y = msg->y;
    is_spinning = msg->is_spinning;
}

void SentryPlannerNode::TimerCallback() {
    auto message = PlannerToDriverMsg();
    
    if (!has_odom || !has_target){
        publisher_->publish(message);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "No odom or target, stop: '%f' '%f'", message.vx, message.vy);
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

    if (is_spinning) {
        message.wz = spin_speed;
    } else {
        double e = std::atan2(std::sin(yaw0 - odom_yaw), std::cos(yaw0 - odom_yaw));
        message.wz = std::clamp(heading_kp * e, -1.0, 1.0);
    }
    
    publisher_->publish(message);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "goal (%.1f, %.1f) dist %.2f | v %.2f %.2f w %.1f | pos %.2f %.2f yaw %.2f",
    target_x, target_y, dist, message.vx, message.vy, message.wz,
    odom_x, odom_y, odom_yaw);
}
