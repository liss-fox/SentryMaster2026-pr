#include "common.h"

class SentryPlannerNode : public rclcpp::Node {
public:
    SentryPlannerNode(std::string name);

private:
     
    double max_speed {}, goal_tolerance {}, target_x {}, target_y {}, spin_speed {};
    bool is_spinning {}, has_odom {}, has_target {};

    double odom_x {}, odom_y {}, odom_yaw {};
    double yaw_offset {};

    double yaw0 = 0.0;
    bool has_yaw0 = false;
    double heading_kp;

    rclcpp::Subscription<OdomMsg>::SharedPtr odom_suber;
    rclcpp::Subscription<ChassisDataMsg>::SharedPtr chassis_data_suber;
    rclcpp::Subscription<DecisionToPlannerMsg>::SharedPtr decision_suber;

    rclcpp::Publisher<PlannerToDriverMsg>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 坐标变换接收者
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    
    void ChassisInfoCallback(const ChassisDataMsg::SharedPtr msg);
    void OdomCallBack(const OdomMsg::SharedPtr msg);
    void DecisionCallBack(const DecisionToPlannerMsg::SharedPtr msg);
    void TimerCallback();
};
