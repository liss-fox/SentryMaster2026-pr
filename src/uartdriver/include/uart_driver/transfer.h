#pragma once
#include "cmath"
#include "common.h"
#include "serial/serial.h"
#include "struct_def.hpp"

class Transfer{
private:
    rclcpp::Node *node = nullptr;
    rclcpp::Publisher<RefereeMsg>::SharedPtr referee_puber;
    rclcpp::Publisher<ChassisMsg>::SharedPtr chassis_puber;

    rclcpp::Subscription<PlannerToDriverMsg>::SharedPtr planner_message_suber;
    rclcpp::Subscription<DecisionToDriverMsg>::SharedPtr decision_message_suber;

    std::shared_ptr<serial::Serial> serial_port;

    ChassisControl chassis_control{};
    bool is_spining{};

    bool send_serial_packet(const std::vector<uint8_t> &datas, const std::string &tag);
    void PlannerMessageCallBack(const PlannerToDriverMsg::SharedPtr msg);
    void DecisionMessageCallBack(const DecisionToDriverMsg::SharedPtr msg);

public:
    Transfer(rclcpp::Node *node, std::shared_ptr<serial::Serial> _serial_port);

    void handle_datas(const std::vector<uint8_t> &datas);
    void pub_referee_msg(const RefereeData &referee_info);
    void pub_chassis_msg(const ChassisData &chassis_info);
};
