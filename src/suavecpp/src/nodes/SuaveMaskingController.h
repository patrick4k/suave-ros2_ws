#pragma once
#include "SuaveController.h"

#include <geometry_msgs/msg/quaternion.hpp>

class SuaveMaskingController : public SuaveController
{
public:
    SuaveMaskingController() : SuaveController()
    {
        // Initialize ROS topics here
    }

    void shutdown() override;

private:
    // Subscriptions ///////////////////////////////////////////////////

    // masking_pid_publisher
    using QuaternionMsg = geometry_msgs::msg::Quaternion;
    rclcpp::Subscription<QuaternionMsg>::SharedPtr m_masking_pid_publisher_subscription{ nullptr };
    void masking_pid_publisher_callback(const QuaternionMsg::SharedPtr msg);
};
