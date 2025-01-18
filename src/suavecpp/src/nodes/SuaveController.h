#pragma once
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.12/pcl/io/pcd_io.h>

class SuaveController : public rclcpp::Node
{
public:
    SuaveController() : rclcpp::Node("suave_controller")
    {
        // Initialize ROS topics here
    }

    virtual void shutdown();

private:
    // Subscriptions ///////////////////////////////////////////////////

    // odom
    using OdomMsg = nav_msgs::msg::Odometry;
    rclcpp::Subscription<OdomMsg>::SharedPtr m_odom_subscription{ nullptr };
    void odom_callback(const OdomMsg::SharedPtr msg);

    // cloud_map
    using CloudMapMsg = sensor_msgs::msg::PointCloud2;
    rclcpp::Subscription<CloudMapMsg>::SharedPtr m_cloud_map_subscription{ nullptr };
    void cloud_map_callback(const CloudMapMsg::SharedPtr msg);
    CloudMapMsg::SharedPtr m_cloud_map{ nullptr };
};
