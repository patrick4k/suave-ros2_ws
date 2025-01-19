#pragma once
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.12/pcl/io/pcd_io.h>

/*
This class serves the following purposes:
- Inherits rclcpp::Node such that all children of this class are also ROS2 nodes.
- To construct and interact with the MAVSDK system that is our drone.
- Localize by subscribing to the /odom topic, transforming the odom and sending it to PX4 via MAVSDK.
- Export the point cloud data by subscribing to /cloud_map and saving the pointer to the data each update. This will be exported on shutdown.
*/

class SuaveController : public rclcpp::Node
{
public:
    virtual void start();

    virtual void shutdown();

protected:
    SuaveController(std::string node_name) : rclcpp::Node(node_name)
    {
        info("Initializing %s", this->get_name());
        // Initialize ROS topics here
    }

    template<typename... T>
    void info(T... msg)
    {
        RCLCPP_INFO(this->get_logger(), msg...);
    }

    template<typename... T>
    void warn(T... msg)
    {
        RCLCPP_WARN(this->get_logger(), msg...);
    }

    template<typename... T>
    void error(T... msg)
    {
        RCLCPP_ERROR(this->get_logger(), msg...);
    }

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
