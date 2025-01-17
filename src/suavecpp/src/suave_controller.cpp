#include <rclcpp/utilities.hpp>
#include <iostream>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::cout << "suavecpp suave_controller" << std::endl;
    rclcpp::shutdown();
}
