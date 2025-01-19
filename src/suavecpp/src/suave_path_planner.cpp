#include <iostream>

#include "nodes/SuavePathPlanner.h"

std::shared_ptr<SuavePathPlanner> s_controller = nullptr;

void signal_handler(int /*sig*/)
{
    if (s_controller)
    {
        s_controller->shutdown();
    }
    rclcpp::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::vector signals = { SIGINT, SIGTERM, SIGQUIT };
    for (const auto sig : signals)
    {
        std::signal(sig, signal_handler);
    }
    
    try 
    {
        s_controller = std::make_shared<SuavePathPlanner>();
        s_controller->start();
        rclcpp::spin(s_controller);
    } 
    catch (const std::exception& e) 
    {
        std::cout << "suavecpp suave_path_planner had an unexpected error!" << std::endl;
    }

    rclcpp::shutdown();
}
