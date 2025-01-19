#include <iostream>
#include <memory>
#include <vector>

#include "nodes/SuaveController.h"

std::shared_ptr<SuaveController> s_controller = nullptr;

void signal_handler(int /*sig*/)
{
    if (s_controller)
    {
        s_controller->shutdown();
    }
    rclcpp::shutdown();
    exit(0);
}

template<typename TController>
int start(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::vector signals = { SIGINT, SIGTERM, SIGQUIT };
    for (const auto sig : signals)
    {
        std::signal(sig, signal_handler);
    }
    
    try 
    {
        s_controller = std::make_shared<TController>();
        s_controller->start();
        rclcpp::spin(s_controller);
    } 
    catch (const std::exception& e) 
    {
        std::cout << "suavecpp suave_path_planner had an unexpected error!" << std::endl;
    }

    rclcpp::shutdown();

    return 0;
}
