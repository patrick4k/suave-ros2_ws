#include <iostream>

#include "nodes/SuaveMaskingController.h"

using Controller = SuaveMaskingController; // Specify which controller you want to use here

std::shared_ptr<Controller> s_controller = nullptr;

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
        s_controller = std::make_shared<Controller>();
        rclcpp::spin(s_controller);
    } 
    catch (const std::exception& e) 
    {
        std::cout << "suavecpp suave_controller had an unexpected error!" << std::endl;
    }

    rclcpp::shutdown();
}
