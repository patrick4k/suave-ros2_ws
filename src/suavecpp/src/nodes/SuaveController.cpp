#include "SuaveController.h"

void SuaveController::shutdown()
{
    std::cout << "SuaveController::shutdown() called" << std::endl;    
    
    // TODO: Implement me!

}

void SuaveController::odom_callback(const OdomMsg::SharedPtr msg)
{
    std::cout << "SuaveController::odom_callback() called" << std::endl;
}

void SuaveController::cloud_map_callback(const CloudMapMsg::SharedPtr msg)
{
    std::cout << "SuaveController::cloud_map_callback() called" << std::endl;
}
