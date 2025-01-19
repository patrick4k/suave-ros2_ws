#include "SuaveController.h"

void SuaveController::start()
{

}

void SuaveController::shutdown()
{
    info("Shutting down %s", this->get_name());
    
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
