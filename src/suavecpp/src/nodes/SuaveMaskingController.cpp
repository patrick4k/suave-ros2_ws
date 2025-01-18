#include "SuaveMaskingController.h"

void SuaveMaskingController::shutdown()
{
    SuaveController::shutdown();

    std::cout << "SuaveMaskingController::shutdown() called" << std::endl;

    // TODO: Implement me!
}


void SuaveMaskingController::masking_pid_publisher_callback(const QuaternionMsg::SharedPtr msg)
{
    std::cout << "SuaveMaskingController::masking_pid_publisher_callback() called" << std::endl;
}
