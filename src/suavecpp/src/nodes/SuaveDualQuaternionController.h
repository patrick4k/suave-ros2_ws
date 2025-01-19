#pragma once
#include "SuaveController.h"

class SuaveDualQuaternionController : public SuaveController
{
public:
    SuaveDualQuaternionController() : SuaveController("suave_dual_quaternion_controller")
    {
        // Initialize ROS topics here
    }

    void start() override;

    void shutdown() override;

private:
    // Subscriptions ///////////////////////////////////////////////////

};
