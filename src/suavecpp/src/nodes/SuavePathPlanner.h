#pragma once
#include "SuaveController.h"

class SuavePathPlanner : public SuaveController
{
public:
    SuavePathPlanner() : SuaveController("suave_path_planner")
    {
        // Initialize ROS topics here
    }

    void start() override;

    void shutdown() override;

private:
    // Subscriptions ///////////////////////////////////////////////////

};
