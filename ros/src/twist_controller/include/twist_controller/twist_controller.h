#ifndef TWIST_CONTROLLER__H
#define TWIST_CONTROLLER__H

#include "predicted_control_values.h"
#include "yaw_controller.h"
#include "pid_controller.h"

namespace DBWNODE_NS {

class Controller {
    public:
        Controller();
        PredictedControlValues control();
        ~Controller();

    private:
        YawController yawController;
        PIDController pidController;
};
}

#endif
