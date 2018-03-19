#include "twist_controller.h"

namespace DBWNODE_NS {

Controller::Controller() {
    // no-op
}

PredictedControlValues Controller::control() {
    // Throttle values passed to publish should be in the range 0 to 1
    float throttle = 12;
    // Brake values passed to publish should be in units of torque (N*m). 
    // The correct values for brake can be computed using the desired acceleration, weight of the vehicle, and wheel radius.
    float brake = 0;
    float steer = 0;

    // Can import and use the provided pid.py and lowpass.py if needed for acceleration
    // Can import yaw_controller.py for steering
    return PredictedControlValues(throttle, brake, steer);
}

Controller::~Controller() {
    // no-op
}

}
