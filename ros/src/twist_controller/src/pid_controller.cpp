#include "pid_controller.h"

namespace DBWNODE_NS {

//using namespace std;

PIDController::PIDController() {
}

PIDController::PIDController(const double kp, const double ki, const double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

PIDController::~PIDController() {
    // no-op
}

}
