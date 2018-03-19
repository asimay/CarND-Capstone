#include "predicted_control_values.h"

namespace DBWNODE_NS {

PredictedControlValues::PredictedControlValues() {
    // no-op
}

PredictedControlValues::PredictedControlValues(const float throttle, const float brake, const float steer) {
    throttle_ = throttle;
    brake_ = brake;
    steer_ = steer;
}

float PredictedControlValues::throttle() {
    return throttle_;
}

float PredictedControlValues::brake() {
    return brake_;
}

float PredictedControlValues::steer() {
    return steer_;
}

PredictedControlValues::~PredictedControlValues() {
    // no-op
}

}
