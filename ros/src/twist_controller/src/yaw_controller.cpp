#include "yaw_controller.h"

namespace DBWNODE_NS {

using namespace std;

YawController::YawController(const double wheel_base, const double steer_ratio, const double min_speed, 
                            const double max_lat_accel, const double max_steer_angle) {
    wheel_base_ = wheel_base;
    steer_ratio_ = steer_ratio;
    min_speed_ = min_speed;
    max_lat_accel_ = max_lat_accel;
    min_angle_ = -max_steer_angle;
    max_angle_ = max_steer_angle;
}

YawController::YawController() {
    // no-op
}

YawController::~YawController() {
    // no-op
}

double YawController::get_angle(const double radius) {
    double angle = atan(this->wheel_base_ / radius) * this->steer_ratio_;
    return max(min_angle_, min(max_angle_, angle));
}

double YawController::get_steering(const double linear_velocity, const double angular_velocity, const double current_velocity) {
    double angular_vel = 0.0;
    if (fabs(linear_velocity) > 0.0) {
        angular_vel = current_velocity * angular_velocity / linear_velocity;
    }
    
    if (fabs(current_velocity) > 0.1) {
        double max_yaw_rate = fabs(max_lat_accel_ / current_velocity);
        angular_vel = max(-max_yaw_rate, min(max_yaw_rate, angular_vel));
    }
    
    if (fabs(angular_vel) > 0.0) {
        return get_angle(max(current_velocity, min_speed_) / angular_vel);
    } else {
        return 0.0;
    }
}

}
