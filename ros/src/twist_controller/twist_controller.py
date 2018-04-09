import rospy

from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter


class Controller(object):

    # Class Attributes
    Gas_Density = 2.858

    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                 accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        min_speed = 0.1
        self.yaw_controller = YawController(
            wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        self.throttle_controller = PID(
            kp=0.3,
            ki=0.1,
            kd=0.,
            mn=0.,  # Minimum throttle value
            mx=0.8  # Maximum throttle value
        )

        self.vel_low_pass_filter = LowPassFilter(
            tau=0.5,  # 1/(2pi*tau) = cutoff frequency
            ts=.02  # Sample time
        )

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel, fuel_level):
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        current_vel = self.vel_low_pass_filter.filt(current_vel)

        steering = self.yaw_controller.get_steering(
            linear_vel, angular_vel, current_vel)

        velocity_error = linear_vel - current_vel
        self.last_velocity = current_vel

        current_time = rospy.get_time()
        time_elapsed = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(velocity_error, time_elapsed)
        brake = 0

        current_vehicle_mass = self.vehicle_mass + \
            fuel_level / 100 * self.fuel_capacity * Controller.Gas_Density

        if linear_vel == 0. and current_vel < 0.1:
            throttle = 0
            brake = 400  # N*m to hold the car in place if we are stopped at a light. Acceleration - 1m/s^2
        elif throttle < .1 and velocity_error < 0:
            throttle = 0
            deceleration = max(velocity_error, self.decel_limit)
            brake = abs(deceleration) * current_vehicle_mass * \
                self.wheel_radius  # Torque N*m

        return throttle, brake, steering
