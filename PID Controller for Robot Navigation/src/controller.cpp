#include <tuple>
#include "controller.hpp"

PIDController::PIDController(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd), previous_error_(0), integral_(0) {}

double PIDController::compute(double setpoint, double measured_value) {
    double error = setpoint - measured_value;
    integral_ += error;
    double derivative = error - previous_error_;
    
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    
    previous_error_ = error;
    return output;
}

void PIDController::setGains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

std::tuple<double, double, double> PIDController::getGains() const{
    return std::make_tuple(kp_, ki_, kd_);
}

void PIDController::reset() {
    previous_error_ = 0;
    integral_ = 0;
}
