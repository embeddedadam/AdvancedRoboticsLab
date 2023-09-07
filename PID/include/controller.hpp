#pragma once

#include <tuple>
class PIDController {
public:
    PIDController(double kp, double ki, double kd);
    double compute(double setpoint, double measured_value);
    void setGains(double kp, double ki, double kd);
    void reset();
    std::tuple<double, double, double> getGains() const;

private:
    double kp_, ki_, kd_;
    double previous_error_;
    double integral_;
};
