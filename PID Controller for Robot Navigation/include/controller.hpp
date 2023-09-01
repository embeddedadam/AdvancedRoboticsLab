#pragma once

class PIDController {
public:
    PIDController(double kp, double ki, double kd);
    double compute(double setpoint, double measured_value);
    void reset();

private:
    double kp_, ki_, kd_;
    double previous_error_;
    double integral_;
};
