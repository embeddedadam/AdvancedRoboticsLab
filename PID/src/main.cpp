#include "controller.hpp"
#include "matplotlibcpp.h"
#include <iostream>
#include <vector>

namespace plt = matplotlibcpp;

int main() {
    PIDController pid(0.1, 0.01, 0.01);

    double setpoint = 100.0;
    double measured_value = 0;

    std::vector<double> setpoints;
    std::vector<double> measured_values;

    for (int i = 0; i < 100; ++i) {
        double output = pid.compute(setpoint, measured_value);
        measured_value += output;

        setpoints.push_back(setpoint);
        measured_values.push_back(measured_value);
    }

    plt::named_plot("Setpoint", setpoints);
    plt::named_plot("Measured Value", measured_values);
    plt::legend();
    plt::show();

    return 0;
}
