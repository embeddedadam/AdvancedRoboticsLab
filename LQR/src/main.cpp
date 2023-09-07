#include "matplotlibcpp.h"
#include "lqr.hpp"
#include <Eigen/Dense>

namespace plt = matplotlibcpp;

int main() {
    Eigen::MatrixXd A(1, 1), B(1, 1), Q(1, 1), R(1, 1);
    A << 1;
    B << 1;
    Q << 1;
    R << 1;

    LQR lqr(A, B, Q, R);

    Eigen::VectorXd x(1);
    x << 10;  // Initial state

    std::vector<double> states, times;
    double dt = 0.1;

    for(double t=0; t<10; t+=dt) {
        Eigen::VectorXd u = lqr.control(x);
        x = A * x + B * u;

        states.push_back(x(0));
        times.push_back(t);
    }

    plt::plot(times, states);
    plt::title("LQR Controller Behavior");
    plt::xlabel("Time");
    plt::ylabel("State");
    plt::grid(true);
    plt::show();

    return 0;
}
