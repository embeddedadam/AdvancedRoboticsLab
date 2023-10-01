#include "mpc.hpp"
#include <iostream>

int main() {
    MPCController mpc(10, 0.1, 10.0);
    Eigen::Vector2d x = Eigen::Vector2d::Zero();

    for (int i = 0; i < 100; i++) {
        double u = mpc.control(x);
        x = Eigen::MatrixXd(2, 2) << 1, 0.1, 0, 1 * x + Eigen::MatrixXd(2, 1) << 0.5 * 0.1 * 0.1, 0.1 * u;
        std::cout << "Step " << i << ": Position = " << x(0) << ", Velocity = " << x(1) << ", Control = " << u << std::endl;
    }

    return 0;
}
