#include "lqg.hpp"
#include "matplotlibcpp.h"

#include <vector>

namespace plt = matplotlibcpp;

int main() {
    // Define 1D system matrices
    Eigen::MatrixXd A(1,1), B(1,1), C(1,1), Q(1,1), R(1,1), Q_kf(1,1), R_kf(1,1);
    A << 1.0;
    B << 0.5;
    C << 1.0;
    Q << 1.0;
    R << 1.0;
    Q_kf << 1.0;
    R_kf << 0.5;

    LQG lqg(A, B, C, Q, R, Q_kf, R_kf);

    Eigen::VectorXd x_true(1), x_hat(1), y(1), u(1);
    x_true << 0.0;
    x_hat << 0.0;

    const int T = 1000;
    std::vector<double> x_history, u_history, y_history;

    for (int t = 0; t < T; t++) {
        y = C * x_true + Eigen::VectorXd::Random(1) * sqrt(R_kf(0,0)); // noise
        x_hat = lqg.update(x_hat, u, y);
        u = lqg.control(x_hat);
        x_true = A * x_true + B * u + Eigen::VectorXd::Random(1) * sqrt(Q_kf(0,0)); // noise

        // Store data for plotting
        x_history.push_back(x_true(0));
        u_history.push_back(u(0));
        y_history.push_back(y(0));
    }

    // Plot results using matplotlib-cpp
    plt::named_plot("True state", x_history);
    plt::named_plot("Control", u_history);
    plt::named_plot("Measurement", y_history);
    plt::legend();
    plt::show();

    return 0;
}
