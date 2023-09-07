#include "lqg.hpp"

LQG::LQG(const Eigen::MatrixXd &A_, const Eigen::MatrixXd &B_, const Eigen::MatrixXd &C_, 
         const Eigen::MatrixXd &Q_, const Eigen::MatrixXd &R_, const Eigen::MatrixXd &Q_kf_, const Eigen::MatrixXd &R_kf_)
    : A(A_), B(B_), C(C_), Q(Q_), R(R_), Q_kf(Q_kf_), R_kf(R_kf_) {
    designLQR();
    designKalmanFilter();
}

void LQG::designLQR() {
    // Simplified Riccati equation for a 1D system
    K = (R + B.transpose() * Q * B).inverse() * B.transpose() * Q * A;
}

void LQG::designKalmanFilter() {
    // Simplified Riccati equation for a 1D system
    L = (R_kf + C.transpose() * Q_kf * C).inverse() * C.transpose() * Q_kf * A;
}

Eigen::VectorXd LQG::control(const Eigen::VectorXd &x_hat) {
    return -K * x_hat;
}

Eigen::VectorXd LQG::update(const Eigen::VectorXd &x_hat, const Eigen::VectorXd &u, const Eigen::VectorXd &y) {
    return A * x_hat + B * u + L * (y - C * x_hat);
}
