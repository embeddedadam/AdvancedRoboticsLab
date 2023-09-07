#include "lqr.hpp"

LQR::LQR(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) 
    : A(A), B(B), Q(Q), R(R) {
    computeK();
}

Eigen::VectorXd LQR::control(const Eigen::VectorXd& x) {
    return -K * x;
}

void LQR::computeK() {
    Eigen::MatrixXd P = Q;
    for (int i = 0; i < 1000; ++i) {
        P = A.transpose() * P * A - A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A + Q;
    }
    K = (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
}
