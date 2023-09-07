#ifndef LQG_H
#define LQG_H

#include <Eigen/Dense>

class LQG {
private:
    Eigen::MatrixXd A, B, C, Q, R, Q_kf, R_kf;
    Eigen::MatrixXd L, K;
    void designLQR();
    void designKalmanFilter();

public:
    LQG(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &C, 
        const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R, const Eigen::MatrixXd &Q_kf, const Eigen::MatrixXd &R_kf);
    
    Eigen::VectorXd control(const Eigen::VectorXd &x_hat);
    Eigen::VectorXd update(const Eigen::VectorXd &x_hat, const Eigen::VectorXd &u, const Eigen::VectorXd &y);
};

#endif // LQG_H
