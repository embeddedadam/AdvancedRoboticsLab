#ifndef LQR_HPP
#define LQR_HPP

#include <Eigen/Dense>

class LQR {
public:
    LQR(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);
    Eigen::VectorXd control(const Eigen::VectorXd& x);

private:
    Eigen::MatrixXd A, B, Q, R;
    Eigen::VectorXd K;

    void computeK();
};

#endif // LQR_HPP
