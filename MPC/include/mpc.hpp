#pragma once

#include <Eigen/Dense>
#include <osqp.h>

class MPCController {
public:
    MPCController(int N, double dt, double ref);
    double control(const Eigen::Vector2d& x0);

private:
    Eigen::MatrixXd buildEqualityMatrix();
    Eigen::VectorXd buildReferenceVector();

    int N;  // Horizon
    Eigen::MatrixXd A, B;
    double ref;
    OSQPWorkspace* work;
};
