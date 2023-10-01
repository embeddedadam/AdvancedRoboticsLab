#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "lqr.hpp"

class LQRTest : public ::testing::Test {
protected:
    Eigen::MatrixXd A, B, Q, R;
    LQR* lqr;

    void SetUp() override {
        A.resize(1, 1);
        B.resize(1, 1);
        Q.resize(1, 1);
        R.resize(1, 1);

        A << 1;
        B << 1;
        Q << 1;
        R << 1;

        lqr = new LQR(A, B, Q, R);
    }

    void TearDown() override {
        delete lqr;
    }
};

TEST_F(LQRTest, DISABLED_ControlOutputTest) {
    Eigen::VectorXd x(1);
    x << 10;
    Eigen::VectorXd u = lqr->control(x);

    // Dummy expected value, replace with real known value if possible
    double expected_value = -10.0; // Given the provided LQR code, assuming K is identity.

    double threshold = 1e-5;
    EXPECT_NEAR(u(0), expected_value, threshold);
}

TEST_F(LQRTest, KMatrixDimensionTest) {
    // Assuming K is a public member in LQR. If it's private, consider a getter or make it protected.
    // The matrix K dimension should match B's column count and A's row count
    EXPECT_EQ(lqr->K.rows(), A.rows());
    EXPECT_EQ(lqr->K.cols(), B.cols());
}

TEST_F(LQRTest, DISABLED_ZeroMatrixTest) {
    // Reset matrices to zero
    A.setZero();
    B.setZero();
    Q.setZero();
    R.setZero();

    LQR zeroLQR(A, B, Q, R);
    Eigen::VectorXd x(1);
    x << 10;
    Eigen::VectorXd u = zeroLQR.control(x);

    // Depending on the behavior defined for zero matrices, 
    // the controller might return zero control or some other default value.
    EXPECT_NEAR(u(0), 0.0, 1e-5);
}

TEST_F(LQRTest, BoundaryValueTest) {
    // Test using extreme large/small values, assuming double type
    Eigen::MatrixXd largeA(1, 1), largeB(1, 1), largeQ(1, 1), largeR(1, 1);
    largeA << 1e10;
    largeB << 1e10;
    largeQ << 1e10;
    largeR << 1e10;

    LQR largeLQR(largeA, largeB, largeQ, largeR);
    Eigen::VectorXd x(1);
    x << 10;
    Eigen::VectorXd u = largeLQR.control(x);

    // This test will be mainly to ensure no overflow errors or similar issues.
    // You might not have an expected value, but you can check if it's finite.
    EXPECT_TRUE(std::isfinite(u(0)));
}

// You can extend tests based on different scenarios, boundary conditions, and expected behaviors.

