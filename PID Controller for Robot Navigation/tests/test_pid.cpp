#include <random>
#include "gtest/gtest.h"
#include "controller.hpp"

class PIDControllerTest : public ::testing::Test {
protected:
    PIDController pid_controller;

    PIDControllerTest() : pid_controller(1.0, 1.0, 1.0) {}

    void SetUp() override {
        pid_controller.reset();
    }
};

TEST_F(PIDControllerTest, TestProportional) {
    pid_controller.setGains(1.0, 0.0, 0.0);
    double output = pid_controller.compute(4.0, 2.0);
    EXPECT_DOUBLE_EQ(output, 2.0); // Error = 2, so output = kp * 2 = 2
}

TEST_F(PIDControllerTest, TestIntegral) {
    pid_controller.setGains(1.0, 1.0, 1.0);
    pid_controller.compute(1.0, 0.0);
    double output = pid_controller.compute(1.0, 0.0);
    EXPECT_DOUBLE_EQ(output, 3.0); // Error accumulates, so output = kp * 1 + ki * (1 + 1) = 3
}

TEST_F(PIDControllerTest, TestDerivative) {
    pid_controller.setGains(1.0, 1.0, 1.0);
    pid_controller.compute(1.0, 0.0);
    double output = pid_controller.compute(1.0, 0.0);
    EXPECT_DOUBLE_EQ(output, 3.0);  // Derivative is zero since error isn't changing, so it won't affect output
}

TEST_F(PIDControllerTest, TestReset) {
    pid_controller.setGains(1.0, 1.0, 1.0);
    pid_controller.compute(1.0, 0.0);
    pid_controller.reset();
    double output = pid_controller.compute(1.0, 0.0);
    EXPECT_DOUBLE_EQ(output, 3.0);  // After reset, only the proportional component should be there
}

TEST_F(PIDControllerTest, TestSetGains) {
    // Setting up random number generation
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 150.0);  // Adjust range as necessary

    const int num_iterations = 100; // or any other number of iterations you'd like

    for (int i = 0; i < num_iterations; ++i) {
        double kp = dis(gen);
        double ki = dis(gen);
        double kd = dis(gen);

        pid_controller.setGains(kp, ki, kd);
        auto gains = pid_controller.getGains();

        EXPECT_DOUBLE_EQ(std::get<0>(gains), kp);
        EXPECT_DOUBLE_EQ(std::get<1>(gains), ki);
        EXPECT_DOUBLE_EQ(std::get<2>(gains), kd);
    }
}
