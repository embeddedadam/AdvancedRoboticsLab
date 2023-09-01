#include <gtest/gtest.h>
#include "controller.hpp"

TEST(PIDControllerTest, TestProportional) {
    PIDController pid(1.0, 0.0, 0.0); // Kp=1, Ki=0, Kd=0
    double output = pid.compute(2.0, 1.0); // Setpoint=2, Current value=1
    EXPECT_DOUBLE_EQ(output, 1.0);  // Expected output is 1.0 (simple proportional difference)
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
