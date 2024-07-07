#include "test_foot_pos/test_gait.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    penta_pod::kin::test_gait::TestGait().spin();
    rclcpp::shutdown();
    return 0;
}