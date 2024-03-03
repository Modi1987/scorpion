#include "test_foot_pos/test_foot_pos.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    penta_pod::kin::test_foot_pos::TestFootPos().spin();
    rclcpp::shutdown();
    return 0;
}