#include "gait_generator/gait_generator.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    penta_pod::kin::gait_generator::GaitGenerator().spin();
    rclcpp::shutdown();
    return 0;
}