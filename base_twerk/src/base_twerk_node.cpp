#include "base_twerk/base_twerk_cmd_publisher.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    penta_pod::kin::base_twerk_cmd_publisher::BaseTwerkCmdMux().spin();
    rclcpp::shutdown();
    return 0;
}