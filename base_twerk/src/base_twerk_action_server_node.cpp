#include "base_twerk/base_twerk_action_server.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    penta_pod::kin::base_twerk::BaseTwerkActionServer().spin();
    rclcpp::shutdown();
    return 0;
}