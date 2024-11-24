#include "base_twerk/base_twerk_cmd_mux.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    penta_pod::kin::base_twerk_cmd_mux::BaseTwerkCmdMux().spin();
    rclcpp::shutdown();
    return 0;
}