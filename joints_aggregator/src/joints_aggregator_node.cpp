#include "joints_aggregator/joints_aggregator.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    penta_pod::kin::joints_aggregator::JointsAggregator().spin();
    rclcpp::shutdown();
    return 0;
}