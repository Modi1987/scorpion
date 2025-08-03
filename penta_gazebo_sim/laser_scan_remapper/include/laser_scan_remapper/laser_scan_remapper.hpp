#ifndef LASER_SCANNER_REMAPPER_HPP
#define LASER_SCANNER_REMAPPER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>

namespace laser_scan_remapper
{
class LaserScanRemapper
{

public:
    explicit LaserScanRemapper();

    void spin() {
        rclcpp::spin(node_);
    };

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;

    std::string name_space_;
    std::string laser_frame_id_;

    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void declareParameters();
    void loadParameters();
};

}  // namespace laser_scan_remapper

#endif  // LASER_SCANNER_REMAPPER_HPP