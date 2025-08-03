#include <laser_scan_remapper/laser_scan_remapper.hpp>

namespace laser_scan_remapper
{
LaserScanRemapper::LaserScanRemapper()
{
    node_ = rclcpp::Node::make_shared("laser_scan_remapper");
    
    declareParameters();
    loadParameters();

    laser_scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "gz/scan", rclcpp::QoS(10),
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            this->laserScanCallback(msg);
        });

    laser_scan_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::QoS(10));
}


void LaserScanRemapper::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    auto remapped_msg = *msg;
    remapped_msg.header.frame_id = laser_frame_id_;
    remapped_msg.header.stamp = node_->now();
    laser_scan_pub_->publish(remapped_msg);
}

void LaserScanRemapper::declareParameters()
{
    node_->declare_parameter<std::string>("name_space", "");
    node_->declare_parameter<std::string>("laser_frame_id", "laser");
}

void LaserScanRemapper::loadParameters()
{
    name_space_ = node_->get_parameter("name_space").as_string();
    laser_frame_id_ = node_->get_parameter("laser_frame_id").as_string();  
}

}  // namespace laser_scan_remapper