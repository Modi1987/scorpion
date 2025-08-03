#include <laser_scan_remapper/laser_scan_remapper.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // Create an instance of the LaserScanRemapper node
    auto laser_scan_remapper_node = std::make_shared<laser_scan_remapper::LaserScanRemapper>();

    laser_scan_remapper_node->spin();
    
    rclcpp::shutdown();
    return 0;
}