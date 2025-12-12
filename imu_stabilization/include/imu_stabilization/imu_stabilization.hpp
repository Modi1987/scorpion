#ifndef IMU_STABILIZATION_HPP
#define IMU_STABILIZATION_HPP

#include "rclcpp/rclcpp.hpp" // for rclcpp
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"
#include <Eigen/Geometry>

namespace pentapod::imu::stabilizer {

Eigen::Matrix3d euler2rotation(double roll, double pitch, double yaw);
Eigen::Matrix3d get_skew_symmetric(const Eigen::Vector3d& v);
Eigen::Matrix3d normalize_columns(const Eigen::Matrix3d& R);

using Imu = sensor_msgs::msg::Imu;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Float64 = std_msgs::msg::Float64;

class ImuStabilizer {

public:
    ImuStabilizer(rclcpp::Node::SharedPtr node);
    ~ImuStabilizer();

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<PoseStamped>::SharedPtr base_pose_sub_;
    rclcpp::Publisher<PoseStamped>::SharedPtr base_pose_publisher_;
    rclcpp::Publisher<Float64>::SharedPtr angular_error_pub_;
    std::shared_ptr<Float64> angular_error_msg_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<PoseStamped> current_base_pose_; // current pose feedback
    std::shared_ptr<Imu> imu_feedback_; // imu feedback
    std::shared_ptr<PoseStamped> pose_cmd_; // command stabilization pose
    bool enable_controller_{false};

    struct NodeParams {
        int interval_millis;
        double mounting_rpy[3];
        double kp;
        Eigen::Matrix3d R_mounting;
        int joy_enable_button_index;
        struct tilt_limits {
            double tan_x;
            double tan_y;
            double tan_z;
        } tilt_limits;
    } params_;

    void timer_callback();
    void declare_parameters();
    void get_parameters();
    bool check_tilt_limits(const Eigen::Matrix3d& R_target);
};

} // pentapod::imu::stabilizer

#endif // IMU_STABILIZATION_HPP