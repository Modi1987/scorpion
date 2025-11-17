#ifndef IMU_STABILIZATION_HPP
#define IMU_STABILIZATION_HPP

#include "rclcpp/rclcpp.hpp" // for rclcpp
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace pentapod:imu:stabilizer {

using Imu = sensor_msgs::msgs::Imu;
using PoseStamped = geometry_msgs::msg::PoseStamped;

class ImuStabilizer {

public:
    ImuStabilizer(rclcpp::Node::SharedPtr node);
    ~ImuStabilizer();

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<PoseStamped>::SharedPtr base_pose_sub_;
    rclcpp::Publisher<PoseStamped>::SharedPtr base_pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<PoseStamped> current_base_pose_; // current pose feedback
    std::shared_ptr<Imu> imu_feedback_; // imu feedback
    stdd::shared_ptr<PoseStamped> pose_cmd_; // command stabilization pose

    struct NodeParams {
        int interval_millis;
        double mounting_rpy[3];
        double kp;
    } params_;

    void timer_callback();
    void declare_parameters();
    void get_parameters();
}

} // pentapod:imu:stabilizer

#endif

ImuStabilizer::ImuStabilizer(rclcpp::Node::SharedPtr node)  : node_(node) {
    // parameters
    declare_parameters();
    get_parameters();
    // sub, pub and timers
    base_pose_publisher_ = node_->create_publisher<PoseStamped>(
        "base_orientation_pub",
        1
    );

    imu_sub_ = node_->create_subscription<Imu>(
        "imu_sub",
        1,
        [this](const Imu::SharedPtr &msg) {
            imu_feedback_ = msg;
        });

    base_pose_sub_ = node_->create_subscription<PoseStamped>(
        "base_orientation_sub",
        1,
        [this](const PoseStamped::SharedPtr &msg) {
            current_base_pose_ = msg;
        });

    timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(interval_millis)),
      [this]() { timer_callback(); });
}

void ImuStabilizer::timer_callback() {

    std::shared_ptr<PoseStamped> current_base_pose_; // current pose feedback
    auto quat = imu_feedback_->orientation; // imu feedback
    auto R = imu_quat_to_rotation_matrix(quat);
    auto z_axis = std::vector<double>{R[2][0], R[2][1], R[2][2]};
    auto vertical = std::vector<double>{0.0, 0.0, 1.0};
    auto error = cross_product(vertical, z_axis);
    auto w_stabilization = - k * error;
    auto skew = get_skew_symmetric(w_stabilization);
    auto interval_second =  params_.interval_millis / 1000.0;
    auto R_target = R + skew * R *  interval_second;
    R_target = normalize_columns(R_target);
    pose_cmd_->position = current_base_pose_->position; // command stabilization pose
    pose_cmd_->orientation = rotation_matrix_to_quat(R_target);
}

void ImuStabilizer::declare_parameters() {
    node_->declare_parameter<int>("interval_millis", 20);
    node_->declare_parameter<double>("mounting.roll", 0.0);
    node_->declare_parameter<double>("mounting.pitch", 0.0);
    node_->declare_parameter<double>("mounting.yaw", 0.0);
}

void ImuStabilizer::get_parameters() {
    int default_millis = 20;
    if (!node_get_parameter("interval_millis", params_.interval_millis)) {
        params_.interval_millis = default_millis;
        RCLCPP_WARN(node_->get_logger(), "Could not load parameter interval_millis, defaulting to: %d", defualt_millis);
    } else {
        RCLCPP_WARN(node_->get_logger(), "Loaded parameter value for interval_millis is: %d", params_.interval_millis);
    }

    double default_kp = 0.1;
    if (!node_get_parameter("kp", params_.kp)) {
        params_.kp = default_kp;
        RCLCPP_WARN(node_->get_logger(), "Could not load parameter kp, defaulting to: %d", defualt_kp);
    } else {
        RCLCPP_WARN(node_->get_logger(), "Loaded parameter value for kp is: %d", params_.kp;
    }

    std::vector<std::string> keys = {"mounting.roll", "mounting.pitch", "mounting.yaw"};
    for (int i = 0; i < 3; i++) {
        double default_val = 0.0;
        if (!node_get_parameter(keys[i], params_.mounting_rpy[i])) {
            params_.mounting_rpy[i] = default_val;
            RCLCPP_WARN(node_->get_logger(), "Could not load parameter %s, defaulting to: %f", keys[i], default_val);
        } else {
            RCLCPP_WARN(node_->get_logger(), "Loaded parameter value for %s is: %f", keys[i], params_.mounting_rpy[i]);
        }
    }

}

