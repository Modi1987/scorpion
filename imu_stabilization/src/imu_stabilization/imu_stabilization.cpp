#include "imu_stabilization/imu_stabilization.hpp"

namespace pentapod::imu::stabilizer {

Eigen::Matrix3d euler2rotation(double roll, double pitch, double yaw) {
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    Eigen::Matrix3d R = q.toRotationMatrix();
    return R;
}

Eigen::Matrix3d get_skew_symmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S = Eigen::Matrix3d::Zero();
    S(0, 1) = -v(2); S(0, 2) =  v(1);
    S(1, 0) =  v(2); S(1, 2) = -v(0);
    S(2, 0) = -v(1); S(2, 1) =  v(0);
    return S;
}

Eigen::Matrix3d normalize_columns(const Eigen::Matrix3d& R) {
    Eigen::Matrix3d R_normalized;
    for (int i = 0; i < 3; i++) {
        R_normalized.col(i) = R.col(i).normalized();
    }
    return R_normalized;
};

ImuStabilizer::ImuStabilizer(rclcpp::Node::SharedPtr node)  : node_(node) {
    // parameters
    declare_parameters();
    get_parameters();
    // sub, pub and timers
    base_pose_publisher_ = node_->create_publisher<PoseStamped>(
        "base_orientation_pub",
        10
    );

    imu_sub_ = node_->create_subscription<Imu>(
        "imu_sub",
        10,
        [this](const Imu::SharedPtr msg) {
            if (!imu_feedback_) {
                imu_feedback_ = std::make_shared<Imu>();
            }
            *imu_feedback_ = *msg;
        });

    base_pose_sub_ = node_->create_subscription<PoseStamped>(
        "base_orientation_sub",
        10,
        [this](const PoseStamped::SharedPtr msg) {
            if (!current_base_pose_) {
                current_base_pose_ = std::make_shared<PoseStamped>();
            }
            *current_base_pose_ = *msg;
        });

    timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(params_.interval_millis)),
      [this]() { timer_callback(); });
}

ImuStabilizer::~ImuStabilizer() {}

void ImuStabilizer::timer_callback() {

    if (!imu_feedback_ || !current_base_pose_) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), 
            *node_->get_clock(), 
            5000, 
            "IMU or Base Pose feedback not ready yet."
        );
        return;
    }

    std::shared_ptr<PoseStamped> current_base_pose_; // current pose feedback
    auto quat = Eigen::Quaterniond(
        imu_feedback_->orientation.w,
        imu_feedback_->orientation.x,
        imu_feedback_->orientation.y,
        imu_feedback_->orientation.z
    );
    Eigen::Matrix3d R = quat.toRotationMatrix();
    auto R_mounting = euler2rotation(
        params_.mounting_rpy[0],
        params_.mounting_rpy[1],
        params_.mounting_rpy[2]
    );
    R = R * R_mounting.transpose(); // compensate mounting orientation
    Eigen::Vector3d z_axis(R(2,0), R(2,1), R(2,2));
    Eigen::Vector3d vertical(0.0, 0.0, 1.0);
    auto error = vertical.cross(z_axis);
    auto w_stabilization = -params_.kp * error;
    Eigen::Quaterniond q_current(R);
    Eigen::Vector3d delta_angle = w_stabilization * params_.interval_millis / 1000.0;
    double angle = delta_angle.norm();
    Eigen::Quaterniond q_target;
    if (angle < 1e-6) {
        q_target = q_current;
    } else {
        Eigen::Quaterniond q_delta(Eigen::AngleAxisd(angle, delta_angle.normalized()));
        q_target = q_delta * q_current;
    }
    q_target.normalize();
    Eigen::Matrix3d R_target = q_target.toRotationMatrix();
    pose_cmd_->pose.position = current_base_pose_->pose.position; // command stabilization pose
    Eigen::Quaterniond target_q(R_target);
    pose_cmd_->pose.orientation.x = target_q.x();
    pose_cmd_->pose.orientation.y = target_q.y();
    pose_cmd_->pose.orientation.z = target_q.z();
    pose_cmd_->pose.orientation.w = target_q.w();
    base_pose_publisher_->publish(*pose_cmd_);
}

void ImuStabilizer::declare_parameters() {
    node_->declare_parameter<int>("interval_millis", 20);
    node_->declare_parameter<double>("mounting.roll", 0.0);
    node_->declare_parameter<double>("mounting.pitch", 0.0);
    node_->declare_parameter<double>("mounting.yaw", 0.0);
}

void ImuStabilizer::get_parameters() {
    int default_millis = 20;
    if (!node_->get_parameter("interval_millis", params_.interval_millis)) {
        params_.interval_millis = default_millis;
        RCLCPP_WARN(node_->get_logger(), "Could not load parameter interval_millis, defaulting to: %d", params_.interval_millis);
    } else {
        RCLCPP_WARN(node_->get_logger(), "Loaded parameter value for interval_millis is: %d", params_.interval_millis);
    }

    double default_kp = 0.1;
    if (!node_->get_parameter("kp", params_.kp)) {
        params_.kp = default_kp;
        RCLCPP_WARN(node_->get_logger(), "Could not load parameter kp, defaulting to: %f", params_.kp);
    } else {
        RCLCPP_WARN(node_->get_logger(), "Loaded parameter value for kp is: %f", params_.kp);
    }

    std::vector<std::string> keys = {"mounting.roll", "mounting.pitch", "mounting.yaw"};
    for (int i = 0; i < 3; i++) {
        double value = 0.0; // degrees
        if (!node_->get_parameter(keys[i], value)) {
            params_.mounting_rpy[i] = value * M_PI / 180.0; // convert to radian
            RCLCPP_WARN(node_->get_logger(), "Could not load parameter %s, defaulting to: %f", keys[i].c_str(), value);
        } else {
            params_.mounting_rpy[i] = value * M_PI / 180.0; // convert to radian
            RCLCPP_WARN(node_->get_logger(), "Loaded parameter value for %s is: %f", keys[i].c_str(), params_.mounting_rpy[i]);
        }
    }
}

} // namespace pentapod::imu::stabilizer
