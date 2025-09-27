#ifndef CMD_VEL_SMOOTHING_HPP
#define CMD_VEL_SMOOTHING_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>


namespace penta_pod::teleop::twist_smoothing
{

class CmdVelSmoothing
{
    using Twist = geometry_msgs::msg::Twist;

public:
    CmdVelSmoothing(rclcpp::Node::SharedPtr node);


private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<Twist>::SharedPtr smoothed_cmd_vel_pub_;
    Twist::SharedPtr target_cmd_vel_;
    Twist::SharedPtr smoothed_cmd_vel_;

    struct node_parameters {
        double max_linear_acceleration;
        double max_angular_acceleration;
        double publishing_rate;
        double input_cmd_vel_timeout_sec;
    } params_;
    double last_cmd_vel_time_;

    double smoothenScalar(double target, double current, double acc, double timer_interval_sec);
    auto smoothenTwist(const Twist& target_cmd_vel, const Twist& current_cmd_vel, double timer_interval_sec) -> Twist::SharedPtr;


    void declare_parameters();
    void load_parameters();
    void cmdVelCallback(const Twist::SharedPtr msg);
};

} // namespace penta_pod::teleop::twist_smoothing

#endif // CMD_VEL_SMOOTHING_HPP