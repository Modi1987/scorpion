#ifndef COMMONS_ROS2_UTILS_HPP_
#define COMMONS_ROS2_UTILS_HPP_

#include "rclcpp/rclcpp.hpp"

namespace penta_pod::kin::commons {

template <typename T>
auto service_call_template(
    const rclcpp::Node::SharedPtr &node,
    const typename rclcpp::Client<T>::SharedPtr &service_client,
    const typename T::Request::SharedPtr &msg,
    const std::chrono::milliseconds timeout_millis) -> bool {
  using namespace std::chrono_literals;
  auto service_name = service_client->get_service_name();
  if (!service_client->wait_for_service(1s)) {
    RCLCPP_ERROR(node->get_logger(), "Service %s not online!", service_name);
    return false;
  }
  auto future = service_client->async_send_request(msg);
  auto status = future.wait_for(timeout_millis);
  if (status == std::future_status::ready) {
    auto result = future.get();
    RCLCPP_INFO(node->get_logger(), "Service %s response is ready!",
                service_name);
    return result->success;
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service %s response timed out!",
                 service_name);
    return false;
  }
}

inline bool is_intra_process_arg(int argc, char **argv) {
  // parse interaprocess arg
  bool use_intra = false;
  for (int i = 1; i < argc - 1; i++) {
    if (std::string(argv[i]).find("--intra-process") != std::string::npos) {
      if (!(i + 1 < argc))
        break;
      std::string val = std::string(argv[i + 1]);
      if (val.find("true") != std::string::npos) {
        use_intra = true;
      }
    }
  }
  return use_intra;
}

} // namespace penta_pod::kin::commons

#endif // COMMONS_ROS2_UTILS_HPP_
