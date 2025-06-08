#ifndef COMMONS_QUAT_UTILS_HPP_
#define COMMONS_QUAT_UTILS_HPP_

#include "geometry_msgs/msg/quaternion.hpp"

namespace penta_pod::kin::commons::quaternion_utils {

using geometry_msgs::msg::Quaternion;

Quaternion rpy_to_quaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  Quaternion q;
  q.w = cy * cp * cr + sy * sp * sr;
  q.x = cy * cp * sr - sy * sp * cr;
  q.y = sy * cp * sr + cy * sp * cr;
  q.z = sy * cp * cr - cy * sp * sr;
  return q;
}

Quaternion hamilton_product(Quaternion u, Quaternion v)
{
    Quaternion result;

    result.w = u.w*v.w - u.x*v.x - u.y*v.y - u.z*v.z;
    result.x = u.w*v.x + u.x*v.w + u.y*v.z - u.z*v.y;
    result.y = u.w*v.y - u.x*v.z + u.y*v.w + u.z*v.x;
    result.z = u.w*v.z + u.x*v.y - u.y*v.x + u.z*v.w;

    return result;

}

} // namespace penta_pod::kin::commons::quaternion_utils

#endif // COMMONS_QUAT_UTILS_HPP_