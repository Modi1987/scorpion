cmake_minimum_required(VERSION 3.16)
project(imu_stabilization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# add external dependencies
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
set(NON_ROS_LIBRARIES ${EIGEN3_LIBRARIES} ${PINOCCHIO_LIBRARIES})

add_library(${PROJECT_NAME}
        src/imu_stabilization/imu_stabilization.cpp
        )

target_include_directories(imu_stabilization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(imu_stabilization PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(${PROJECT_NAME}
        rclcpp::rclcpp
        sensor_msgs::sensor_msgs
        geometry_msgs::geometry_msgs
        Eigen3::Eigen
        )

add_executable(imu_stabilization_node src/imu_stabilization_node.cpp)
target_link_libraries(imu_stabilization_node
        ${PROJECT_NAME}
        )

#############
## Install ##
#############

install(
        DIRECTORY include/
        DESTINATION include
)


install(TARGETS 
        ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        )

install(TARGETS 
        imu_stabilization imu_stabilization_node
        DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
