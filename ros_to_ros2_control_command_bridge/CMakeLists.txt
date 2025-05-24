cmake_minimum_required(VERSION 3.8)
project(ros_to_ros2_control_command_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Declare library
add_library(${PROJECT_NAME}
        src/${PROJECT_NAME}/forward_joint_command_bridge.cpp
        )

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        std_msgs
        sensor_msgs
        )

add_executable(forward_joint_command_bridge_node src/forward_joint_command_bridge_node.cpp)
target_link_libraries(forward_joint_command_bridge_node
        ${PROJECT_NAME}
        )

#############
# ## Install ##
#############
install(
        DIRECTORY include/
        DESTINATION include
        )

install(TARGETS 
        forward_joint_command_bridge_node
        DESTINATION lib/${PROJECT_NAME}
        )


install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
        )
##############
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
