cmake_minimum_required(VERSION 3.16)
project(penta_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(base_twerk_msgs REQUIRED)
find_package(gait_generator_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(commons REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Declare library
add_library(${PROJECT_NAME}
        src/${PROJECT_NAME}/joystick_extra_controls.cpp
        src/${PROJECT_NAME}/declare_load_parameters.cpp
        src/${PROJECT_NAME}/cmd_vel_smoothing.cpp
        src/${PROJECT_NAME}/joystick_base_link_motion.cpp
        src/${PROJECT_NAME}/joystick_turn_head_using_bumper.cpp
        )

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME}
        rclcpp::rclcpp
        sensor_msgs::sensor_msgs
        base_twerk_msgs::base_twerk_msgs
        gait_generator_msgs::gait_generator_msgs
        rclcpp_action::rclcpp_action
        commons::commons
        tf2::tf2
        tf2_geometry_msgs::tf2_geometry_msgs
        )

add_executable(joystick_extra_controls_node src/joystick_extra_controls_node.cpp)
target_link_libraries(joystick_extra_controls_node
        ${PROJECT_NAME}
        )

add_executable(cmd_vel_smoothing_node src/cmd_vel_smoothing_node.cpp)
target_link_libraries(cmd_vel_smoothing_node
        ${PROJECT_NAME}
        )

add_executable(joystick_base_link_motion_node src/joystick_base_link_motion_node.cpp)
target_link_libraries(joystick_base_link_motion_node
        ${PROJECT_NAME}
        )   
# Install
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
  joystick_extra_controls_node
  cmd_vel_smoothing_node
  joystick_base_link_motion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
endif()

ament_package()
