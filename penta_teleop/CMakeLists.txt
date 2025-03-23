cmake_minimum_required(VERSION 3.8)
project(penta_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(base_twerk_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Declare library
add_library(${PROJECT_NAME}
        src/${PROJECT_NAME}/joystick_extra_controls.cpp
        )

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        base_twerk_msgs
        sensor_msgs
        )

add_executable(joystick_extra_controls_node src/joystick_extra_controls_node.cpp)
target_link_libraries(joystick_extra_controls_node
        ${PROJECT_NAME}
        )

# Install
install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS 
  joystick_extra_controls_node
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
