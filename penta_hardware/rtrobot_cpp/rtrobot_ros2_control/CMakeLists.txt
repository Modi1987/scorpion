cmake_minimum_required(VERSION 3.16)
project(rtrobot_ros2_control LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rtrobot_interfaces
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE HW interface
add_library(
  rtrobot_ros2_control
  SHARED
  hardware/rtrobot_hw_interface.cpp
  hardware/rtrobot_serial_api.cpp
)
target_compile_features(rtrobot_ros2_control PUBLIC cxx_std_17)
target_include_directories(rtrobot_ros2_control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(
  rtrobot_ros2_control PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
install(
  FILES rtrobot_hw_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(hardware_interface rtrobot_hw_interface.xml)

# Add executable for testing
add_executable(test_rtrobot_hw_interface_node hardware/test_rtrobot_serial_api.cpp)
target_link_libraries(test_rtrobot_hw_interface_node
        rtrobot_ros2_control
        )


# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS test_rtrobot_hw_interface_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS rtrobot_ros2_control
  EXPORT export_rtrobot_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY bringup
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_rtrobot_ros2_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
