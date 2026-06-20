cmake_minimum_required(VERSION 3.16)
project(pentapod_imu)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Declare library
add_library(${PROJECT_NAME}
        src/${PROJECT_NAME}/imu_serial_api.cpp
        src/${PROJECT_NAME}/pentapod_imu.cpp
        )

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME}
        rclcpp::rclcpp
        sensor_msgs::sensor_msgs
        )

add_executable(pentapod_imu_node src/pentapod_imu_node.cpp)
target_link_libraries(pentapod_imu_node
        ${PROJECT_NAME}
        )

add_executable(penta_imu_test src/penta_imu_test.cpp)
target_link_libraries(penta_imu_test
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
  pentapod_imu_node
  penta_imu_test
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
