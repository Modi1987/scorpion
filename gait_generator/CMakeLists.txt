cmake_minimum_required(VERSION 3.16)
project(gait_generator)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(limb_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gait_generator_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(commons REQUIRED)

# Declare library
add_library(${PROJECT_NAME}
        src/${PROJECT_NAME}/${PROJECT_NAME}_parameters.cpp
        src/${PROJECT_NAME}/${PROJECT_NAME}_core.cpp
        src/${PROJECT_NAME}/base_tf_broadcaster.cpp
        src/${PROJECT_NAME}/gait_utils.cpp
        )

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME}
        rclcpp::rclcpp
        std_msgs::std_msgs
        geometry_msgs::geometry_msgs
        limb_msgs::limb_msgs
        gait_generator_msgs::gait_generator_msgs
        tf2::tf2
        tf2_geometry_msgs::tf2_geometry_msgs
        commons::commons
        )

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
        ${PROJECT_NAME}
        )


add_executable(base_tf_broadcaster_node src/base_tf_broadcaster_node.cpp)
target_link_libraries(base_tf_broadcaster_node
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
        ${PROJECT_NAME}_node
        base_tf_broadcaster_node
        DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

##############
## Testing  ##
##############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
