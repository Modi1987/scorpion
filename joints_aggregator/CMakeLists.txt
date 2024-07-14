cmake_minimum_required(VERSION 3.12)
project(joints_aggregator)

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
find_package(sensor_msgs REQUIRED)

# # Declare library
add_library(${PROJECT_NAME}
        src/${PROJECT_NAME}/${PROJECT_NAME}.cpp
        )

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)


ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        std_msgs
        geometry_msgs
        limb_msgs
        sensor_msgs
        )


add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
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
        ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

##############.
## Testing  ##
##############

if(BUILD_TESTING)
endif()

ament_package()
