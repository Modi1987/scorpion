cmake_minimum_required(VERSION 3.12)
project(scorpion_cmd_vel_mux)

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

#############
## Install ##
#############

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

##############.
## Testing  ##
##############

if(BUILD_TESTING)
endif()

ament_package()
