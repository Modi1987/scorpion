cmake_minimum_required(VERSION 3.12)
project(limb_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Pxyz.msg"
        DEPENDENCIES builtin_interfaces std_msgs
        ADD_LINTER_TESTS
        )

if(BUILD_TESTING)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
