cmake_minimum_required(VERSION 3.5)
project(nav2_others_footprint_costmap_plugin)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(lib_name ${PROJECT_NAME}_core)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

set(dep_pkgs
    rclcpp
    nav2_costmap_2d
    pluginlib)


add_library(${lib_name} SHARED
            src/others_footprint_layer.cpp)
include_directories(include)


install(TARGETS ${lib_name}
        DESTINATION lib)


pluginlib_export_plugin_description_file(nav2_costmap_2d others_footprint_layer.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
ament_package()
