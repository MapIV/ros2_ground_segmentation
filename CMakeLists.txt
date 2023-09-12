cmake_minimum_required(VERSION 3.14)
project(ros2_composable_template)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package (PCL REQUIRED)

# composable_sample - Composable Nodes are libraries loaded at runtime
ament_auto_add_library(composable_sample SHARED
        src/composable_sample.cpp
        )

target_link_libraries(composable_sample
        ${PCL_LIBRARIES}
        )

target_include_directories(composable_sample PUBLIC
        ${PCL_INCLUDE_DIRS}
        )

rclcpp_components_register_node(composable_sample
        PLUGIN "Ros2ComposableTemplate"
        EXECUTABLE composable_sample_node
        )

ament_auto_package(INSTALL_TO_SHARE
        launch
        )

