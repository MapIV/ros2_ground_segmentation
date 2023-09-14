cmake_minimum_required(VERSION 3.14)
project(ros2_ground_segmentation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package (PCL REQUIRED)

# line_fit_ground_segmentation - Composable Nodes are libraries loaded at runtime
ament_auto_add_library(linefit_ground_segmentation SHARED
        src/line_fit_ground_segmentation.cpp
        src/linefit/linefit_seg.cpp
        src/linefit/bin.cpp
        src/linefit/segment.cpp
        )

target_link_libraries(linefit_ground_segmentation
        ${PCL_LIBRARIES}
        )

target_include_directories(linefit_ground_segmentation PUBLIC
        include
        ${PCL_INCLUDE_DIRS}
        )

rclcpp_components_register_node(linefit_ground_segmentation
        PLUGIN "LinefitGroundSegmentation"
        EXECUTABLE linefit_ground_segmentation_node
        )

ament_auto_package(INSTALL_TO_SHARE
        launch
        )

