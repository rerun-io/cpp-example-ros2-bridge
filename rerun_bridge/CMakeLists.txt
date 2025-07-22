cmake_minimum_required(VERSION 3.5)
project(rerun_bridge)

if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Avoid warning about CMP0135
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24.0")
  cmake_policy(SET CMP0135 NEW)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

include(FetchContent)
FetchContent_Declare(rerun_sdk URL https://github.com/rerun-io/rerun/releases/download/0.24.0/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

# setup targets (has to be done before ament_package call)
add_library(${PROJECT_NAME} src/rerun_bridge/rerun_ros_interface.cpp)
add_executable(visualizer src/rerun_bridge/visualizer_node.cpp)

# add system dependencies
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_include_directories(visualizer PUBLIC include ${YAML_CPP_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES} rerun_sdk)
target_link_libraries(visualizer ${PROJECT_NAME} ${YAML_CPP_LIBRARIES} rerun_sdk)

# add ament dependencies
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs nav_msgs geometry_msgs cv_bridge tf2_ros tf2_msgs)
ament_target_dependencies(visualizer rclcpp sensor_msgs nav_msgs geometry_msgs cv_bridge tf2_ros tf2_msgs)

ament_export_dependencies(rclcpp sensor_msgs nav_msgs geometry_msgs cv_bridge tf2_ros tf2_msgs)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(TARGETS visualizer DESTINATION lib/${PROJECT_NAME})
