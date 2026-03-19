cmake_minimum_required(VERSION 3.8)
project(gtsam_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(GTSAM REQUIRED)

add_executable(pose_estimator_node src/pose_estimator_node.cpp)

target_include_directories(pose_estimator_node PRIVATE
  ${GTSAM_INCLUDE_DIR}
)

target_link_libraries(pose_estimator_node
  gtsam
)

ament_target_dependencies(pose_estimator_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  pose_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
