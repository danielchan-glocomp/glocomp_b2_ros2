cmake_minimum_required(VERSION 3.8)
project(glocomp_b2_ros2)

include_directories(include include/common)
link_directories(src)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(rosbag2_cpp REQUIRED)


set (
  DEPENDENCY_LIST
  unitree_go
  unitree_hg
  unitree_api
  rclcpp
  std_msgs
  rosbag2_cpp
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
  )

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(b2_tf_node src/b2_tf_node.cpp)
ament_target_dependencies(
    b2_tf_node
    geometry_msgs
    rclcpp
    tf2
    tf2_ros
    nav_msgs
    unitree_go
)

add_executable(b2_odom_node src/b2_odom_node.cpp)
ament_target_dependencies(
    b2_odom_node
    geometry_msgs
    rclcpp
    tf2
    tf2_ros
    nav_msgs
    unitree_go
)

add_executable(clock_pub src/clock_pub.cpp)
ament_target_dependencies(
    clock_pub
    rclcpp
    nav_msgs
    rosgraph_msgs
)

add_executable(imu_repub src/imu_repub.cpp)
ament_target_dependencies(
    imu_repub
    rclcpp
    sensor_msgs
)

add_executable(cmd_vel_subscriber src/cmd_vel_subscriber.cpp src/common/ros2_sport_client.cpp)
target_compile_features(cmd_vel_subscriber PRIVATE cxx_std_20)
ament_target_dependencies(cmd_vel_subscriber ${DEPENDENCY_LIST})

add_executable(front_cam src/front_cam.cpp)
target_compile_features(front_cam PRIVATE cxx_std_20)
target_link_libraries(front_cam ${OpenCV_LIBS})
ament_target_dependencies(front_cam ${DEPENDENCY_LIST})

add_executable(back_cam src/back_cam.cpp)
target_compile_features(back_cam PRIVATE cxx_std_20)
target_link_libraries(back_cam ${OpenCV_LIBS})
ament_target_dependencies(back_cam ${DEPENDENCY_LIST})

install(TARGETS
    front_cam
    back_cam
    cmd_vel_subscriber
    b2_tf_node
    b2_odom_node
    clock_pub
    imu_repub
    DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
