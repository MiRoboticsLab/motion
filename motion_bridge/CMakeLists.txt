cmake_minimum_required(VERSION 3.8)
project(motion_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lcm REQUIRED)
find_package(protocol REQUIRED)
find_package(cyberdog_common REQUIRED)
find_package(motion_action REQUIRED)
find_package(grid_map REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(dependencies
  rclcpp
  tf2_ros
  lcm
  protocol
  cyberdog_common
  motion_action
  grid_map
  grid_map_core
  grid_map_ros
  grid_map_msgs
  sensor_msgs)

include_directories(include)

add_executable(elevation_bridge
  src/elevation_bridge.cpp
)
target_link_libraries(elevation_bridge lcm)
ament_target_dependencies(elevation_bridge ${dependencies})

add_executable(imu_bridge
  src/imu_bridge.cpp
)
target_link_libraries(imu_bridge lcm)
ament_target_dependencies(imu_bridge ${dependencies})

add_executable(odom_out_publisher
  src/odom_out_publisher.cpp
)
target_link_libraries(odom_out_publisher lcm)
ament_target_dependencies(odom_out_publisher ${dependencies})


add_executable(as_controller
  test/as_controller.cpp
)
target_link_libraries(as_controller lcm)
ament_target_dependencies(as_controller ${dependencies})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)xmllint
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  elevation_bridge
  imu_bridge
  odom_out_publisher
  as_controller
  DESTINATION lib/${PROJECT_NAME}
)
ament_export_dependencies(${dependencies})
ament_package()
