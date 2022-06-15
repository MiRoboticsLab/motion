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
find_package(cyberdog_common REQUIRED)
find_package(motion_action REQUIRED)
find_package(grid_map REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(dependencies
  rclcpp
  tf2_ros
  lcm
  cyberdog_common
  motion_action
  grid_map
  grid_map_core
  grid_map_ros
  grid_map_msgs)

include_directories(include)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/elevation_bridge.cpp
  src/leg_odom_publisher.cpp
)
target_link_libraries(${PROJECT_NAME} lcm)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(as_controller
  test/as_controller.cpp)
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
  ${PROJECT_NAME} as_controller
  DESTINATION lib/${PROJECT_NAME}
)
ament_export_dependencies(${dependencies})
ament_package()
