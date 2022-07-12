cmake_minimum_required(VERSION 3.8)
project(motion_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(protocol REQUIRED)
find_package(cyberdog_system REQUIRED)
find_package(cyberdog_debug REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cyberdog_common)

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/motion_utils.cpp)
# target_link_libraries(${PROJECT_NAME} lcm)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  protocol
  cyberdog_system
  cyberdog_common
  cyberdog_debug
  nav_msgs
)

add_executable(distance_test test/distance_test.cpp)
target_link_libraries(distance_test ${PROJECT_NAME})
ament_target_dependencies(distance_test lcm cyberdog_common cyberdog_debug)

add_executable(duration_test test/duration_test.cpp)
target_link_libraries(duration_test ${PROJECT_NAME})
ament_target_dependencies(duration_test lcm cyberdog_common cyberdog_debug)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  ${PROJECT_NAME}
  distance_test
  duration_test
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/motion_teleop.py
  scripts/statistics.py
  scripts/motion_units.py
  scripts/motion_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(cyberdog_common cyberdog_debug)
ament_package()
