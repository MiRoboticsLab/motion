cmake_minimum_required(VERSION 3.8)
project(motion_action)

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
find_package(cyberdog_common)
find_package(lcm REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/motion_action.cpp)
target_link_libraries(${PROJECT_NAME} lcm)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  protocol
  cyberdog_system
)

add_executable(test_as_manager test/test_as_manager.cpp)
target_link_libraries(test_as_manager ${PROJECT_NAME} lcm)
ament_target_dependencies(test_as_manager lcm cyberdog_common)

add_executable(test_as_controller test/test_as_controller.cpp)
target_link_libraries(test_as_controller ${PROJECT_NAME} lcm)
ament_target_dependencies(test_as_controller lcm cyberdog_common)

add_executable(test_as_publisher test/test_as_publisher.cpp)
target_link_libraries(test_as_publisher ${PROJECT_NAME} lcm)
ament_target_dependencies(test_as_publisher lcm cyberdog_common)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY preset
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS
  ${PROJECT_NAME} test_as_manager test_as_controller test_as_publisher
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/motion_teleop.py
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
ament_export_dependencies(lcm)
ament_package()
