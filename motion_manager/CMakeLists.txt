cmake_minimum_required(VERSION 3.8)
project(motion_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(manager_base REQUIRED)
find_package(cyberdog_common REQUIRED)
find_package(cyberdog_system REQUIRED)
find_package(protocol REQUIRED)
find_package(motion_action REQUIRED)
find_package(cyberdog_debug REQUIRED)
find_package(cyberdog_machine REQUIRED)

include_directories(include)

add_executable(${PROJECT_NAME}
  src/motion_handler.cpp
  src/motion_decision.cpp
  src/motion_manager.cpp
  src/main.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  protocol
  manager_base
  cyberdog_system
  cyberdog_common
  motion_action
  protocol
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
