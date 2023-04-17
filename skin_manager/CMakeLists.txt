cmake_minimum_required(VERSION 3.8)
project(skin_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(toml REQUIRED)
find_package(protocol REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cyberdog_common REQUIRED)
find_package(elec_skin REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/skin_manager.cpp)


# MESSAGE('--------------${CMAKE_INSTALL_PREFIX}------------------')

ament_target_dependencies( ${PROJECT_NAME}
  rclcpp
  std_msgs
  toml
  protocol
  builtin_interfaces
  ament_index_cpp
  cyberdog_common
  elec_skin )

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
