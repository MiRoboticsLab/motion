cmake_minimum_required(VERSION 3.8)
project(elec_skin)

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
find_package(toml REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cyberdog_common REQUIRED)
find_package(cyberdog_embed_protocol REQUIRED)

include_directories(include)

add_executable(${PROJECT_NAME} src/main.cpp)

# MESSAGE('--------------${CMAKE_INSTALL_PREFIX}------------------')

ament_target_dependencies( ${PROJECT_NAME}
  rclcpp
  std_msgs
  toml
  ament_index_cpp
  cyberdog_common
  cyberdog_embed_protocol )

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY toml_config/
  DESTINATION share/${PROJECT_NAME}/toml_config/
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
