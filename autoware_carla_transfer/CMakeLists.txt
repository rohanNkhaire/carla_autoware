cmake_minimum_required(VERSION 3.8)
project(autoware_carla_transfer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

include_directories(include)

add_executable(${PROJECT_NAME} src/autoware_carla_transfer_core.cpp
                               src/autoware_carla_transfer_node.cpp
              )

ament_target_dependencies(${PROJECT_NAME} rclcpp autoware_auto_control_msgs ackermann_msgs)

install(TARGETS
${PROJECT_NAME}
DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/spawn_egovehicle.py
  scripts/dbw_actuation.py
  DESTINATION lib/${PROJECT_NAME}
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
