cmake_minimum_required(VERSION 3.8)
project(px4_command)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(mission_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(px4_command "src/px4_command.cpp" "src/main.cpp")

target_include_directories(px4_command
  PUBLIC
  "include/"
)

ament_target_dependencies(px4_command
  rclcpp
  rclcpp_action
  mission_msgs
  px4_msgs
)

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

install(TARGETS
  px4_command
  DESTINATION lib/${PROJECT_NAME})

ament_package()
