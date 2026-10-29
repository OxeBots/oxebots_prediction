cmake_minimum_required(VERSION 3.8)
project(oxebots_prediction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(oxebots_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(
  kalman_filter_node
  src/kalman_filter_node.cpp
  src/ball_kalman_filter.cpp
  src/robot_kalman_filter.cpp
)

ament_target_dependencies(kalman_filter_node rclcpp oxebots_interfaces)
target_link_libraries(kalman_filter_node Eigen3::Eigen)

install(TARGETS
    kalman_filter_node
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kalman_filter
    test/test_kalman_filter.cpp
    src/ball_kalman_filter.cpp
    src/robot_kalman_filter.cpp
  )
  ament_target_dependencies(test_kalman_filter
    rclcpp
    oxebots_interfaces
  )
  target_link_libraries(test_kalman_filter Eigen3::Eigen)

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

ament_package()
