cmake_minimum_required(VERSION 3.8)
project(rs_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rs_bridge_msgs REQUIRED)

# Define C++ targets
add_executable(rs_bridge_node src/rs_bridge_node.cpp)
target_include_directories(rs_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rs_bridge_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  rs_bridge_node
  "rclcpp"
  "rclcpp_action"
  "rclcpp_lifecycle"
  "rclpy"
  "tf2_ros"
  "builtin_interfaces"
  "std_msgs"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
  "diagnostic_msgs"
  "action_msgs"
  "rs_bridge_msgs"
)

# Install C++ targets
install(TARGETS rs_bridge_node
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables (and other scripts)
install(PROGRAMS
  #scripts/example_py_node.py
  #scripts/example_script.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Install other directories
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
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

ament_package()
