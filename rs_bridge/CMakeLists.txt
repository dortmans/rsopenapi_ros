cmake_minimum_required(VERSION 3.8)
project(rs_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
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
find_package(action_msgs REQUIRED)
find_package(rs_bridge_msgs REQUIRED)

add_subdirectory(rsopenapi)
# ALTERNATIVE: add_library() instead of add_subdirectory(). Needs building of rsopenapi project first.
#set(RSOPENAPI_LIB_LOCATION "${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}/rsopenapi/src/librsopenapi.a")
#set(RTDB_LIB_LOCATION "${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}/rsopenapi/rtdb2/rtdb2/libRtDBrtdb.a")
#add_library(rsopenapi STATIC IMPORTED)
#set_target_properties(rsopenapi PROPERTIES IMPORTED_LOCATION ${RSOPENAPI_LIB_LOCATION})
#add_library(RtDBrtdb STATIC IMPORTED)
#set_target_properties(RtDBrtdb PROPERTIES IMPORTED_LOCATION ${RTDB_LIB_LOCATION})

set(RTDB_CONFIG "rtdb_configuration.xml")
configure_file("rsopenapi/config/${RTDB_CONFIG}" "${CMAKE_BINARY_DIR}/${RTDB_CONFIG}" COPYONLY)
configure_file("rsopenapi/config/${RTDB_CONFIG}" "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/${RTDB_CONFIG}" COPYONLY)

# Define C++ targets
add_executable(rs_bridge_node src/rs_bridge_node.cpp)

target_include_directories(rs_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/rsopenapi/src/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/rsopenapi/rtdb2/comm>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/rsopenapi/rtdb2/rtdb2>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/rsopenapi/rtdb2/definitions/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/rsopenapi/rtdb2/utils/include>
  $<INSTALL_INTERFACE:include>)
  
#target_compile_features(rs_bridge_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(rs_bridge_node
  "rclcpp"
  "rclcpp_action"
  "rclcpp_lifecycle"
  "tf2_ros"
  "builtin_interfaces"
  "std_msgs"
  "geometry_msgs"
  "nav_msgs"
  "action_msgs"
  "rs_bridge_msgs"
)

target_link_libraries(rs_bridge_node
  rsopenapi
  RtDBrtdb # RtDB::rtdb
)

# Install C++ targets
install(TARGETS 
  rs_bridge_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS 
  RtDBcomm
  RtDButils
  DESTINATION lib)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables (and other scripts)
install(PROGRAMS
  #scripts/example_py_node.py
  scripts/install_requirements.sh
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
