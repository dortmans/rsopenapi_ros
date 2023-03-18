cmake_minimum_required(VERSION 3.8)
project(rs_bridge_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/BallPickup.msg"
  "msg/HwStatus.msg"
  "msg/Metadata.msg"
  "msg/Object.msg"
  "msg/PassDetail.msg"
  "msg/PassRequest.msg"
  "msg/PlayerStatus.msg"
  "msg/WorldModel.msg"
)

set(srv_files
  "srv/Empty.srv"
  # etc
)

set(action_files
  "action/Kick.action"
  # etc
)

set(msg_dependencies
  builtin_interfaces
  std_msgs
  geometry_msgs
  nav_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES ${msg_dependencies}
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
