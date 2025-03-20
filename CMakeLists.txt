# PROJECT SETUP
cmake_minimum_required(VERSION 3.8)
project(luggage_av)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# DEPENDENCIES
set(DEPENDENCIES
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  Protobuf
  geometry_msgs   # Required for geometry_msgs/PoseStamped
  behaviortree_cpp_v3
  nav2_behavior_tree
  service_msgs
   tf2
   yaml-cpp
  # luggage_av_msgs
   nav2_msgs
   rclcpp_action
)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

# INCLUDE DIRECTORIES
include_directories(
  include
  ${Protobuf_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
   ${nav2_msgs_INCLUDE_DIRS}
   ${rclcpp_action_INCLUDE_DIRS}
)

# CODE GENERATION
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROJECT_SOURCE_DIR}/hardware_interface/proto/wheel_commands.proto ${PROJECT_SOURCE_DIR}/hardware_interface/proto/wheel_states.proto)

# LIBRARY/EXECUTABLE CONFIGURATION
add_library(  #define luggage_av as a SHARED library
  luggage_av SHARED
  hardware_interface/src/luggage_av_hardware_interface.cpp
  hardware_interface/src/cobs.c
  behavior_trees/nodes/wait_for_route_node.cpp
   behavior_trees/nodes/tbn.cpp
   behavior_trees/nodes/navigate_to_location_node.cpp
   behavior_trees/nodes/navigate_test_node.cpp
  ${PROTO_SRCS}
)
ament_target_dependencies(
  luggage_av PUBLIC
  ${DEPENDENCIES}
)

target_compile_features(luggage_av PUBLIC cxx_std_17)

target_include_directories(luggage_av PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware_interface/include>
  $<INSTALL_INTERFACE:include/luggage_av>
  ${Protobuf_INCLUDE_DIRS}
)

pluginlib_export_plugin_description_file(hardware_interface luggage_av.xml)

ament_export_dependencies(rosidl_default_runtime)

# EXECUTABLE SETUP
add_executable(behavior_tree_node behavior_trees/behavior_tree_execution.cpp)
ament_target_dependencies(behavior_tree_node rclcpp behaviortree_cpp_v3 geometry_msgs nav2_msgs rclcpp_action)

add_executable(navigate_test_node behavior_trees/nodes/navigate_test_node.cpp) #zzTemp remove this
ament_target_dependencies(navigate_test_node rclcpp geometry_msgs nav2_msgs rclcpp_action)

# add_executable(simple_node behavior_trees/nodes/simple_node.cpp) #zzTemp remove this
# ament_target_dependencies(simple_node rclcpp geometry_msgs nav2_msgs rclcpp_action)

# INSTALL
install(
  DIRECTORY hardware_interface/include
  DESTINATION include/luggage_av_hardware_interface
)

install(
  DIRECTORY configs launch maps parameters urdf worlds msg behavior_trees behavior_trees/nodes
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS luggage_av
  EXPORT export_luggage_av
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS behavior_tree_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS navigate_test_node
  DESTINATION lib/${PROJECT_NAME})
# install(TARGETS simple_node
#   DESTINATION lib/${PROJECT_NAME})

# TESTS
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(hardware_interface)
  find_package(ament_cmake_gmock REQUIRED)
# the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# EXPORT
ament_export_targets(export_luggage_av HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
