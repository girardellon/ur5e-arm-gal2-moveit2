cmake_minimum_required(VERSION 3.10)
project(contactile_gripper_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)#used for custom service

#bring in static c++ library headers
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Contactile_Gripper_C++_Lib_LIN_V1.13/include
)
#add the static library
add_library(contactile_gripper_lib STATIC IMPORTED)
set_target_properties(contactile_gripper_lib PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/include/Contactile_Gripper_C++_Lib_LIN_V1.13/lib/libContactileGripper.a
)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${angular_gripper_interface_INCLUDE_DIRS}
)

#add the service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GripperCommand.srv"
  "srv/GripperSerialPort.srv"
  DEPENDENCIES std_msgs
)

# Export the service
ament_export_dependencies(rosidl_default_runtime)

#create the executable
add_executable(contactile_gripper_node 
  src/gripper_node.cpp
)
target_link_libraries(contactile_gripper_node contactile_gripper_lib)

# Link dependencies
ament_target_dependencies(contactile_gripper_node
  rclcpp
  std_msgs
)

# this dependancy seems to cause some build warnings with deprecated python commands but won't be a problem
rosidl_target_interfaces(contactile_gripper_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install targets
install(TARGETS
  contactile_gripper_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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
