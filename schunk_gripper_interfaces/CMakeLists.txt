cmake_minimum_required(VERSION 3.22)
project(schunk_gripper_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
    srv/AddGripper.srv
    srv/ListGrippers.srv
    srv/MoveToAbsolutePosition.srv
    srv/Grip.srv
    srv/Release.srv
    srv/ShowConfiguration.srv
    srv/ShowGripperSpecification.srv
    msg/Gripper.msg
    msg/GripperState.msg
    msg/GripperSpec.msg
    msg/KeyValue.msg
  DEPENDENCIES
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
