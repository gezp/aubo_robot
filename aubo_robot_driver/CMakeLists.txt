cmake_minimum_required(VERSION 3.5)
project(aubo_robot_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# ros package
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include sdk/inc)
link_directories(sdk/lib64)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  tf2_ros
)

# lib
add_library(${PROJECT_NAME} SHARED
  src/joint_controller_node.cpp
)
target_link_libraries(${PROJECT_NAME}
  auborobotcontroller
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# joint controller
add_executable(joint_controller
  src/joint_controller_main.cpp
)

target_link_libraries(joint_controller
  ${PROJECT_NAME}
)

# example
add_executable(example
  src/example_main.cpp
)

target_link_libraries(example
  ${PROJECT_NAME}
)

# install library
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(FILES
  sdk/lib64/liblog4cplus.so
  sdk/lib64/liblog4cplus-1.2.so.5
  sdk/lib64/libauborobotcontroller.so
  sdk/lib64/libauborobotcontroller.so.1
  DESTINATION lib
)

# Install executables
install(TARGETS example joint_controller
  DESTINATION lib/${PROJECT_NAME} 
)

# install
install( DIRECTORY
    config launch
  DESTINATION share/${PROJECT_NAME}
)

#test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# specific order: dependents before dependencies
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

ament_package()
