cmake_minimum_required(VERSION 3.16)
project(robot_arm_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_library(robot_arm_core
  src/kinematics.cpp
)
target_include_directories(robot_arm_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(robot_arm_core PUBLIC cxx_std_17)
ament_target_dependencies(robot_arm_core rclcpp)

add_executable(arm_node src/arm_node.cpp)
target_link_libraries(arm_node robot_arm_core)
ament_target_dependencies(arm_node rclcpp)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS robot_arm_core
  EXPORT export_robot_arm_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ROS 2 convention: executables live in lib/<package_name>/ so that
# `ros2 run <package> <exe>` can find them.
install(TARGETS arm_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_robot_arm_core HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)

ament_package()
