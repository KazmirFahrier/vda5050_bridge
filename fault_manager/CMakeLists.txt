cmake_minimum_required(VERSION 3.16)
project(fault_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(fault_core REQUIRED)
find_package(fault_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)

add_library(${PROJECT_NAME}_lib
  src/fault_manager_node.cpp
  src/snapshot_recorder.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}_lib PUBLIC
  fault_core::fault_core
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  ${fault_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
)

add_executable(fault_manager_node src/main.cpp)
target_link_libraries(fault_manager_node PRIVATE ${PROJECT_NAME}_lib)

install(TARGETS fault_manager_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
