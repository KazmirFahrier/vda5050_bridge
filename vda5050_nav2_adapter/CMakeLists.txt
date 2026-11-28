cmake_minimum_required(VERSION 3.16)
project(vda5050_nav2_adapter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(vda5050_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQUITTO REQUIRED IMPORTED_TARGET libmosquitto)

add_library(${PROJECT_NAME}_lib
  src/adapter_node.cpp
  src/mosquitto_transport.cpp
  src/nav2_navigation_adapter.cpp
  src/topic_action_handler.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}_lib PUBLIC
  vda5050_core::vda5050_core
  PkgConfig::MOSQUITTO
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  rclcpp_action::rclcpp_action
  tf2::tf2
  tf2_ros::tf2_ros
  ${lifecycle_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

add_executable(vda5050_adapter src/main.cpp)
target_link_libraries(vda5050_adapter PRIVATE ${PROJECT_NAME}_lib)

install(TARGETS vda5050_adapter DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # Style and static checks, as listed in package.xml.
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
