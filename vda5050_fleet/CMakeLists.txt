cmake_minimum_required(VERSION 3.16)
project(vda5050_fleet VERSION 0.1.0 LANGUAGES CXX)

# The fleet manager as a program (fleet_core behind an MQTT connection) and a
# simulated vehicle for trying it out. No ROS at run time. Builds under colcon
# and standalone with plain CMake, like the libraries it uses.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)
endif()

find_package(ament_cmake QUIET)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQUITTO REQUIRED IMPORTED_TARGET libmosquitto)

# In a ROS workspace the libraries are installed packages; standalone, the
# sibling directories are built in place (fleet_core brings vda5050_core).
find_package(fleet_core QUIET)
if(fleet_core_FOUND)
  find_package(vda5050_core REQUIRED)
else()
  set(vda5050_fleet_saved_build_testing ${BUILD_TESTING})
  set(BUILD_TESTING OFF)
  add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../fleet_core
    ${CMAKE_CURRENT_BINARY_DIR}/fleet_core
    EXCLUDE_FROM_ALL)
  set(BUILD_TESTING ${vda5050_fleet_saved_build_testing})
endif()

add_library(${PROJECT_NAME}_mqtt STATIC src/mqtt_client.cpp)
target_include_directories(${PROJECT_NAME}_mqtt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(${PROJECT_NAME}_mqtt PUBLIC
  vda5050_core::vda5050_core PkgConfig::MOSQUITTO Threads::Threads)

add_executable(fleet_manager src/fleet_manager_main.cpp)
target_link_libraries(fleet_manager PRIVATE ${PROJECT_NAME}_mqtt fleet_core::fleet_core)

add_executable(sim_vehicle src/sim_vehicle_main.cpp)
target_link_libraries(sim_vehicle PRIVATE ${PROJECT_NAME}_mqtt)

if(ament_cmake_FOUND)
  install(TARGETS fleet_manager sim_vehicle RUNTIME DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
  install(PROGRAMS test/fleet_system_test.py DESTINATION lib/${PROJECT_NAME})

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()
  ament_package()
endif()
