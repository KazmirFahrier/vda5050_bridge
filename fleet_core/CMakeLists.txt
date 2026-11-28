cmake_minimum_required(VERSION 3.16)
project(fleet_core VERSION 0.1.0 LANGUAGES CXX)

# Fleet traffic control without ROS and without MQTT: roadmap, conflict
# detection, multi-robot planning, deadlock-free execution, and the master
# control side of VDA 5050. Builds under colcon and standalone, like
# vda5050_core.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)
endif()

option(FLEET_CORE_SANITIZE "Build with AddressSanitizer and UBSan" OFF)
if(FLEET_CORE_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

find_package(ament_cmake QUIET)

find_package(vda5050_core QUIET)
if(NOT vda5050_core_FOUND)
  set(fleet_core_saved_build_testing ${BUILD_TESTING})
  set(BUILD_TESTING OFF)
  add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../vda5050_core
    ${CMAKE_CURRENT_BINARY_DIR}/vda5050_core
    EXCLUDE_FROM_ALL)
  set(BUILD_TESTING ${fleet_core_saved_build_testing})
endif()

add_library(${PROJECT_NAME}
  src/conflicts.cpp
  src/fleet_manager.cpp
  src/planner.cpp
  src/reservations.cpp
  src/roadmap.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC vda5050_core::vda5050_core)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(ament_cmake_FOUND)
  install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
  install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endif()

include(CTest)
if(BUILD_TESTING)
  set(TEST_SOURCES
    test/test_roadmap.cpp
    test/test_planner.cpp
    test/test_fleet_sim.cpp
    test/test_fleet_robustness.cpp
  )

  if(ament_cmake_FOUND)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(${PROJECT_NAME}_tests ${TEST_SOURCES})
    target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})
  else()
    add_executable(${PROJECT_NAME}_tests ${TEST_SOURCES})
    target_link_libraries(${PROJECT_NAME}_tests PRIVATE ${PROJECT_NAME})
    find_package(GTest QUIET)
    if(GTest_FOUND)
      target_link_libraries(${PROJECT_NAME}_tests PRIVATE GTest::gtest GTest::gtest_main)
    else()
      message(STATUS "GoogleTest not found - using vda5050_core/test/support/mini_gtest")
      target_sources(${PROJECT_NAME}_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../vda5050_core/test/support/mini_gtest_main.cpp)
      target_include_directories(${PROJECT_NAME}_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../vda5050_core/test/support/shim)
    endif()
    add_test(NAME ${PROJECT_NAME}_tests COMMAND ${PROJECT_NAME}_tests)
  endif()
endif()

if(ament_cmake_FOUND)
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_export_dependencies(vda5050_core)
  ament_package()
endif()
