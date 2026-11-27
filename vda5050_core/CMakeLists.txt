cmake_minimum_required(VERSION 3.16)
project(vda5050_core VERSION 0.1.0 LANGUAGES CXX)

# This package builds two ways from the same file:
#   * inside a ROS 2 workspace with colcon (ament_cmake is found), and
#   * standalone with plain CMake on any machine with a C++17 compiler.
# The core has no ROS dependency by design (see docs/design_decisions.md, ADR-001).

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)
endif()

option(VDA5050_CORE_SANITIZE "Build with AddressSanitizer and UBSan" OFF)
if(VDA5050_CORE_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

find_package(ament_cmake QUIET)

add_library(${PROJECT_NAME}
  src/codec.cpp
  src/header.cpp
  src/json.cpp
  src/order_manager.cpp
  src/order_validator.cpp
  src/session.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# A runnable demonstration with no ROS and no broker: a scripted master
# control, a simulated robot, and the MQTT traffic printed to the terminal.
add_executable(simulated_vehicle examples/simulated_vehicle.cpp)
target_link_libraries(simulated_vehicle PRIVATE ${PROJECT_NAME})

# Writes example messages for validation against the official JSON schemas
# (see test/run_schema_conformance.sh).
add_executable(emit_samples tools/emit_samples.cpp)
target_link_libraries(emit_samples PRIVATE ${PROJECT_NAME})

include(CTest)
if(BUILD_TESTING)
  set(TEST_SOURCES
    test/test_json.cpp
    test/test_codec.cpp
    test/test_header.cpp
    test/test_order_validator.cpp
    test/test_order_manager.cpp
    test/test_session.cpp
  )

  if(ament_cmake_FOUND)
    # ROS 2 workspace: GoogleTest as vendored by ament.
    # Style and static checks (cpplint, uncrustify, xmllint, lint_cmake,
    # flake8, pep257), as listed in package.xml. The GoogleTest stand-in is
    # macro-heavy third-party-style code and is not linted.
    find_package(ament_lint_auto REQUIRED)
    set(AMENT_LINT_AUTO_FILE_EXCLUDE
      ${CMAKE_CURRENT_SOURCE_DIR}/test/support/shim/gtest/gtest.h)
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
      # No GoogleTest on this machine: fall back to the bundled shim, which
      # implements the subset of the GoogleTest API these tests use.
      message(STATUS "GoogleTest not found - using test/support/mini_gtest")
      target_sources(${PROJECT_NAME}_tests PRIVATE test/support/mini_gtest_main.cpp)
      target_include_directories(${PROJECT_NAME}_tests PRIVATE test/support/shim)
    endif()
    add_test(NAME ${PROJECT_NAME}_tests COMMAND ${PROJECT_NAME}_tests)
    # The example exits non-zero if the simulated order does not complete.
    add_test(NAME simulated_vehicle_example COMMAND simulated_vehicle)
  endif()
  target_include_directories(${PROJECT_NAME}_tests PRIVATE test/support)
endif()

if(ament_cmake_FOUND)
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_package()
endif()
