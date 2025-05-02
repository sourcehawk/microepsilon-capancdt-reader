cmake_minimum_required(VERSION 3.10)
project(capancdt_proximity_sensor_lib LANGUAGES CXX)

option(ENABLE_COVERAGE "Enable coverage reporting" OFF)
if(ENABLE_COVERAGE)
  message(STATUS "Building with coverage instrumentation")
  add_compile_options(-fprofile-arcs -ftest-coverage -O0 -g)
  add_link_options(-fprofile-arcs -ftest-coverage)
endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
include(CMakePackageConfigHelpers)
set(config_package_location lib/cmake/capancdt_proximity_sensor)

find_package(Threads REQUIRED)

add_library(capancdt_proximity_sensor_lib STATIC
  src/proximity_sensor_message.cpp
  src/proximity_connection.cpp
  src/proximity_sensor.cpp
)

target_include_directories(capancdt_proximity_sensor_lib
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(capancdt_proximity_sensor_lib
  PUBLIC Threads::Threads
)

add_executable(capancdt_read_sensor
  src/read_sensor.cpp
)

target_link_libraries(capancdt_read_sensor
  PRIVATE
    capancdt_proximity_sensor_lib
    Threads::Threads
)

target_link_options(capancdt_read_sensor PRIVATE
  "-static-libstdc++"
  "-static-libgcc"
  "-pthread"
)

install(TARGETS capancdt_read_sensor
  RUNTIME DESTINATION bin
)

set_target_properties(capancdt_proximity_sensor_lib PROPERTIES
  VERSION      1.0.0
  SOVERSION    1
)

install(TARGETS capancdt_proximity_sensor_lib
  EXPORT capancdt_proximity_sensorTargets
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/capancdt_proximity_sensorConfigVersion.cmake"
  VERSION 1.0.0
  COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/capancdt_proximity_sensorConfig.cmake"
  INSTALL_DESTINATION ${config_package_location}
)

install(EXPORT capancdt_proximity_sensorTargets
  FILE capancdt_proximity_sensorTargets.cmake
  NAMESPACE capancdt_proximity_sensor::
  DESTINATION ${config_package_location}
)

install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/capancdt_proximity_sensorConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/capancdt_proximity_sensorConfigVersion.cmake"
  DESTINATION ${config_package_location}
)

option(BUILD_TESTING "Enable unit tests" ON)
if(BUILD_TESTING)
  enable_testing()
  find_package(GTest REQUIRED)
  include(GoogleTest)
  
  # discover all test_*.cpp in test/
  file(GLOB TEST_SOURCES test/test_*.cpp)
  foreach(test_src ${TEST_SOURCES})
    get_filename_component(test_name ${test_src} NAME_WE)
    add_executable(${test_name} ${test_src})
    target_link_libraries(${test_name}
      PRIVATE
        capancdt_proximity_sensor_lib
        GTest::gtest_main
        Threads::Threads
    )
    gtest_discover_tests(${test_name})
  endforeach()
endif()
