cmake_minimum_required(VERSION 3.10)
project(airbase LANGUAGES CXX VERSION 1.0.0)

set(THREADS_PREFER_PTHREAD_FLAG ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE Debug)
set(SLAMTEC_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/thirdparty/slamtec/include)
set(SLAMTEC_LIB_DIR ${CMAKE_SOURCE_DIR}/thirdparty/slamtec/lib)
set(JSON_INCLUDE_DIR /usr/local/include/nlohmann)

set(RPOS_SLAMWARE_NAME rpos_robotplatforms_rpslamware)
set(RPOS_CORE_NAME rpos_framework)

find_package(
  Python
  COMPONENTS Interpreter Development
  REQUIRED)

include_directories(${JSON_INCLUDE_DIR} ${SLAMTEC_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/src)
link_directories(${SLAMTEC_LIB_DIR})

add_subdirectory(thirdparty/argparse)

add_library(airbase SHARED ${CMAKE_SOURCE_DIR}/src/airbase.cpp)
target_link_libraries(airbase
        # static libs
        stdc++
        dl
        z
        rt
        m
        ${RPOS_SLAMWARE_NAME}
        ${RPOS_CORE_NAME}
        boost_atomic
        boost_chrono
        boost_date_time
        boost_regex
        boost_system
        boost_thread
        boost_filesystem
        boost_random
        base64
        jsoncpp
        rlelib
        crypto
        curl
        ssl
        cares
        # dynamic libs
        pthread
        ncurses
        argparse)

add_executable(airbase_demonstrate ${CMAKE_SOURCE_DIR}/examples/mobileAloha.cpp)
target_link_libraries(airbase_demonstrate airbase)

add_executable(keyboardControl ${CMAKE_SOURCE_DIR}/examples/keyboardControl.cpp)
target_link_libraries(keyboardControl airbase)

add_subdirectory(thirdparty/pybind11)
pybind11_add_module(airbase_py src/python_interface.cpp)
target_link_libraries(airbase_py PRIVATE Python::Python airbase)

target_compile_definitions(airbase_py PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})

# Install targets
install(TARGETS airbase airbase_demonstrate
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib/static
)

# Install headers
install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/
        DESTINATION include/airbase
        FILES_MATCHING PATTERN "*.h"
)