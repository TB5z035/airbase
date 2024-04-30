cmake_minimum_required(VERSION 3.10)
project(airbase)

set(SLAMTEC_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/slamtec/include)
set(SLAMTEC_LIB_DIR ${CMAKE_SOURCE_DIR}/slamtec/lib)
set(JSON_INCLUDE_DIR /usr/local/include/nlohmann)

set(RPOS_SLAMWARE_NAME rpos_robotplatforms_rpslamware)
set(RPOS_CORE_NAME rpos_framework)

include_directories(${JSON_INCLUDE_DIR} ${SLAMTEC_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/src)
link_directories(${SLAMTEC_LIB_DIR})

add_library(airbase SHARED ${CMAKE_SOURCE_DIR}/src/airbase.cpp)
target_link_libraries(airbase 
        stdc++
        dl
        z
        rt
        m
        pthread
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
        ncurses)

add_executable(mobileAloha ${CMAKE_SOURCE_DIR}/examples/mobileAloha.cpp)
target_link_libraries(mobileAloha airbase)

add_executable(keyboardControl ${CMAKE_SOURCE_DIR}/examples/keyboardControl.cpp)
target_link_libraries(keyboardControl airbase)