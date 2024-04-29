cmake_minimum_required(VERSION 3.10)
project(airbase)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -pthread")
set(SLAMTEC_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/slamtec/include)
set(SLAMTEC_LIB_DIR ${CMAKE_SOURCE_DIR}/slamtec/lib)
set(JSON_INCLUDE_DIR /usr/local/include/nlohmann)

set(LD_LIBS
    -Xlinker "-("
    -lstdc++
    -ldl
    -lz
    -lrt
    -lm
    -lpthread
    -l${RPOS_SLAMWARE_NAME}
    -l${RPOS_CORE_NAME}
    -lboost_atomic
    -lboost_chrono
    -lboost_date_time
    -lboost_regex
    -lboost_system
    -lboost_thread
    -lboost_filesystem
    -lboost_random
    -lbase64
    -ljsoncpp
    -lrlelib
    -lcrypto
    -lcurl
    -lssl
    -lcares
    -lncurses
    -Xlinker "-)"
)

include_directories(${JSON_INCLUDE_DIR} ${SLAMTEC_INCLUDE_DIR})
link_directories(${SLAMTEC_LIB_DIR})

add_library(airbase STATIC src/main.cpp)
target_link_libraries(airbase PRIVATE ${LD_LIBS})
