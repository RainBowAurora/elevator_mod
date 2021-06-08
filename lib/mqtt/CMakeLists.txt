cmake_minimum_required(VERSION 3.0.2)

project(mqtt)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -pthread")

find_package(PkgConfig)

if(NOT PkgConfig_FOUND)
    message(STATUS "No Such mosquitto Please install")
endif()

add_library(zr_mqtt SHARED mqtt.cpp)
target_link_libraries(zr_mqtt mosquittopp)

find_package(GTest)
if(GTest_FOUND)
    add_executable(${PROJECT_NAME}_test mqtt_test.cpp) 
    target_link_libraries(${PROJECT_NAME}_test zr_mqtt  ${GTEST_BOTH_LIBRARIES} pthread)
endif()
