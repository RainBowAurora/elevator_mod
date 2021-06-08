cmake_minimum_required(VERSION 3.0.2)

project(topic)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -pthread")

find_package(nlohmann_json 3.2.0 REQUIRED)

if(NOT nlohmann_json_FOUND)
    message(STATUS "No Such json Please install")
endif()

find_package(GTest)
if(GTest_FOUND)
    add_executable(${PROJECT_NAME}_test topic_test.cpp) 
    target_link_libraries(${PROJECT_NAME}_test  ${GTEST_BOTH_LIBRARIES} pthread)
endif()
