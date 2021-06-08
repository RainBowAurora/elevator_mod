cmake_minimum_required(VERSION 3.0.2)

project(mode)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -pthread")

###########################elevator_control###################################
add_library(elevator_control SHARED elevator_control/elevator_control.cpp)
target_link_libraries(elevator_control zr_mqtt)

add_executable(elevator_control_test elevator_control/elevator_control_test.cpp) 
target_link_libraries(elevator_control_test elevator_control)


###############################module_argument#################################
add_library(module_argument SHARED module_argument/module_argument.cpp)
target_link_libraries(module_argument zr_mqtt)

add_executable(module_argument_test module_argument/module_argument_test.cpp) 
target_link_libraries(module_argument_test module_argument)