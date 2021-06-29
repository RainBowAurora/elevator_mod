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


###############################robot_control####################################
aux_source_directory(robot_control ROBOT_CONTROL_DIR)
message(STATUS "robot_control = ${ROBOT_CONTROL_DIR}")
add_library(robot_control_mod ${ROBOT_CONTROL_DIR})
target_link_libraries(robot_control_mod ${catkin_LIBRARIES})

add_executable(robot_control_test robot_control/robot_control_test.cpp)
target_link_libraries(robot_control_test robot_control_mod)


###############################process_planning####################################
add_library(process_planning SHARED process_planning.cpp)
target_link_libraries(process_planning robot_control_mod elevator_control)


###################################node###########################################
add_executable(elevator_node node.cpp)
target_link_libraries(elevator_node process_planning)
