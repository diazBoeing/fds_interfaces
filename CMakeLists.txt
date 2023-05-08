cmake_minimum_required(VERSION 3.0.2)
project(fds_interfaces)

set(CMAKE_VERBOSE_MAKEFILE ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  genmsg
  geometry_msgs
  message_generation
  message_runtime
  sensor_msgs
  std_msgs
  trajectory_msgs
  visualization_msgs
  kdl_conversions
  pcl_ros

  a5_sanding_process_planning

  mc_utils
)

find_package(yaml-cpp REQUIRED)
# find_package(rapidxml REQUIRED)

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#    ProcessAction.msg
#)

add_service_files(
  FILES
    GetJointPositions.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
    ExecuteCommand.action
    FDS_SM_action.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    sensor_msgs
    trajectory_msgs
    visualization_msgs
    std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    genmsg
    geometry_msgs
    message_runtime
    sensor_msgs
    trajectory_msgs
    visualization_msgs
    std_msgs
    kdl_conversions
    pcl_ros
    visualization_msgs
    a5_sanding_process_planning
    mc_utils
  DEPENDS
    YAML_CPP
)

###########
## Build ##
###########

include_directories(
  include
  /home/developer/cat_ws/src/fds_interfaces/include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

set(SUBD1_SOURCES
    "src/aircraft_data_file.cpp"
    "src/base_position_file.cpp"
    "src/ik_solution_file.cpp"
    "src/orientation_file.cpp"
    "src/target_file.cpp"
    "src/waypoint_data_file.cpp"
    "src/CsvParser.c"
)

## library for common classes
add_library(
    ${PROJECT_NAME}
    ${SUBD1_SOURCES}
)

target_link_libraries(
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    yaml-cpp
)

add_dependencies(
    ${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

## test executable node
add_executable(test_ik_solution_file src/test/test_ik_solution_file.cpp)
target_link_libraries(test_ik_solution_file ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(test_ik_solution_file ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(test_base_position_file src/test/test_base_position_file.cpp)
target_link_libraries(test_base_position_file ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(test_base_position_file ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(test_aircraft_data_file src/test/test_aircraft_data_file.cpp)
target_link_libraries(test_aircraft_data_file ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(test_aircraft_data_file ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(test_waypoint_file src/test/test_waypoint_file.cpp)
target_link_libraries(test_waypoint_file ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(test_waypoint_file ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(test_orientation_file src/test/test_orientation_file.cpp)
target_link_libraries(test_orientation_file ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(test_orientation_file ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(publish_ac_data src/test/test_rviz_display.cpp)
target_link_libraries(publish_ac_data ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(publish_ac_data ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(print_bp_data src/test/print_base_position_data.cpp)
target_link_libraries(print_bp_data ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(test_target_file src/test/test_target_file.cpp)
target_link_libraries(test_target_file ${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

install(
  DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  TARGETS
     ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# cp $WORKSPACE_ROOT/src/fds_interfaces/include/fds_state_machine_commands.h $WORKSPACE_ROOT/devel/.private/fds_interfaces/include/fds_interfaces


# message(STATUS "include dir: ${CATKIN_GLOBAL_INCLUDE_DESTINATION}")

# file(MAKE_DIRECTORY ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${PROJECT_NAME}/)
# file(MAKE_DIRECTORY ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}/)
# add_custom_command(
#     TARGET ${PROJECT_NAME} POST_BUILD
#     COMMAND ${CMAKE_COMMAND} -E cp include/${PROJECT_NAME}/*.h ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )
