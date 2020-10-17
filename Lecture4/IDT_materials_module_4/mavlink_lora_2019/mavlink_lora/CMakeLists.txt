cmake_minimum_required(VERSION 2.8.3)
project(mavlink_lora)

# compile as c++11. Supported in kinetic and forward
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  mavlink_lora_msg.msg
  mavlink_lora_status.msg
  mavlink_lora_pos.msg
  mavlink_lora_attitude.msg
  mavlink_lora_mission_item_int.msg
  mavlink_lora_mission_list.msg
  mavlink_lora_mission_ack.msg
  mavlink_lora_command_ack.msg
  mavlink_lora_command_start_mission.msg
  mavlink_lora_command_set_mode.msg
  mavlink_lora_statustext.msg
  mavlink_lora_heartbeat.msg
  mavlink_lora_set_position_target_local_ned.msg
  mavlink_lora_enable_offboard.msg
  mavlink_lora_command_takeoff.msg
  mavlink_lora_command_land.msg
  mavlink_lora_command_reposition.msg
  mavlink_lora_gps_raw.msg
  mavlink_lora_radio_status.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mavlink_lora_node src/mavlink_lora_node.cpp src/mavlink_lora_lib.c src/serial.c)

# needed because the message file needs to be maked before the node using it
add_dependencies(mavlink_lora_node ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(mavlink_lora_node
  ${catkin_LIBRARIES}
)

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)

    # ROStest for unit and integration test
    find_package(rostest REQUIRED)

    # Add gtest based cpp test target and link libraries. This function actually makes a test-executable
    # REMEMBER to add lib files too to not get undefined reference
    # alternative we can compile as lib and link the lib instead.
    catkin_add_gtest(${PROJECT_NAME}-test
            test/packing_test.cpp
            src/mavlink_lora_lib.c
            src/serial.c)

    # link target as usual
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})



endif() # CATKIN_ENABLE_TESTING
