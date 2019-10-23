cmake_minimum_required(VERSION 2.8.3)
project(reach_ros_node)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roslint
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  LLH.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

install(PROGRAMS
   scripts/nmea_tcp_driver
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Check package for pep8 style, add a test to fail on violations.
roslint_python()
roslint_add_test()
