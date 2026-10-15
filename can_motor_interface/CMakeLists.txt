cmake_minimum_required(VERSION 3.0.2)
project(can_motor_interface)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  can_msgs
  diagnostic_msgs
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs can_msgs diagnostic_msgs actionlib_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(can_interface_node
  src/can_interface_node.cpp
)

add_dependencies(can_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(can_interface_node
  ${catkin_LIBRARIES}
  pthread
)

install(TARGETS can_interface_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/y42_direct_node.py scripts/check_stm32_can.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_can_protocol test/test_can_protocol.cpp)
  target_include_directories(test_can_protocol PRIVATE include)
  target_link_libraries(test_can_protocol gtest_main)
  add_rostest_gtest(test_can_transport test/can_transport.test
    test/test_can_transport.cpp src/can_interface_node.cpp)
  target_compile_definitions(test_can_transport PRIVATE CAN_MOTOR_INTERFACE_NO_MAIN)
  target_link_libraries(test_can_transport ${catkin_LIBRARIES} pthread
    -Wl,--wrap=socket -Wl,--wrap=ioctl -Wl,--wrap=bind
    -Wl,--wrap=setsockopt -Wl,--wrap=read -Wl,--wrap=write -Wl,--wrap=close)
endif()
