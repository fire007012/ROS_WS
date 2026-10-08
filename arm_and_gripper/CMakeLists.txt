cmake_minimum_required(VERSION 3.0.2)
project(arm_and_gripper)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

# ── 服务定义 ──
add_service_files(
  FILES
  ServoCommand.srv
  PlaceMedicine.srv
  ArmPlaceMedicine.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ── 舵机控制节点 ──
add_executable(servo_controller_node
  src/servo_controller_node.cpp
)
add_dependencies(servo_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(servo_controller_node
  ${catkin_LIBRARIES}
  pigpio
  pthread
)

# ── 机械臂+夹爪主控制节点 ──
add_executable(arm_and_gripper_node
  src/arm_and_gripper_node.cpp
)
add_dependencies(arm_and_gripper_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_and_gripper_node
  ${catkin_LIBRARIES}
  pthread
)

# ── 安装 ──
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_arm_cancellation test/arm_cancellation.test
    test/test_arm_cancellation.cpp src/arm_and_gripper_node.cpp)
  if(TARGET test_arm_cancellation)
    target_compile_definitions(test_arm_cancellation PRIVATE ARM_AND_GRIPPER_NO_MAIN)
    add_dependencies(test_arm_cancellation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(test_arm_cancellation ${catkin_LIBRARIES} pthread)
  endif()
endif()

install(TARGETS servo_controller_node arm_and_gripper_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
