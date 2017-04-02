cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_tilt)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  dynamixel_sdk
  dynamixel_workbench_msgs
  dynamixel_workbench_toolbox
  laser_assembler
  msg
  pcl_ros
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamixel_tilt
  CATKIN_DEPENDS roscpp std_msgs dynamixel_workbench_msgs tf msg laser_assembler pcl_ros
  DEPENDS roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)
 add_executable(dynamixel_tilt src/dynamixel_tilt.cpp)
 add_dependencies(dynamixel_tilt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(dynamixel_tilt ${catkin_LIBRARIES})

 add_executable(send_angle src/send_angle.cpp)
 add_dependencies(send_angle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(send_angle ${catkin_LIBRARIES})

 add_executable(dynamixel_tf src/dynamixel_tf.cpp)
 add_dependencies(dynamixel_tf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(dynamixel_tf ${catkin_LIBRARIES})

 add_executable(laser_assembler src/laser_assembler.cpp)
 add_dependencies(laser_assembler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(laser_assembler ${catkin_LIBRARIES})
