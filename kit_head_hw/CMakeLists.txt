cmake_minimum_required(VERSION 2.8.3)
project(kit_head_hw)

## DEPENDENCIES

find_package(catkin REQUIRED COMPONENTS
  roscpp
  control_toolbox
  controller_manager
  hardware_interface 
  transmission_interface
  pluginlib
  joint_limits_interface
  urdf
  angles
  gazebo_ros
  gazebo_ros_control
)

find_package(gazebo REQUIRED)

add_definitions (-fpermissive -std=c++11)

## CATKIN CONFIGURATION

catkin_package(
  CATKIN_DEPENDS
    roscpp
    controller_manager
    pluginlib
    transmission_interface
    gazebo_ros_control
  INCLUDE_DIRS include
  LIBRARIES kit_head_hw_sim
)

## BUILD

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)



# the plugin
add_library(kit_head_gazebo_ros_control src/kit_head_hw_sim_plugin.cpp)
set_property(TARGET kit_head_gazebo_ros_control PROPERTY CXX_STANDARD 14)
target_link_libraries(kit_head_gazebo_ros_control
	${catkin_LIBRARIES} 
	${GAZEBO_LIBRARIES}
)

# the default sim behavior
add_library(kit_head_hw_sim src/default_kit_head_hw_sim.cpp)
target_link_libraries(kit_head_hw_sim  
  ${catkin_LIBRARIES} 
  ${GAZEBO_LIBRARIES}
)

add_dependencies(
  kit_head_gazebo_ros_control
  ${catkin_EXPORTED_TARGETS}
)

## INSTALL

install(TARGETS kit_head_gazebo_ros_control kit_head_hw_sim
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(FILES kit_head_hw_sim_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )



