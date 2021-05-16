cmake_minimum_required(VERSION 3.0.2)
project(project1)

find_package(
  catkin REQUIRED
  COMPONENTS
    message_generation
    nav_msgs
    std_msgs
    roscpp
    geometry_msgs
    message_filters
    tf2
    tf2_ros
    tf2_geometry_msgs
    dynamic_reconfigure
    robotics_hw1
)

generate_dynamic_reconfigure_options(
  cfg/integration.cfg
)



add_service_files(
  FILES
    RiCentra.srv
    RiPosiziona.srv
)


add_message_files(
  FILES
    CustomOdom.msg
)



generate_messages(
  DEPENDENCIES
    std_msgs
    nav_msgs
    geometry_msgs
    
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    nav_msgs
    geometry_msgs
    message_filters
    message_runtime
)


include_directories(include ${catkin_INCLUDE_DIRS})


##estimate node

add_executable(estimate src/estimate.cpp)

add_dependencies(estimate ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(estimate project1_generate_messages_cpp)
add_dependencies(estimate ${catkin_EXPORTED_TARGETS})

target_link_libraries(estimate ${catkin_LIBRARIES})


##odometry node

add_executable(odometry src/odometry.cpp)

add_dependencies(odometry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(odometry project1_generate_messages_cpp)
add_dependencies(odometry ${PROJECT_NAME}_gencfg) ##for param config

target_link_libraries(odometry ${catkin_LIBRARIES})
