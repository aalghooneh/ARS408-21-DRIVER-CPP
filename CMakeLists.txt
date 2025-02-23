cmake_minimum_required(VERSION 3.0.2)
project(decode_radars)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  can_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  message_filters
  message_generation
  tf
)

set (Boost_INCLUDE_DIRS "/usr/include/boost")
set (Boost_LIBRARY_DIRS "/usr/lib/x86_64-linux-gnu/")
set (Eigen_INCLUDE_DIRS "/usr/include/eigen3/")
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(OpenCV 4.5.2 REQUIRED)

find_package(PCL REQUIRED COMPONENTS)




## Generate messages in the 'msg' folder
add_message_files(
   FILES
   ContiRadar.msg
   ContiRadarList.msg
   ContiList.msg
   ClusterRadar.msg
   ClusterList.msg
   radarCluster.msg
   radarClusterList.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   can_msgs
   geometry_msgs 
   nav_msgs
   std_msgs
 )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS message_runtime message_filters
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


link_directories(${Boost_LIBRARY_DIRS})

add_executable(decode_node src/decode_node.cpp src/decoder.cpp )
add_dependencies(decode_node ${decode_info_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(decode_node 
                ${PCL_LIBRARIES}
                ${catkin_LIBRARIES} 
                Boost::program_options)



add_executable(pointcloud_node src/_2pointcloud.cpp)
target_link_libraries(pointcloud_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
