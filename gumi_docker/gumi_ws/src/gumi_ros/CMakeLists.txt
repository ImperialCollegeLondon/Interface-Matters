cmake_minimum_required(VERSION 2.8.3)
project(gumi_ros)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    geometry_msgs
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
# 
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#  
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   
# )

###################################
## catkin specific configuration ##
###################################

# add_service_files(FILES
#     PickAndPlaceService.srv
# )

# generate_messages(DEPENDENCIES geometry_msgs)

catkin_package()

###########
## Build ##
###########

# include_directories(
# include
#   ${catkin_INCLUDE_DIRS}
# )

#############
## Install ##
#############

# catkin_install_python(PROGRAMS
#   scripts/gumi_ros_node.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
