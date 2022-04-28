cmake_minimum_required(VERSION 3.0.2)
project(artuga)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    mavros
     mavros_msgs
    cv_bridge
    image_transport
    pcl_conversions
    pcl_ros
)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

include(MavrosMavlink)

catkin_package(

)

###########
## Build ##
###########

include_directories(

    ${catkin_INCLUDE_DIRS}
    ${mavlink_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}

)

add_executable(rosnode_artuga src/rosnode_artuga.cpp src/camera/camera.cpp src/artugaMarker/artugaMarker.cpp)

target_link_libraries(rosnode_artuga ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})





#############
## Install ##
#############



#############
## Testing ##
#############


