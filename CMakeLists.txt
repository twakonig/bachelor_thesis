cmake_minimum_required(VERSION 2.8.3)
project(target_tracking)

## Use C++11
add_definitions(-std=c++11)
## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
#add_definitions(-std=c++11 -Wall -Werror)

set(PACKAGE_DEPENDENCIES
        roscpp
        roslib
        cv_bridge
        image_transport
        sensor_msgs
        geometry_msgs
        #std_msgs
        #visualization_msgs
        message_generation
        #tf2
        #tf2_ros
        #confusion
        )


## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
            ${PACKAGE_DEPENDENCIES}
        )


## Find system libraries
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)


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
        INCLUDE_DIRS
            include
        LIBRARIES
            ${PROJECT_NAME}_core
        CATKIN_DEPENDS
            ${PACKAGE_DEPENDENCIES}
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        #  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}_core
        src/FeatureTriangulation.cpp include/target_tracking/FeatureTriangulation.hpp)


add_executable(FeatureTriangulation src/FeatureTriangulation.cpp)
target_link_libraries(FeatureTriangulation ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(target_tracking_node src/target_tracking_node.cpp)
#target_link_libraries(target_tracking_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})



#############
## Install ##
#############

install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )


