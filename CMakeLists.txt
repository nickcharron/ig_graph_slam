cmake_minimum_required(VERSION 2.8.3)
project(ig_graph_slam)

## Compile as C++11, supported in ROS Kinetic and newer
SET(CMAKE_CXX_FLAGS "-std=gnu++11 -Wall -Wextra ${CMAKE_CXX_FLAGS}")
#SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake_modules")

ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)

FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(Boost COMPONENTS REQUIRED log_SETup log)
FIND_PACKAGE(PCL 1.8 REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(yaml-cpp REQUIRED)
FIND_PACKAGE(GTSAM REQUIRED)
FIND_PACKAGE(GeographicLib 1.49 REQUIRED)
FIND_PACKAGE(wave REQUIRED utils matching containers geometry gtsam)
FIND_PACKAGE(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  )
FIND_PACKAGE(beam REQUIRED utils calibration)

ADD_DEFINITIONS(${PCL_DEFINITIONS})
ADD_DEFINITIONS(${GeographicLib_DEFINITIONS})

## Specify additional locations of header files
## Your package locations should be listed before other locations
INCLUDE_DIRECTORIES(
        include/ig_graph_slam
        ~/catkin_ws/devel/include
        #~/ig_catkin_ws/devel/include
        /opt/ros/kinetic/include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR} # NOTE: No 'S' for Boost's include directory
        /usr/local/include/gtsam/3rdparty/Eigen
        /usr/include/eigen3
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIRS}
        ${GeographicLib_INCLUDE_DIRS}
)

LINK_DIRECTORIES(
        ${PCL_LIBRARY_DIRS}
        ~/catkin_ws/devel/lib
        #~/ig_catkin_ws/devel/lib
        /opt/ros/kinetic/lib
)

ADD_EXECUTABLE(${PROJECT_NAME}_node
        src/main.cpp
        src/load_ros_data.cpp
        src/gtsam_graph.cpp
        src/scan_matcher.cpp
        src/slam_params.cpp
        src/world_frame_conversions.cpp
        )

TARGET_LINK_LIBRARIES(${PROJECT_NAME}_node
        wave::wave
        beam::utils
        beam::calibration
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        ${GeographicLib_LIBRARIES}
        gtsam
        rosbag
        rosbag_storage
        roscpp_serialization
        roscpp
        cpp_common
        roslib
        cv_bridge
        rostime
        rosbag
        rosconsole
)

catkin_package(
)
