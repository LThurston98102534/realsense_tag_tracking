# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(realsense_ar_tracking_2)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs ar_track_alvar_msgs genmsg tf geometry_msgs)

add_definitions( -DANALYSIS=ON )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system signals)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wdeprecated-declarations")
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

message ( STATUS " CMake C++ FLAGS ${CMAKE_CXX_FLAGS}")

## Declare ROS messages and services
#add_message_files(FILES AlvarMarker.msg AlvarMarkers.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs ar_track_alvar_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(tracker_2 src/main.cpp src/tracker.cpp)
target_link_libraries(tracker_2 ${catkin_LIBRARIES})

#add_executable(main src/main.cpp)
#target_link_libraries(main ${catkin_LIBRARIES})

add_executable(tf_broadcaster_2 src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster_2 ${catkin_LIBRARIES})

add_executable(camera_calibrator src/camera_calib.cpp src/calibrator.cpp)
target_link_libraries(camera_calibrator ${catkin_LIBRARIES})

add_executable(sealer_fridge_calibrator src/sealer_fridge_calib.cpp src/calibrator.cpp)
target_link_libraries(sealer_fridge_calibrator ${catkin_LIBRARIES})

add_executable(alvar_tester src/test_alvar.cpp src/alvar_tester.cpp)
target_link_libraries(alvar_tester ${catkin_LIBRARIES})


# %EndTag(FULLTEXT)%
