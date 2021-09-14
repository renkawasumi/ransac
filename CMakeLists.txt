cmake_minimum_required(VERSION 2.8.3)
project(ransac)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(    
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(
    ransac
    src/main.cpp
)

target_link_libraries(
    ransac
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
