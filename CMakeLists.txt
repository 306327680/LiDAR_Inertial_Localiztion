cmake_minimum_required(VERSION 3.0.2)
project(LiDAR_Inertia_Localiztion)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARIES_DIRS})

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        pcl_conversions
)

catkin_package(

)

include_directories(
        include
  ${catkin_INCLUDE_DIRS}
)
add_executable(LiDAR_matching src/LiDAR_matching.cpp include/LiDAR_matching_lib/LiDAR_matching_lib.h include/LiDAR_matching_lib/LiDAR_matching_lib.cpp include/pointType/pointTypes.h)
add_dependencies(LiDAR_matching ${catkin_EXPORTED_TARGETS}  )
target_link_libraries(LiDAR_matching ${catkin_LIBRARIES}   ${PCL_LIBRARIES})
