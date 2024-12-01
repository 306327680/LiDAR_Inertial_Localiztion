cmake_minimum_required(VERSION 3.0.2)
project(LiDAR_Inertia_Localiztion)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")
find_package(Boost REQUIRED COMPONENTS timer)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARIES_DIRS})

find_package(GTSAM REQUIRED QUIET)
include_directories(${GTSAM_INCLUDE_DIR})
link_directories(${GTSAM_LIBRARY_DIRS})

find_package(Ceres REQUIRED)
include_directories(  ${CERES_INCLUDE_DIRS})


find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        pcl_conversions
        geometry_msgs
        diagnostic_msgs
        tf
)
catkin_package(
)

include_directories(
        include
  ${catkin_INCLUDE_DIRS}
)

add_library(registration
        include/registration/registration.cpp
        include/registration/registration.h
        include/pointType/pointTypes.h
        include/IMULiDARicp/imuLiDARicp.cpp
        include/IMULiDARicp/imuLiDARicp.h)
target_link_libraries(registration
        ${CERES_LIBRARIES}
        ${PCL_LIBRARIES})

add_executable(LiDAR_matching src/LiDAR_matching_node.cpp include/LiDAR_matching_lib/LiDAR_matching_lib.h
        include/LiDAR_matching_lib/LiDAR_matching_lib.cpp include/pointType/pointTypes.h include/utils.cpp)
add_dependencies(LiDAR_matching ${catkin_EXPORTED_TARGETS}  )
target_link_libraries(LiDAR_matching ${catkin_LIBRARIES}   registration ${PCL_LIBRARIES})

add_executable(IMU_Preintegration src/IMU_preintergration_node.cpp include/utils.cpp include/IMU_preintergration_lib/IMU_preintergration_lib.cpp)
add_dependencies(IMU_Preintegration ${catkin_EXPORTED_TARGETS}  )
target_link_libraries(IMU_Preintegration ${catkin_LIBRARIES}   registration ${PCL_LIBRARIES} gtsam)