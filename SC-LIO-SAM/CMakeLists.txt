cmake_minimum_required(VERSION 2.8.3)
project(lio_sam)

set(CMAKE_BUILD_TYPE "release")
# set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")
#set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CMAKE_COMMAND} -E time")
#set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
#set(CMAKE_C_COMPILER "/usr/bin/clang")
#find_program(CLANGPP clang++ HINTS /usr/local/bin /usr/bin /opt/local/bin)
#if(${CLANGPP} MATCHES "clang")
#set(CXX ${CLANGPP})
#set(MAKE_CXX_COMPILER ${CLANGPP})
#message(STATUS "CXX = ${CXX}")
#endif(${CLANGPP} MATCHES "clang")



find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  # pcl library
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED )
find_package (PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM

  CATKIN_DEPENDS 
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime 
  message_generation
  LIBRARIES ${PROJECT_NAME}_EXPORT
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

###########
## Build ##
###########


# Range Image Projection
#Zadd_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
#add_dependencies(${PROJECT_NAME}_imageProjection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(${PROJECT_NAME}_imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Feature Association
#add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
#add_dependencies(${PROJECT_NAME}_featureExtraction ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(${PROJECT_NAME}_featureExtraction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Mapping Optimization

set (${PROJECT_NAME}_LIB_SRCS_EXPORT
  src/lio_sam/generics.cpp
)

add_library(${PROJECT_NAME}_EXPORT ${${PROJECT_NAME}_LIB_SRCS_EXPORT})
target_link_libraries(${PROJECT_NAME}_EXPORT ${catkin_LIBRARIES}  ${CERES_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_LIBRARIES} boost_serialization )

set (${PROJECT_NAME}_LIB_SRCS
  src/lio_sam/utility.cpp
  src/lio_sam/Scancontext.cpp
)


add_library(${PROJECT_NAME} ${${PROJECT_NAME}_LIB_SRCS})
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_EXPORT ${catkin_LIBRARIES}  ${CERES_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam ${Boost_FILESYSTEM_LIBRARY} )

#${PROJECT_NAME}_EXPORT

add_executable(${PROJECT_NAME}_mapOptmization src/mapOptmization.cpp)
add_dependencies(${PROJECT_NAME}_mapOptmization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_mapOptmization ${PROJECT_NAME} ${PROJECT_NAME}_EXPORT  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})




#add_executable(gps_converter src/gps_converter.cpp)
#add_dependencies(gps_converter ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(gps_converter ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam ${Boost_FILESYSTEM_LIBRARY}
#    ${Boost_SYSTEM_LIBRARY}
#    ${Boost_REGEX_LIBRARY})



# IMU Preintegration
#add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
#target_link_libraries(${PROJECT_NAME}_imuPreintegration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

