cmake_minimum_required(VERSION 2.8.3)
project(depth)

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
#IF(NOT CMAKE_BUILD_TYPE)
#	    SET(CMAKE_BUILD_TYPE Release)
#ENDIF(NOT CMAKE_BUILD_TYPE)
################################################################
#find_package(ZED 2.0 REQUIRED)
#exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
#unset(CUDA_USE_STATIC_CUDA_RUNTIME CACHE)
#option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
#message ("CUDA_USE_STATIC_CUDA_RUNTIME : ${CUDA_USE_STATIC_CUDA_RUNTIME}")



find_package(ZED 2.2 REQUIRED)
find_package(CUDA 8.0 REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  #cv_bridge
  sensor_msgs
  #pcl_conversions
  #image_transport
  #rosconsole
  #dynamic_reconfigure
  #tf2_ros
  #nodelet
)

catkin_package(
	LIBRARIES depth
	CATKIN_DEPENDS
	  roscpp
	  #cv_bridge
	  sensor_msgs
	  #opencv
	  #pcl_conversions
	  #image_transport
	  #dynamic_reconfigure
	  #tf2_ros
)

############################################################
#INCLUDES

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${ZED_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  #${PCL_INCLUDE_DIRS}
)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY})
#link_directories(${PCL_LIBRARY_DIRS})
#############################################################
#EXECUTABLE
add_definitions(-std=c++11)
set(LINK_LIBRARIES
	  ${catkin_LIBRARIES}
	  ${ZED_LIBRARIES}
	  ${CUDA_LIBRARIES} ${CUDA_nppi_LIBRARY} ${CUDA_npps_LIBRARY}
	  ${OpenCV_LIBRARIES}
	  #${PCL_LIBRARIES}
)

add_executable(depth src/talker.cpp)
add_dependencies(depth ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(depth ${LINK_LIBRARIES} )

#############################################################
