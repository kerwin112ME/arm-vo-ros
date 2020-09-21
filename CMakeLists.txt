cmake_minimum_required(VERSION 3.0.2)
project(arm_vo)

SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -O3 -std=c++11 -mfpu=neon -mfloat-abi=hard")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  image_transport
  cv_bridge
  roscpp
  std_msgs
)

find_package(OpenCV 3 REQUIRED)

find_package(OpenMP QUIET)
if(OpenMP_FOUND)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
else()
    message(WARNING "OpenMP not found.")
endif()

catkin_package(
  INCLUDE_DIRS include
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB SOURCES "src/*.cpp")
add_executable(ARM_VO ${SOURCES})
add_dependencies(ARM_VO ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ARM_VO
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)



