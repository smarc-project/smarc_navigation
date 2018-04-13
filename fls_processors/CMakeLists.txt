cmake_minimum_required(VERSION 2.8.3)
project(fls_processors)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "-std=c++14")
     add_compile_options(-std=c++14 -Wall -Wextra)
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++11")
     add_compile_options(-std=c++11 -Wall -Wextra)
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 or C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  geometry_msgs
  std_msgs
  sensor_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fls_processors
  CATKIN_DEPENDS roscpp tf   geometry_msgs std_msgs sensor_msgs cv_bridge
#  DEPENDS system_lib
)

include_directories(
 ${catkin_INCLUDE_DIRS}
 ${OPENCV_INCLUDE_DIRS}
)

add_executable(fls_rock_detector
    src/fls_rock_detector.cpp
)

add_dependencies(fls_rock_detector
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(fls_rock_detector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
