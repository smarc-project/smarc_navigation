cmake_minimum_required(VERSION 2.8.3)
project(auv_ekf_localization)

# check c++14 / c++0x
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
    rospy
    std_msgs
    tf
    message_generation
    geometry_msgs
    sensor_msgs
    message_filters
    pcl_ros
 )

find_package(PCL REQUIRED)
if(PCL_FOUND)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
endif()

#set(Boost_USE_STATIC_LIBS        ON)
find_package(Boost REQUIRED COMPONENTS system signals)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
endif()

# setup.py for map_provider_node.py
catkin_python_setup()

add_service_files(
    DIRECTORY srv
    FILES
    map_ekf.srv
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
    std_msgs
    )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sonar_manipulator
  CATKIN_DEPENDS roscpp rospy std_msgs tf message_runtime message_filters
#  DEPENDS system_lib
)


include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_library(sonar_manipulator
 src/sonar_manipulator.cpp
)

#add_library(mbes_receptor
#    src/mbes_receptor.cpp
#    )

add_executable(ekf_localization_node
    src/correspondence_obj.cpp
    src/ekf_localization.cpp
    src/ekf_localization_node.cpp
    src/mbes_receptor.cpp
    )

add_dependencies(ekf_localization_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(ekf_localization_node
    utils_matrices
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    )

add_executable(mbes_receptor_node
    src/mbes_receptor_node.cpp
    src/mbes_receptor.cpp
    )

add_dependencies(mbes_receptor_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(mbes_receptor_node
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
    sonar_manipulator
    utils_matrices
    )












