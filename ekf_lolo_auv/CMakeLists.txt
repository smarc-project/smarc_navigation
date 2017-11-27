cmake_minimum_required(VERSION 2.8.3)
project(ekf_lolo_auv)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" HAS_FLAGS)
if(HAS_FLAGS)
    add_compile_options(-std=c++11 -Wall -Wextra)
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    ekf_general
 )

#set(Boost_USE_STATIC_LIBS        ON)
find_package(Boost 1.36.0 COMPONENTS
    system
    )

if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
endif()


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS roscpp rospy std_msgs ekf_general #message_runtime
#  DEPENDS system_lib
)


include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_library(ekf_class
 src/ekf_class.cpp
)

add_executable(lolo_auv_navigation
    src/lolo_auv_nav.cpp
    src/lolo_auv_nav_node.cpp
    )

add_dependencies(lolo_auv_navigation
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
#    lolo_navigation_node_generate_messages_cpp
    )

target_link_libraries(lolo_auv_navigation
    utils_matrices
    ${catkin_LIBRARIES}
    )
