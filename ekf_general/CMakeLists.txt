cmake_minimum_required(VERSION 2.8.3)
project(ekf_general)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" HAS_FLAGS)
if(HAS_FLAGS)
    add_compile_options(-std=c++11 -Wall -Wextra)
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    genmsg
    message_generation
    )

#set(Boost_USE_STATIC_LIBS        ON)
find_package(Boost 1.36.0 COMPONENTS
    system
    )

if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
endif()


# Generate messages in the 'msg' folder
 add_message_files(
   DIRECTORY msg
   FILES
   sensors_read.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ekf_general
  CATKIN_DEPENDS std_msgs roscpp message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
 add_library(landmark_ML
   src/landmark_ml.cpp
   include/landmark_ml.hpp
 )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(localization_node
    src/localization_node.cpp
    src/localization.cpp
    include/localization.hpp
)

add_executable(sens_sim_node
    src/sens_sim_node.cpp
    src/sens_sim.cpp
    include/sens_sim.hpp
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(localization_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )

add_dependencies(sens_sim_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    localization_node_generate_messages_cpp
    )

# Specify libraries to link a library or executable target against
target_link_libraries(localization_node
   ${catkin_LIBRARIES}
   landmark_ML
   ${Boost_LIBRARIES}
   )

target_link_libraries(sens_sim_node
   ${catkin_LIBRARIES}
   landmark_ML
   ${Boost_LIBRARIES}
   )

