cmake_minimum_required(VERSION 2.8.3)
project(evo_rd_platform_example)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11 -O2)
set(CMAKE_THREAD_PREFERE_PTHREAD)
set(THREADS_PREFERE_PTHREAD_FLAG)
find_package(Threads REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
find_package(evo-mbed-tools REQUIRED)

add_compile_options(-std=c++17  -DROS -O2)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    roscpp
    rospy
    tf
    sensor_msgs
    message_generation
    evo_dcdc_shield_interface
    evo_tof_board_interface
    evo_robot_base_interface
    evo_logger)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
    FILES
    evo_ToF_detailed.msg
#   Message2.msg
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

## Generate added messages and services with any dependencies listed here
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
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
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
#  LIBRARIES evo_rd_platform_example
#  CATKIN_DEPENDS
   DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/evo_rd_platform_example.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

add_executable(dcdc_controller_node
    src/node/EvoDCDCControllerNode.cpp

    src/sw_interface/DCDCControllerROS.cpp
    include/sw_interface/DCDCControllerROS.h
    )

add_executable(base_controller_node
    src/node/EvoBaseControllerNode.cpp

    src/sw_interface/BaseControllerROS.cpp
    include/sw_interface/BaseControllerROS.h)

# MMA: this is moved into the sim package - idk if it could be here 
#add_executable(sim_base_controller_node
#    src/node/EvoSimBaseControllerNode.cpp

#    src/sw_interface/SimBaseControllerROS.cpp
#    include/sw_interface/SimBaseControllerROS.h)

add_executable(tof_controller_node
    src/node/EvoToFControllerNode.cpp

    src/sw_interface/ToFControllerROS.cpp
    include/sw_interface/ToFControllerROS.h
    )


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
#set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above

add_dependencies(dcdc_controller_node ${catkin_EXPORTED_TARGETS})
add_dependencies(base_controller_node ${catkin_EXPORTED_TARGETS})
#add_dependencies(sim_base_controller_node ${catkin_EXPORTED_TARGETS})
add_dependencies(tof_controller_node ${catkin_EXPORTED_TARGETS} 
                                     ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(dcdc_controller_node
   ${catkin_LIBRARIES}
   evo-mbed-tools::evo-mbed-tools
   Threads::Threads
 )

target_link_libraries(base_controller_node
   ${catkin_LIBRARIES}
   evo-mbed-tools::evo-mbed-tools
   Threads::Threads
 )

# MMA: this is moved into the sim package - idk if it could be here 
# target_link_libraries(sim_base_controller_node
#   ${catkin_LIBRARIES}
#   evo-mbed-tools::evo-mbed-tools
#	Threads::Threads
# )

target_link_libraries(tof_controller_node
   ${catkin_LIBRARIES}
   evo-mbed-tools::evo-mbed-tools
   Threads::Threads
 )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_evo_rd_platform_example.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
