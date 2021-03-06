cmake_minimum_required(VERSION 2.8.3)
project(ros_pcl)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "-g -Wall -Wextra -O0")
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -Wextra -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-g -Wall -Wextra -O0")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-g -Wall -Wextra -O0")
set(CATKIN_DEVEL_PREFIX /root/catkin_ws/devel)

set(YAML_CPP_INCLUDE_DIR /usr/local/include)
set(YAML_CPP_LIBRARY /usr/local/lib/libyaml-cpp.so)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  visualization_msgs
  tf2_ros
  tf2_msgs
  tf2_sensor_msgs
  geometry_msgs
  ros_pcl_msgs
  tf2_eigen
  thesis_visualization_msgs
)
#find_library()
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(PCL 1.8 REQUIRED COMPONENTS common io search features keypoints kdtree octree visualization filters recognition sample_consensus surface registration)


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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   srv_getScenePcd.srv
#   srv_preprocess.srv
#   srv_detectors.srv
#   srv_descriptors.srv
#   srv_matching.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   sensor_msgs
#   visualization_msgs
# )

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
  LIBRARIES ros_pcl
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs std_msgs visualization_msgs ros_pcl_msgs geometry_msgs tf2_eigen tf2_ros thesis_visualization_msgs
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
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ros_pcl.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(getScenePcdSrv src/getScenePcdSrv.cpp)
target_link_libraries(getScenePcdSrv ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(preprocessSrvServer src/preprocessSrvServer.cpp include/pcl_objRec/impl/model_prepare.cpp 
                                                               include/pcl_objRec/impl/scene_prepare.cpp   
                                                               include/pcl_objRec/impl/preprocessor.cpp
                                                               include/pcl_objRec/impl/utils.cpp
                                                               include/pcl_objRec/impl/detectors.cpp
                                                               include/pcl_objRec/impl/descriptors.cpp )
target_link_libraries(preprocessSrvServer ${catkin_LIBRARIES} ${PCL_LIBRARIES})                                                               


#add_executable(modelRescale include/pcl_objRec/impl/modelRescale.cpp) 
#target_link_libraries(modelRescale ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(detectorSrvServer src/detectorSrvServer.cpp include/pcl_objRec/impl/detectors.cpp
                                                     include/pcl_objRec/impl/utils.cpp)
target_link_libraries(detectorSrvServer ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(descriptorSrvServer src/descriptorsSrvServer.cpp include/pcl_objRec/impl/utils.cpp
                                                           include/pcl_objRec/impl/descriptors.cpp)
target_link_libraries(descriptorSrvServer ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(matchingSrvServer src/matchingSrvServer.cpp include/pcl_objRec/impl/utils.cpp
                                                           include/pcl_objRec/impl/preprocessor.cpp)
target_link_libraries(matchingSrvServer ${catkin_LIBRARIES} ${PCL_LIBRARIES})


# add_executable(static_tf2_broadcaster src/static_broadcaster.cpp)
# target_link_libraries(static_tf2_broadcaster  ${catkin_LIBRARIES} )

add_executable(transformListener src/transformListener.cpp)
target_link_libraries(transformListener  ${catkin_LIBRARIES} )

add_executable(pcd_merge src/pcd_merge.cpp)
target_link_libraries(pcd_merge ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(matchingLocalPipeline include/pcl_objRec/impl/matchingLocalPipeline.cpp)
# target_link_libraries(matchingLocalPipeline ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${YAML_CPP_LIBRARY})

# add_executable(computeResolution src/computeResolution.cpp include/pcl_objRec/impl/utils.cpp)
# target_link_libraries(computeResolution ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(utils_tools include/pcl_objRec/impl/utils_tools.cpp)
target_link_libraries(utils_tools ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(recogSrv src/recogSrv.cpp)
target_link_libraries(recogSrv ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${YAML_CPP_LIBRARY})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against





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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ros_pcl.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
