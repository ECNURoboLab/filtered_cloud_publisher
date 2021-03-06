cmake_minimum_required(VERSION 2.8.3)
project(filtered_cloud_publisher)

## Use C++11
# USE PCL so no c++11
#add_definitions(--std=c++11)

# set test or not
set(CATKIN_ENABLE_TESTING false)


## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
        roscpp
        pcl_conversions
        pcl_ros
        tf
        std_msgs
        geometry_msgs
        gpd
        message_generation
)

find_package(Boost REQUIRED)

## Generate actions in the 'action' folder
add_action_files(
        FILES
        GpdAdapter.action
)
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        actionlib_msgs
)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
#  LIBRARIES
  CATKIN_DEPENDS
    sensor_msgs
#  DEPENDS
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

## Declare a cpp library
add_library(${PROJECT_NAME}_core
   src/Algorithm.cpp
)

## Declare cpp executables
add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}_node.cpp
        src/FilteredCloudPublisher.cpp
)

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)


add_executable(gpd_adapter_action_node
        src/gpd_adapter_action_node.cpp
        src/GpdAdapterActionServer.cpp
        )
add_dependencies(gpd_adapter_action_node
        ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(gpd_adapter_action_node
        ${catkin_LIBRARIES}
        )

add_executable(action_client_test
        src/action_client_test.cpp)
target_link_libraries(action_client_test
        ${catkin_LIBRARIES})
#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
  DIRECTORY doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test
  test/test_ros_package_template.cpp
  test/AlgorithmTest.cpp)
endif()

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)
endif()
