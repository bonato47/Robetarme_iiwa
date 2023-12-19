cmake_minimum_required(VERSION 3.3)
project(metal_additive_manufacturing)

set(CMAKE_CXX_STANDARD 20)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS
  system
  unit_test_framework
  log
  log_setup
)

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
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  libs
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(PLANNER "maplanner")
add_executable(${PLANNER}
  src/maplanner.cpp
)

target_link_libraries(${PLANNER}
  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

set(WP_EXTRACTION "wp_extraction")
add_executable(${WP_EXTRACTION}
  src/waypoints_extraction.cpp
)

target_link_libraries(${WP_EXTRACTION}
  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

# # Test suite
# add_executable(${PROJECT_NAME}_test tests/test_maplanner.cpp)
# target_link_libraries(${PROJECT_NAME}_test ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

# enable_testing()
# add_test(NAME TestInitialState COMMAND ${PROJECT_NAME}_test)
