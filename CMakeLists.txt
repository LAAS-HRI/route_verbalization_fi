cmake_minimum_required(VERSION 2.8.3)
project(route_verbalization_fi)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ontologenius
  message_generation
  semantic_route_description
  dialogue_translator
)
find_package(ontologenius 0.1.3 REQUIRED)
find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
   FILES
   VerbalizeRegionRoute.srv
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
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
#  LIBRARIES semantic_route_drawer
  CATKIN_DEPENDS ontologenius roscpp semantic_route_description
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
  ${OpenCV_INCLUDE_DIRS}
  ${ontoloGenius_lib_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(route_verbalization_fi_lib
  src/mapManipulators/MapReader.cpp
  src/mapManipulators/MapDrawer.cpp
  src/Verbalizers/RegionVerbalizer.cpp
  src/Verbalizers/PlaceVerbalizer.cpp
  src/Verbalizers/sentences.cpp
)
 target_link_libraries(route_verbalization_fi_lib ${catkin_LIBRARIES})
 add_dependencies(route_verbalization_fi_lib ${catkin_EXPORTED_TARGETS})
 target_link_libraries(route_verbalization_fi_lib ${OpenCV_LIBS})



add_executable(verbalize_route_fi nodes/verbalize.cpp)
target_link_libraries(verbalize_route_fi ${catkin_LIBRARIES})
add_dependencies(verbalize_route_fi ${catkin_EXPORTED_TARGETS})
add_dependencies(verbalize_route_fi route_verbalization_gencpp)
target_link_libraries(verbalize_route_fi route_verbalization_fi_lib)

add_executable(verbalize_test_fi nodes/tester.cpp)
target_link_libraries(verbalize_test_fi ${catkin_LIBRARIES})
add_dependencies(verbalize_test_fi ${catkin_EXPORTED_TARGETS})
add_dependencies(verbalize_test_fi route_verbalization_gencpp)
target_link_libraries(verbalize_test_fi route_verbalization_fi_lib)

add_executable(all_sentences_fi nodes/allSentences.cpp)
target_link_libraries(all_sentences_fi ${catkin_LIBRARIES})
add_dependencies(all_sentences_fi ${catkin_EXPORTED_TARGETS})
add_dependencies(all_sentences_fi route_verbalization_gencpp)
target_link_libraries(all_sentences_fi route_verbalization_fi_lib)
