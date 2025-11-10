cmake_minimum_required(VERSION 3.0.2)
project(graph_search_wholebody_contact_planner_sample)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  graph_search_wholebody_contact_planner
  c-wcp
  choreonoid_viewer
  choreonoid_bullet
  choreonoid_cddlib
  ik_constraint2_bullet
  choreonoid_contact_candidate_generator
  roslib
  )

find_package(choreonoid REQUIRED)

catkin_package(
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${CHOREONOID_INCLUDE_DIRS}
)

add_library(CnoidGraphSearchWholeBodyContactPlannerSamplePlugin SHARED
  src/CnoidGraphSearchWholeBodyContactPlannerSamplePlugin.cpp
  src/world_common.cpp
  src/jaxon_common.cpp
  src/sample0_display.cpp
  )
target_link_libraries(CnoidGraphSearchWholeBodyContactPlannerSamplePlugin
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_PLUGIN_LIBRARIES}
  )
set_target_properties(CnoidGraphSearchWholeBodyContactPlannerSamplePlugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR})
