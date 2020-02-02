cmake_minimum_required (VERSION 2.6.0)
project(elastic_strips)

set(CMAKE_CXX_FLAGS "-std=c++0x -O3 -fopenmp ${CMAKE_CXX_FLAGS}")

find_package(OpenRAVE 0.9 REQUIRED)

if (DEFINED CATKIN_TOPLEVEL OR (NOT ("$ENV{ROS_DISTRO}" STREQUAL "fuerte")))
    include(${PROJECT_SOURCE_DIR}/catkin.cmake)
else ()
    include(${PROJECT_SOURCE_DIR}/rosbuild.cmake)
endif ()

# if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
#   add_definitions("-fno-strict-aliasing -Wall")
# endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

# optional in case boost is used
# find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT)

# include_directories(${OpenRAVE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
# link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
# add_library(elastic_strips SHARED ElasticStrips.cpp)
# set_target_properties(elastic_strips PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
# target_link_libraries(elastic_strips ${OpenRAVE_LIBRARIES})
# install(TARGETS elastic_strips DESTINATION .)
