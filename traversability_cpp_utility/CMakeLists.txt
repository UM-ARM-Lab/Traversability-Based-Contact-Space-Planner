cmake_minimum_required(VERSION 2.8.3)
project(traversability_cpp_utility)

set(CMAKE_CXX_FLAGS "-std=c++0x -O3 -fopenmp ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED urdf srdfdom openrave_catkin)

find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(OpenRAVE REQUIRED)
find_package(OpenMP REQUIRED)

if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
  set(OpenMP_LIBRARIES gomp)
endif()

catkin_package()
catkin_python_setup()

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${OpenRAVE_INCLUDE_DIRS}
)
link_directories(
    ${Boost_LIBRARY_DIRS}
    ${catkin_LIBRARY_DIRS}
    ${OpenRAVE_LIBRARY_DIRS}
)

openrave_plugin("${PROJECT_NAME}_plugin"
    src/Utilities.cpp
    src/TrimeshSurface.cpp
    src/PointGrid.cpp
    src/ContactPoint.cpp
    src/ContactRegion.cpp
    src/SurfaceContactPointGrid.cpp
    src/MapGrid.cpp
    src/ContactState.cpp
    src/TraversabilityCPPUtility.cpp
    src/Drawing.cpp
    src/Boundary.cpp
)

target_link_libraries("${PROJECT_NAME}_plugin"
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenMP_LIBRARIES}
)
