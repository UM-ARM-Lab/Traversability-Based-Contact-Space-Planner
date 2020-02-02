cmake_minimum_required(VERSION 2.8.3)
project(openrave_catkin)

find_package(catkin REQUIRED)
find_package(openrave REQUIRED)

catkin_package(
    DEPENDS openrave
    CFG_EXTRAS "${PROJECT_NAME}-extras.cmake"
)

set(BASE_DIRECTORY "openrave-${OpenRAVE_LIBRARY_SUFFIX}")
set(DATA_DIRECTORY "${BASE_DIRECTORY}/data")
set(PLUGINS_DIRECTORY "${BASE_DIRECTORY}/plugins")
set(DATABASES_DIRECTORY "${BASE_DIRECTORY}/databases")

catkin_add_env_hooks("20.${PROJECT_NAME}"
    SHELLS sh
    DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks"
)
