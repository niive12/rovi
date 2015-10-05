project(pathplanner)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})
# Verbose mode prints more info when doing make
set(CMAKE_VERBOSE_MAKEFILE true)


# ----------------- User settings ----------------- #

add_definitions("-std=c++11") # Enable C++11.
# add_definitions("-std=c++1y") # Enable C++14

# ------------------------------------------------- #


# The name of the project
MESSAGE(${PROJECT_NAME} ":")

# Set the RobWork root (edit in .bashrc if necessary)
SET(RW_ROOT $ENV{RW_ROOT})
# Use RobWork
SET(RobWork_DIR ${RW_ROOT}/cmake)
FIND_PACKAGE(RobWork REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

# The shared library to build:
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${ROBWORK_LIBRARIES})

MESSAGE(${PROJECT_NAME} " done!")

