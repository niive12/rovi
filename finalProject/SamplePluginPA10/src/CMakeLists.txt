# Test CMake version
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)

# The name of the project.
PROJECT(RoVi1PluginPA10)
MESSAGE(${PROJECT_NAME} ":")

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Set the RobWork root (edit in .bashrc if necessary)
SET(RW_ROOT $ENV{RW_ROOT})

# Set the RobWorkStudio root (edit in .bashrc if necessary)
SET(RWSTUDIO_ROOT $ENV{RWS_ROOT})

#USING FIND_PACKAGE INSTEAD:
# Set the OpenCV root (edit in .bashrc if necessary)
#SET(OpenCV_ROOT $ENV{OpenCV_ROOT})
#MESSAGE("-- OPENCV: " ${OpenCV_ROOT})

# Set the level of verbosity when running make.
set(CMAKE_VERBOSE_MAKEFILE False)

# Set build type to release
IF( NOT CMAKE_BUILD_TYPE ) # "NOT DEFINED" will not work, since "CMAKE_BUILD_TYPE" defined but set empty
  SET(CMAKE_BUILD_TYPE release)
ENDIF()
MESSAGE("-- Build type: " ${CMAKE_BUILD_TYPE})

# Use RobWork
MESSAGE("-- RobWork root: " ${RW_ROOT})
SET(RobWork_DIR ${RW_ROOT}/cmake) 
FIND_PACKAGE(RobWork REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

# Use RobWorkStudio
MESSAGE("-- RobWorkStudio root: " ${RWSTUDIO_ROOT})
SET(RobWorkStudio_DIR ${RWSTUDIO_ROOT}/cmake)
FIND_PACKAGE(RobWorkStudio REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORKSTUDIO_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS} )

# Use OpenCV
#SET(OpenCV_DIR ${OpenCV_ROOT}/cmake) #USING FIND PACKAGE
MESSAGE("-- OpenCV root: " ${OpenCV_ROOT})
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${OpenCV_LIB_DIR} )
MESSAGE("-- OpenCV dir: " ${OpenCV_DIR})

# Set the output dir for generated libraries and binaries
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/../bin/${CMAKE_BUILD_TYPE}" CACHE PATH "Runtime directory" FORCE)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/../libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Library directory" FORCE)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/../libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Archive directory" FORCE)

# If we want to use ui files add them here
QT4_WRAP_UI(UIS_OUT_H SamplePlugin.ui)
# Moc
QT4_WRAP_CPP(MocSrcFiles SamplePlugin.hpp)
# Rcc the files:
QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)

add_definitions(-std=c++11)

# Add the source files to build (besides the Qt generated source files)
SET(SrcFiles SamplePlugin.cpp visualServoing.cpp vis_marker01.cpp vis_marker02.cpp vis_marker03.cpp find_limits_dt.cpp find_trackingerror.cpp)

# The shared library to build:
ADD_LIBRARY(SamplePlugin MODULE ${SrcFiles} ${UIS_OUT_H} ${MocSrcFiles} ${RccSrcFiles})
TARGET_LINK_LIBRARIES(SamplePlugin ${OpenCV_LIBS} ${ROBWORK_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES})

MESSAGE(${PROJECT_NAME} " done!")