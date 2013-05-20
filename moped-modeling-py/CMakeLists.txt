cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# add_subdirectory(${PROJECT_SOURCE_DIR}/src)
#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/imundistort.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# Find the Python libraries
FIND_PACKAGE ( PythonLibs )
IF (NOT PYTHON_FOUND AND PYTHON_LIBRARIES)
  SET(PYTHON_FOUND "YES")
ELSE (NOT PYTHON_FOUND AND PYTHON_LIBRARIES)
  SET(PYTHON_FOUND "NO")
ENDIF(NOT PYTHON_FOUND AND PYTHON_LIBRARIES)
IF (PYTHON_FOUND)
  MESSAGE(STATUS "Found Python libraries in ${PYTHON_INCLUDE_PATH} as ${PYTHON_LIBRARIES}")
ELSE (PYTHON_FOUND)
  MESSAGE(FATAL_ERROR "Python libraries not found")
ENDIF (PYTHON_FOUND) 
INCLUDE_DIRECTORIES ( ${PYTHON_INCLUDE_PATH} )

EXECUTE_PROCESS(COMMAND python -c "import pyublas, os.path; print(os.path.join(os.path.dirname(pyublas.__path__[0]), 'include'))" OUTPUT_VARIABLE PYUBLAS_INCLUDE_PATH OUTPUT_STRIP_TRAILING_WHITESPACE)
MESSAGE(STATUS "Found PyUBlas in ${PYUBLAS_INCLUDE_PATH}")
INCLUDE_DIRECTORIES(${PYUBLAS_INCLUDE_PATH})

ROSBUILD_ADD_LIBRARY(imundistort SHARED src/imundistort.cpp src/cvundistort.cpp src/cvundistort.h)
rosbuild_add_boost_directories()
rosbuild_link_boost(imundistort python)
