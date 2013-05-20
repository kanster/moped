PROJECT(meanshift)
cmake_minimum_required(VERSION 2.6)

# Find Boost and Boost.python
FIND_PACKAGE( Boost )
FIND_PACKAGE( Boost COMPONENTS python )
set(Boost_USE_MULTITHREAD ON)

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
INCLUDE_DIRECTORIES ( ${Boost_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES ( ${PYTHON_INCLUDE_PATH} )

# If somebody knows a better way of finding python libraries automatically, please let
# me know!
# EXECUTE_PROCESS(COMMAND python -c "import pyublas, os.path; print(os.path.join(os.path.dirname(pyublas.__path__[0]), 'include'))" OUTPUT_VARIABLE PYUBLAS_INCLUDE_PATH OUTPUT_STRIP_TRAILING_WHITESPACE)
# MESSAGE(STATUS "Found PyUBlas in ${PYUBLAS_INCLUDE_PATH}")
# INCLUDE_DIRECTORIES(${PYUBLAS_INCLUDE_PATH})

ADD_LIBRARY(meanshift SHARED meanshiftpy.cpp)
target_link_libraries(meanshift ${Boost_PYTHON_LIBRARY})

