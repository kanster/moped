cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

find_package(OpenCV REQUIRED)

add_custom_target(BUILD_LIBMOPED DEPENDS libmoped.a)
add_custom_command(OUTPUT libmoped.a 
                   DEPENDS ${PROJECT_SOURCE_DIR}/libmoped/src/moped.cpp
                   DEPENDS ${PROJECT_SOURCE_DIR}/libmoped/src/config.hpp
                   DEPENDS ${PROJECT_SOURCE_DIR}/libmoped/src/util.hpp
                   DEPENDS ${PROJECT_SOURCE_DIR}/libmoped/src/STATUS_DISPLAY.hpp
                   DEPENDS ${PROJECT_SOURCE_DIR}/libmoped/src/GLOBAL_DISPLAY.hpp
                   COMMAND make
                   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/libmoped
                   COMMENT Building libmoped.a)

link_directories(${PROJECT_SOURCE_DIR}/libmoped/lib)
include_directories(${PROJECT_SOURCE_DIR}/libmoped/include)

rosbuild_add_executable(moped2 moped.cpp)
rosbuild_add_compile_flags(moped2 "-O3 -Wall -march=native -mfpmath=sse -fopenmp")
rosbuild_add_link_flags(moped2 "-Wl,--start-group")
target_link_libraries(moped2 
  gomp GLEW glut IL 
  ${PROJECT_SOURCE_DIR}/libmoped/lib/libmoped.a 
  ${OpenCV_LIBS}
  )

add_dependencies(moped2 BUILD_LIBMOPED)

add_custom_command(TARGET moped2 PRE_BUILD COMMAND ./find_opencv2.sh WORKING_DIRECTORY ..)



rosbuild_add_executable(moped2_test moped_test.cpp)

rosbuild_add_link_flags(moped2_test "-Wl,--start-group")
rosbuild_add_compile_flags(moped2_test "-O3 -Wall -march=native -mfpmath=sse -fopenmp")
target_link_libraries(moped2_test gomp GLEW glut IL ${PROJECT_SOURCE_DIR}/libmoped/lib/libmoped.a) 
add_custom_command(TARGET moped2_test
                   PRE_BUILD COMMAND ./find_opencv2.sh WORKING_DIRECTORY ..)
add_dependencies(moped2_test BUILD_LIBMOPED)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

