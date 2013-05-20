#!/bin/sh
# Copyright: Carnegie Mellon University and Intel Corporation
# Author: Alvaro Collet (acollet@cs.cmu.edu)

OPENCV_DIR=`rospack find -q opencv2`
OPENCV_INCLUDE_DIR=`rospack find -q opencv2`/opencv/include
OPENCV_LIB_DIR=`rospack find -q opencv2`/opencv/lib
OPENCV_CFLAGS=`rospack export --lang=cpp --attrib=cflags opencv2`
OPENCV_LFLAGS=`rospack export --lang=cpp --attrib=lflags opencv2`

echo -n "prefix=${OPENCV_DIR}\n\
libdir=${OPENCV_INCLUDE_DIR}\n\
includedir=${OPENCV_LIB_DIR}\n\
\n\
Name: OpenCV\n\
Description: OpenCV as built by ROS\n\
Version: 2.0\n\
Libs: ${OPENCV_LFLAGS}\n\
CFlags: ${OPENCV_CFLAGS}" > opencv.pc

export PKG_CONFIG_PATH=${PKG_CONFIG_PATH}:`pwd`
