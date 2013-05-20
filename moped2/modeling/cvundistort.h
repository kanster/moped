// Copyright: Carnegie Mellon University & Intel Corporation
// Author: Alvaro Collet (acollet@cs.cmu.edu)

#ifndef __CV_UNDISTORT_H__
#define __CV_UNDISTORT_H__

// CVUNDISTORT: Both input and output arrays must be of type CV_8U or IPL_DEPTH_8U.
// Also, SRC and DST *cannot* be the same array.
void cvUndistort( const CvArr* _src, CvArr* _dst, const CvMat* A, const CvMat* dist_coeffs );

#endif
