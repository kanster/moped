//**********************************************************************//
//                                                                      
// matlab_cv.h
//
// Conversion functions mxArray <--> IplImage and mxArray <--> CvMat
// to interface Matlab with OpenCV.
//
// Copyright: Carnegie Mellon University & Intel Corporation
// Author: Alvaro Collet (acollet@cs.cmu.edu)
//
//**********************************************************************//

#ifndef __MATLAB_CV__
#define __MATLAB_CV__

#include <cv.h>
#include <mex.h>

// Data pointer definition
typedef union
    {
        unsigned char* u8;
        char* i8;
        unsigned int* u16;
		int* i16;
        float* fl;
        double* db;
    } data;


// MX_TO_IPLIMAGE: Convert mxArray into IplImage. This function allocates 
// necessary memory for you. You need to free it later: cvReleaseImage(img)
IplImage* mx_to_IplImage(const mxArray *mx_img);
    
// IPLIMAGE_TO_MX: Convert IplImage into mxArray. This function allocates 
// necessary memory for you. No need to deallocate anything
void IplImage_to_mx(IplImage *img, mxArray **mx_img);

// MX_TO_CVMAT: Convert mxArray to CvMat, copying data. This function allocates 
// necessary memory for you. You need to free memory with cvReleaseMat(&my_mat);
CvMat* mx_to_CvMat(const mxArray *mx_mat);

// MX_TO_CVMAT_T: Convert mxArray to CvMat, without copying data. Due 
// to the differences between Matlab and OpenCV, this function outputs the 
// transposed matrix to that of the original Matlab array, but it is way 
// faster because no data is copied. This function allocates necessary 
// memory for you. You need to free memory with cvReleaseMatHeader(&my_mat);
// WARNING: If you modify the returned CvMat, BAD THINGS WILL HAPPEN!
CvMat* mx_to_CvMat_T(const mxArray *mx_mat);

// CVMAT_TO_MX: Convert 2D CvMat to mxArray, copying data. This function allocates 
// necessary memory for you. No need to free any memory from this function.
void CvMat_to_mx(CvMat *mat, mxArray **mx_mat);

// CVMAT_TO_MX_T: Convert 2D CvMat to mxArray, transposing the matrix at
// the same time. This function is designed to work with MX_TO_CVMAT_T.
// This function allocates necessary memory for you. No need to free any
// memory from this function.
void CvMat_to_mx_T(CvMat *mat, mxArray **mx_mat);

#endif // __MATLAB_CV__
