//**********************************************************************//
//                                                                      
// matlab_cv.cpp
//
// Conversion functions mxArray <--> IplImage and mxArray <--> CvMat
// to interface Matlab with OpenCV.
//
// Copyright: Carnegie Mellon University & Intel Corporation
// Author: Alvaro Collet (acollet@cs.cmu.edu)
//
//**********************************************************************//

#include "matlab_cv.h"


// Macro that helps making this code more easily readable
#define COPY_MXARRAY3D(mx, img, type, dims0, dims1, dims2, ch_size)    					\
for (long ch = 0; ch < dims2; ch++)                  							\
        for (long x = 0; x < dims1; x++)	      							\
            for (long y = 0; y < dims0; y++)      							\
		CV_IMAGE_ELEM(img, unsigned char, y, x*dims2 + ch) = mx[ch*ch_size + x*dims[0] + y];


//**********************************************************************//
// MX_TO_IPLIMAGE: Convert mxArray into IplImage. This function allocates 
// necessary memory for you. You need to free it later: cvReleaseImage(img)
IplImage* mx_to_IplImage(const mxArray *mx_img){

    int *dims, nDims, depth;
    int img_type;
    long int ChannelSize;
    data pImg;

    IplImage *img = NULL;

    // Get dimensions
    nDims = (int) mxGetNumberOfDimensions(mx_img);
    dims = (int *) mxGetDimensions(mx_img);
    CvSize img_size = cvSize(dims[1], dims[0]);
    
    // Choose correct channel depth
    switch (nDims){
        case 2: depth = 1; break;// 1 Channel
        case 3: depth = dims[2]; break;// Usually, 3 channels
        default: return NULL; // Something weird happened, get outta here!    
    }
    
    // Get Class ID from MxArray
    long int classID = mxGetClassID(mx_img);
    pImg.db = mxGetPr(mx_img);
    ChannelSize = dims[0]*dims[1];


    // Copy data
    switch(classID){
        case mxUINT8_CLASS: 
		img = cvCreateImage(img_size, IPL_DEPTH_8U, depth); 
		COPY_MXARRAY3D(pImg.u8, img, unsigned char, dims[0], dims[1], dims[2], ChannelSize); break;
	case mxINT8_CLASS: 
		img = cvCreateImage(img_size, IPL_DEPTH_8S, depth);
		COPY_MXARRAY3D(pImg.u8, img, char, dims[0], dims[1], dims[2], ChannelSize); break;
	case mxUINT16_CLASS: 
		img = cvCreateImage(img_size, IPL_DEPTH_16U, depth);
		COPY_MXARRAY3D(pImg.u8, img, unsigned int, dims[0], dims[1], dims[2], ChannelSize); break;
	case mxINT16_CLASS: 
		img = cvCreateImage(img_size, IPL_DEPTH_16S, depth);
		COPY_MXARRAY3D(pImg.u8, img, int, dims[0], dims[1], dims[2], ChannelSize); break;
	case mxSINGLE_CLASS: 
		img = cvCreateImage(img_size, IPL_DEPTH_32F, depth);
		COPY_MXARRAY3D(pImg.u8, img, float, dims[0], dims[1], dims[2], ChannelSize); break;
	case mxDOUBLE_CLASS: 
		img = cvCreateImage(img_size, IPL_DEPTH_64F, depth);
		COPY_MXARRAY3D(pImg.u8, img, double, dims[0], dims[1], dims[2], ChannelSize); break;
        default: 
		return NULL; break;           
    }
    return img;
}

// Macro that helps making this code more easily readable
#define COPY_IMAGE(mx, img, type, dims0, dims1, dims2, ch_size) 					\
for (long ch = 0; ch < dims2; ch++)                  							\
        for (long x = 0; x < dims1; x++)	      							\
            for (long y = 0; y < dims0; y++)      							\
		mx[ch*ch_size + x*dims0 + y] = CV_IMAGE_ELEM(img, type, y, x*dims2 + ch);
			

//**********************************************************************//
// IPLIMAGE_TO_MX: Convert IplImage into mxArray. This function allocates 
// necessary memory for you. No need to free any memory from this function
void IplImage_to_mx(IplImage *img, mxArray **mx_img){

    // Define vars
    int dims[3];
    data pImg;

    /* get all inputs */
    dims[0] = img->height;
    dims[1] = img->width;
    dims[2] = img->nChannels;

   // Allocate memory
    switch(img->depth){
        case IPL_DEPTH_8U: *mx_img = mxCreateNumericArray(3, dims, mxUINT8_CLASS, mxREAL); break;
	case IPL_DEPTH_8S: *mx_img = mxCreateNumericArray(3, dims, mxINT8_CLASS, mxREAL); break;
	case IPL_DEPTH_16U: *mx_img = mxCreateNumericArray(3, dims, mxUINT16_CLASS, mxREAL); break;
	case IPL_DEPTH_16S: *mx_img = mxCreateNumericArray(3, dims, mxINT16_CLASS, mxREAL); break;
	case IPL_DEPTH_32F: *mx_img = mxCreateNumericArray(3, dims, mxSINGLE_CLASS, mxREAL); break;
	case IPL_DEPTH_64F: *mx_img = mxCreateNumericArray(3, dims, mxDOUBLE_CLASS, mxREAL); break;
        default: return; break;           
    }

    // Get the pointer
    pImg.db = mxGetPr(*mx_img);
    long int ChannelSize = dims[0] * dims[1];

    // Copy data
    switch(img->depth){
        case IPL_DEPTH_8U: COPY_IMAGE(pImg.u8, img, unsigned char, dims[0], dims[1], dims[2], ChannelSize); break;
	case IPL_DEPTH_8S: COPY_IMAGE(pImg.i8, img, char, dims[0], dims[1], dims[2], ChannelSize); break;
	case IPL_DEPTH_16U: COPY_IMAGE(pImg.u16, img, unsigned int, dims[0], dims[1], dims[2], ChannelSize); break;
	case IPL_DEPTH_16S: COPY_IMAGE(pImg.i16, img, int, dims[0], dims[1], dims[2], ChannelSize); break;
	case IPL_DEPTH_32F: COPY_IMAGE(pImg.fl, img, float, dims[0], dims[1], dims[2], ChannelSize); break;
	case IPL_DEPTH_64F: COPY_IMAGE(pImg.db, img, double, dims[0], dims[1], dims[2], ChannelSize); break;
        default: return; break;           
    }    
}


// Macro that helps making this code more easily readable   
#define COPY_CVMAT(mx, mat, type, dims0, dims1) 							\
        for (long x = 0; x < dims1; x++)	      							\
            for (long y = 0; y < dims0; y++)      							\
		mx[x*dims0 + y] = CV_MAT_ELEM(*mat, type, y, x);

//**********************************************************************//
// MX_TO_CVMAT: Convert 2D mxArray to CvMat, copying data. This function allocates 
// necessary memory for you. You need to free memory with cvReleaseMat(&my_mat);
CvMat* mx_to_CvMat(const mxArray *mx_mat){

    // Define vars
    long int M, N;
    long int ChannelSize;
    CvMat *mat = NULL;
    data pImg;

    // get all inputs
    M = mxGetM(mx_mat);
    N = mxGetN(mx_mat);
    pImg.db = mxGetPr(mx_mat);

    // Get Class ID from MxArray
    long int classID = mxGetClassID(mx_mat);

    // Copy data
    switch(classID){
        case mxUINT8_CLASS: 
		mat = cvCreateMat(M, N, CV_8UC1); 
		COPY_CVMAT(pImg.u8, mat, unsigned char, M, N); break;
	case mxINT8_CLASS: 
		mat = cvCreateMat(M, N, CV_8SC1); 
		COPY_CVMAT(pImg.i8, mat, char, M, N); break;
	case mxUINT16_CLASS: 
		mat = cvCreateMat(M, N, CV_16UC1); 
		COPY_CVMAT(pImg.u16, mat, unsigned int, M, N); break;
	case mxINT16_CLASS: 
		mat = cvCreateMat(M, N, CV_16SC1); 
		COPY_CVMAT(pImg.i16, mat, int, M, N); break;
	case mxSINGLE_CLASS: 
		mat = cvCreateMat(M, N, CV_32FC1); 
		COPY_CVMAT(pImg.fl, mat, float, M, N); break;
	case mxDOUBLE_CLASS: 
		mat = cvCreateMat(M, N, CV_64FC1); 
		COPY_CVMAT(pImg.db, mat, double, M, N); break;
        default: 
		return NULL; break;           
    }
    return mat;
}

//**********************************************************************//
// MX_TO_CVMAT_T: Convert 2D mxArray to CvMat, without copying data. Due 
// to the differences between Matlab and OpenCV, this function outputs the 
// transposed matrix to that of the original Matlab array, but it is way 
// faster because no data is copied. This function allocates necessary 
// memory for you. You need to free memory with cvReleaseMatHeader(&my_mat);
// WARNING: If you modify the returned CvMat, BAD THINGS WILL HAPPEN!
CvMat* mx_to_CvMat_T(const mxArray *mx_mat){

    // Define vars
    long int M, N;
    long int ChannelSize;
    CvMat *mat = NULL;
    data pImg;

    // get all inputs
    M = mxGetM(mx_mat);
    N = mxGetN(mx_mat);
    pImg.db = mxGetPr(mx_mat);

    // Get Class ID from MxArray
    long int classID = mxGetClassID(mx_mat);

    // Copy data pointer only
    switch(classID){
        case mxUINT8_CLASS: 
		mat = cvCreateMatHeader(N, M, CV_8UC1); 
    		cvSetData (mat, pImg.u8, sizeof(unsigned char) * M); break;
	case mxINT8_CLASS: 
		mat = cvCreateMatHeader(N, M, CV_8SC1); 
    		cvSetData (mat, pImg.i8, sizeof(char) * M); break;
	case mxUINT16_CLASS: 
		mat = cvCreateMatHeader(N, M, CV_16UC1); 
    		cvSetData (mat, pImg.u16, sizeof(unsigned int) * M); break;
	case mxINT16_CLASS: 
		mat = cvCreateMatHeader(N, M, CV_16SC1); 
    		cvSetData (mat, pImg.i16, sizeof(int) * M); break;
	case mxSINGLE_CLASS: 
		mat = cvCreateMatHeader(N, M, CV_32FC1); 
    		cvSetData (mat, pImg.fl, sizeof(float) * M); break;
	case mxDOUBLE_CLASS: 
		mat = cvCreateMatHeader(N, M, CV_64FC1); 
    		cvSetData (mat, pImg.db, sizeof(double) * M); break;
        default: 
		return NULL; break;           
    }
    return mat;
}


// Macro that helps making this code more easily readable   
#define COPY_MXARRAY2D(mx, mat, type, dims0, dims1) 							\
        for (long x = 0; x < dims1; x++)	      							\
            for (long y = 0; y < dims0; y++)      							\
		CV_MAT_ELEM(*mat, type, y, x) = mx[x*dims0 + y];


//**********************************************************************//
// CVMAT_TO_MX: Convert 2D CvMat to mxArray, copying data. This function allocates 
// necessary memory for you. No need to free any memory from this function.
void CvMat_to_mx(CvMat *mat, mxArray **mx_mat){

    // Define vars
    long int M, N;
    data pImg;

    // Get dimensions
    M = mat->rows;
    N = mat->cols;

    // Allocate memory
    switch(mat->type){
        case CV_8UC1: *mx_mat = mxCreateNumericMatrix(M, N, mxUINT8_CLASS, mxREAL); break;
	case CV_8SC1: *mx_mat = mxCreateNumericMatrix(M, N, mxINT8_CLASS, mxREAL); break;
	case CV_16UC1: *mx_mat = mxCreateNumericMatrix(M, N, mxUINT16_CLASS, mxREAL); break;
	case CV_16SC1: *mx_mat = mxCreateNumericMatrix(M, N, mxINT16_CLASS, mxREAL); break;
	case CV_32FC1: *mx_mat = mxCreateNumericMatrix(M, N, mxSINGLE_CLASS, mxREAL); break;
	case CV_64FC1: *mx_mat = mxCreateNumericMatrix(M, N, mxDOUBLE_CLASS, mxREAL); break;
        default: return; break;           
    }

    // Get pointer
    pImg.db = mxGetPr(*mx_mat);

    // Copy data
    switch(mat->type){
        case CV_8UC1: COPY_MXARRAY2D(pImg.u8, mat, unsigned char, M, N); break;
	case CV_8SC1: COPY_MXARRAY2D(pImg.i8, mat, char, M, N); break;
	case CV_16UC1: COPY_MXARRAY2D(pImg.u16, mat, unsigned int, M, N); break;
	case CV_16SC1: COPY_MXARRAY2D(pImg.i16, mat, int, M, N); break;
	case CV_32FC1: COPY_MXARRAY2D(pImg.fl, mat, float, M, N); break;
	case CV_64FC1: COPY_MXARRAY2D(pImg.db, mat, double, M, N); break;
        default: return; break; 
    }
}


// Macro that helps making this code more easily readable   
#define COPY_MXARRAY2D_T(mx, mat, type, dims0, dims1) 							\
        for (long x = 0; x < dims1; x++)	      							\
            for (long y = 0; y < dims0; y++)      							\
		CV_MAT_ELEM(*mat, type, x, y) = mx[x*dims0 + y];


//**********************************************************************//
// CVMAT_TO_MX_T: Convert 2D CvMat to mxArray, transposing the matrix at
// the same time. This function is designed to work with MX_TO_CVMAT_T.
// This function allocates necessary memory for you. No need to free any
// memory from this function.
void CvMat_to_mx_T(CvMat *mat, mxArray **mx_mat){

    // Define vars
    long int M, N;
    data pImg;

    // Get dimensions (transposed)
    M = mat->cols;
    N = mat->rows;

    // Allocate memory
    switch(mat->type){
        case CV_8UC1: *mx_mat = mxCreateNumericMatrix(M, N, mxUINT8_CLASS, mxREAL); break;
	case CV_8SC1: *mx_mat = mxCreateNumericMatrix(M, N, mxINT8_CLASS, mxREAL); break;
	case CV_16UC1: *mx_mat = mxCreateNumericMatrix(M, N, mxUINT16_CLASS, mxREAL); break;
	case CV_16SC1: *mx_mat = mxCreateNumericMatrix(M, N, mxINT16_CLASS, mxREAL); break;
	case CV_32FC1: *mx_mat = mxCreateNumericMatrix(M, N, mxSINGLE_CLASS, mxREAL); break;
	case CV_64FC1: *mx_mat = mxCreateNumericMatrix(M, N, mxDOUBLE_CLASS, mxREAL); break;
        default: return; break;           
    }

    // Get pointer
    pImg.db = mxGetPr(*mx_mat);

    // Copy data
    switch(mat->type){
        case CV_8UC1: COPY_MXARRAY2D_T(pImg.u8, mat, unsigned char, M, N); break;
	case CV_8SC1: COPY_MXARRAY2D_T(pImg.i8, mat, char, M, N); break;
	case CV_16UC1: COPY_MXARRAY2D_T(pImg.u16, mat, unsigned int, M, N); break;
	case CV_16SC1: COPY_MXARRAY2D_T(pImg.i16, mat, int, M, N); break;
	case CV_32FC1: COPY_MXARRAY2D_T(pImg.fl, mat, float, M, N); break;
	case CV_64FC1: COPY_MXARRAY2D_T(pImg.db, mat, double, M, N); break;
        default: return; break; 
    }
}
