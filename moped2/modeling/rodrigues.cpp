// RODRIGUES - Transform rotation matrix into rotation vector and viceversa
//      We use the OpenCV implementation for speed.
//  		
//      Usage:  [OUT]=RODRIGUES(IN)
//      If IN is a 3x3 rotation matrix then OUT is the corresponding 3x1 rotation vector
//   	if IN is a rotation 3-vector then OUT is the corresponding 3x3 rotation matrix
//
// Copyright: Carnegie Mellon University & Intel Corporation
// Author: Alvaro Collet (acollet@cs.cmu.edu)

#include "cv.h"
#include "highgui.h"
#include "math.h"
#include "mex.h"

void mexFunction(
    int nargout,
    mxArray *out[],
    int nargin,
    const mxArray *in[]
)
{
    /* declare variables */
    double *mx_r_in;
    double *mx_r_out;
    int output_dim = 3;
    
    /* check arguments */
    if (nargin != 1 || nargout > 1){
        mexErrMsgTxt("Wrong Number of arguments.");
        exit(1);
    }
    
    // Link input vars to pointers in C
    mx_r_in = mxGetPr(in[0]);
    int m = mxGetM(in[0]);
    int n = mxGetN(in[0]);
    
    // Input is a rotation matrix
    if (m == 3 && n == 3){
        output_dim = 1;
    }
    
    // Check input argument: avoid errors
    if (!((m == 3 && n == 3) || (m == 1 && n == 3) || (m == 3 && n == 1))){
        mexPrintf("HELP! ERROR! %d %d\n", m, n);
        exit(1);
    }
    
    // Create OpenCV array for input variable
    // If we want to use cvSetData, our matrices are actually the transposed
    // versions of those that come from Matlab.
    CvMat *r_in_T = cvCreateMatHeader(m, n, CV_64F);
    cvSetData (r_in_T, mx_r_in, sizeof(double)*n);
    
    // Transpose the matrix
    CvMat *r_in = cvCreateMat(n, m, CV_64F);
    cvT(r_in_T, r_in);
    
    // Result
    CvMat *r_out_T = cvCreateMat(output_dim, 3, CV_64F);
    
    // Call cvRodrigues
    cvRodrigues2(r_in, r_out_T);
    
    // Allocate memory for the output var
    out[0] = mxCreateNumericMatrix(3, output_dim, mxDOUBLE_CLASS, mxREAL);
    mx_r_out = mxGetPr(out[0]);    
    
    CvMat* r_out = cvCreateMatHeader(3, output_dim, CV_64F);
    cvSetData (r_out, mx_r_out, sizeof(double)*output_dim);
    cvT(r_out_T, r_out);

    // Free all array headers and return
    cvReleaseMat(&r_in);
    cvReleaseMatHeader(&r_in_T);
    cvReleaseMatHeader(&r_out);

}
