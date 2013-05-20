// Copyright: Carnegie Mellon University & Intel Corporation
// Author: Alvaro Collet (acollet@cs.cmu.edu)

const char description [] = "\n\
 CVUNDISTORT - Matlab wrapper of cvUndistort2 to remove nonlinear distortion\n\
\n\
   Usage:  img = cvUndistort(input_img, K_mat, distortion);\n\
\n\
   Input:\n\
   input_img - Color or grayscale image to be undistorted. It can be either\n\
               UINT8 or DOUBLE.\n\
   K_mat - 4-by-1 double array with intrinsic camera params [fx fy cx cy].\n\
   distortion - 4-by-1 or 5-by-1 double array with radial and tangential\n\
                distortion parameters [r^2 r^4 t^2 t^4].\n\
\n\
   Output:\n\
   img - Output undistorted image.\n\
\n\
 Alvaro Collet\n\
 acollet@cs.cmu.edu\n\
";

#include <cv.h>
#include <mex.h>
#include "cvundistort.h"
#include "matlab_cv.h"

void mexFunction(
    int nargout,
    mxArray *out[],
    int nargin,
    const mxArray *in[]
)
{
    /* declare variables */
	unsigned char *outimg;
	int camID;
    double *kmat, *dist;
    float kmat_f[4], dist_f[4];
    
    long ChannelSize;
    
    kmat = NULL;
    dist = NULL;
    
    if (nargin == 3){
        kmat = mxGetPr(in[1]); 
        dist = mxGetPr(in[2]);
        
        // Convert from double to float
        for (int i = 0; i < 4; i++){
            kmat_f[i] = (float) kmat[i];
            dist_f[i] = (float) dist[i];
        }
        
    }else{
        mexWarnMsgTxt(description);
        mexErrMsgTxt("Not enough input arguments.");
        }
    if (nargout > 1) {
        mexWarnMsgTxt(description);
        mexErrMsgTxt("Too many output arguments.");
    }
    
    // Map input to image
    IplImage *img;
    img = mx_to_IplImage(in[0]);
    if (img == NULL){
        mexPrintf("Error! Could not allocate memory\n");
        return;
    }

    IplImage *out_img = cvCloneImage(img);

    float KK[9] = {kmat_f[0], 0, kmat_f[2], 0, kmat_f[1], kmat_f[3], 0, 0, 1};
    CvMat FullKK = cvMat(3, 3, CV_32FC1, KK);
    CvMat dist_coeffs = cvMat(4, 1, CV_32FC1, dist_f);
    
    // Undistort with OpenCV
    cvUndistort(img, out_img, &FullKK, &dist_coeffs);
    
    // Copy to Matlab
    IplImage_to_mx(out_img, &out[0]);

    //Free memory
    cvReleaseImage(&img);
    cvReleaseImage(&out_img);
} 
