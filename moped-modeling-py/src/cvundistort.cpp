/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                        Intel License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of Intel Corporation may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

// Modified by Alvaro Collet

#include <opencv/cv.h>

// Add these defines to patch up the code
#define  CV_STUB_STEP     (1 << 30)
#define CV_8TO32F(x) ((float) x)
typedef unsigned char uchar;

void
cvUndistort_8u( const uchar* src, int srcstep,
                     uchar* dst, int dststep, CvSize size,
                     const float* intrinsic_matrix,
                     const float* dist_coeffs, int cn )
{
    int u, v, i;
    float u0 = intrinsic_matrix[2], v0 = intrinsic_matrix[5];
    float x0 = (size.width-1)*0.5f, y0 = (size.height-1)*0.5f;
    float fx = intrinsic_matrix[0], fy = intrinsic_matrix[4];
    float ifx = 1.f/fx, ify = 1.f/fy;
    float k1 = dist_coeffs[0], k2 = dist_coeffs[1], k3 = dist_coeffs[4];
    float p1 = dist_coeffs[2], p2 = dist_coeffs[3];

    srcstep /= sizeof(src[0]);
    dststep /= sizeof(dst[0]);

    for( v = 0; v < size.height; v++, dst += dststep )
    {
        float y = (v - v0)*ify, y2 = y*y;

        for( u = 0; u < size.width; u++ )
        {
            float x = (u - u0)*ifx, x2 = x*x, r2 = x2 + y2, _2xy = 2*x*y;
            float kr = 1 + ((k3*r2 + k2)*r2 + k1)*r2;
            float _x = fx*(x*kr + p1*_2xy + p2*(r2 + 2*x2)) + u0;
            float _y = fy*(y*kr + p1*(r2 + 2*y2) + p2*_2xy) + v0;
            int ix = cvFloor(_x), iy = cvFloor(_y);

            if( (unsigned)iy < (unsigned)(size.height - 1) &&
                (unsigned)ix < (unsigned)(size.width - 1) )
            {
                const uchar* ptr = src + iy*srcstep + ix*cn;
                _x -= ix; _y -= iy;
                for( i = 0; i < cn; i++ )
                {
                    float t0 = CV_8TO32F(ptr[i]), t1 = CV_8TO32F(ptr[i+srcstep]);
                    t0 += _x*(CV_8TO32F(ptr[i+cn]) - t0);
                    t1 += _x*(CV_8TO32F(ptr[i + srcstep + cn]) - t1);
                    dst[u*cn + i] = (uchar)cvRound(t0 + _y*(t1 - t0));
                }
            }
            else
            {
                for( i = 0; i < cn; i++ )
                    dst[u*cn + i] = 0;
            }
            
        }
    }
}


void
cvUndistort( const CvArr* _src, CvArr* _dst, const CvMat* A, const CvMat* dist_coeffs )
{

    float a[9], k[5]={0,0,0,0,0};
    int coi1 = 0, coi2 = 0;
    CvMat srcstub, *src = (CvMat*)_src;
    CvMat dststub, *dst = (CvMat*)_dst;
    CvMat _a = cvMat( 3, 3, CV_32F, a ), _k;
    int cn, src_step, dst_step;
    CvSize size;

    // Convert types
    cvConvert( A, &_a );
    _k = cvMat( dist_coeffs->rows, dist_coeffs->cols, CV_32F, k );
    cvConvert( dist_coeffs, &_k );
    
    src = cvGetMat( src, &srcstub, &coi1 );
    dst = cvGetMat( dst, &dststub, &coi2 );

    cn = CV_MAT_CN(src->type);
    size = cvGetSize(src);
    src_step = src->step ? src->step : CV_STUB_STEP;
    dst_step = dst->step ? dst->step : CV_STUB_STEP;

    cvUndistort_8u( src->data.ptr, src_step,
        dst->data.ptr, dst_step, size, a, k, cn );

}
