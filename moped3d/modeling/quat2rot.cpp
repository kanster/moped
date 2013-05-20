//**********************************************************************//
//                                                                      
// quat2rot.cpp
//
// Quaternion to Rotation transformation 
//
// Copyright: Carnegie Mellon University & Intel Corporation
// Author: Alvaro Collet (acollet@cs.cmu.edu)
//
//**********************************************************************//

#include <math.h>


void quat2rot(double *q, double *R){
// QUAT2ROT - Quaternion to rotation matrix transformation
//
//  Usage: quat2rot(q, R); q: quaternion, R: rotation matrix

double x, y, z, w, x2, y2, z2, w2, xy, xz, yz, wx, wy, wz;
double norm_q = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);

x = q[0] / norm_q;
y = q[1] / norm_q;
z = q[2] / norm_q;
w = q[3] / norm_q;
 	
x2 = x*x;  y2 = y*y;  z2 = z*z;  w2 = w*w;
xy = 2*x*y;  xz = 2*x*z;  yz = 2*y*z;
wx = 2*w*x;  wy = 2*w*y;  wz = 2*w*z;

// Format: {R[0] R[1] R[2]; R[3] R[4] R[5]; R[6] R[7] R[8];}
R[0] = w2+x2-y2-z2; R[1] = xy-wz; R[2] = xz+wy;
R[3] = xy+wz; R[4] = w2-x2+y2-z2; R[5] = yz-wx;
R[6] = xz-wy; R[7] = yz+wx; R[8] = w2-x2-y2+z2;

}
