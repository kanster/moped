function R = quat2rot(q)
% QUAT2ROT - Quaternion to rotation matrix transformation
%
%   Usage: R = quat2rot(q);
%
% Copyright: Carnegie Mellon University & Intel Corporation
% Author: Alvaro Collet (acollet@cs.cmu.edu)

% Alvaro: Add this to make it consistent with rot2quat.m
q = q([2 3 4 1]);

q = q/norm(q, 2);
 	
x = q(1);  y = q(2);  z = q(3);  w = q(4);
x2 = x*x;  y2 = y*y;  z2 = z*z;  w2 = w*w;
xy = 2*x*y;  xz = 2*x*z;  yz = 2*y*z;
wx = 2*w*x;  wy = 2*w*y;  wz = 2*w*z;

R = [w2+x2-y2-z2, xy-wz, xz+wy;
     xy+wz, w2-x2+y2-z2, yz-wx;
     xz-wy, yz+wx, w2-x2-y2+z2];
