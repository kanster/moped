% CVUNDISTORT - Matlab wrapper of cvUndistort2 to remove nonlinear distortion
%
%   Usage:  img = imundistort(input_img, K_mat, distortion);\n\
%
%   Input:
%   input_img - Color or grayscale image to be undistorted. It can be either
%               UINT8 or DOUBLE.
%   K_mat - 4-by-1 double array with intrinsic camera params [fx fy cx cy].
%   distortion - 4-by-1 or 5-by-1 double array with radial and tangential
%                 distortion parameters [r^2 r^4 t^2 t^4].
%
%   Output:
%   img - Output undistorted image.
% 
% Copyright: Carnegie Mellon University & Intel Corporation
% Author: Alvaro Collet (acollet@cs.cmu.edu)
