% Compile files

% Copyright: Carnegie Mellon University & Intel Corporation
% Author: Alvaro Collet (acollet@cs.cmu.edu)

mex rodrigues.cpp -I/usr/local/include/opencv -lcxcore -lcv -lhighgui
mex quat2rot.cpp 
mex meanshift/meanShift1.c
movefile meanShift1.* meanshift
mex imundistort.cpp matlab_cv.cpp cvundistort.cpp -I/usr/local/include/opencv -lcxcore -lcv -lhighgui
