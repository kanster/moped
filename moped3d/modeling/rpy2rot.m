function [rot] = rpy2rot(roll, pitch, yaw)
% From to roll, pitch, yaw to rotation matrix.
% roll, pitch, yaw rotation specs:
% * rotations are around fixed-axes.
% * roll is around x-axis, pitch is around y-axis, yaw is around z-axis.
% * assumed application order for the rotations are: first roll, then pitch, then yaw.
% Solution is based on:
% R(yaw, pitch, roll) = 
% | cos(yaw)*cos(pitch)   cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll)    cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll) |
% | sin(yaw)*cos(pitch)   sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll)    sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll) |
% | -sin(pitch)           cos(pitch)*sin(roll)                                cos(pitch)*cos(roll)                             |
%
% Copyright: Carnegie Mellon University & Intel Corporation
% Author: Mehmet Dogar(mdogar@andrew.cmu.edu)

 rot = [
  cos(yaw)*cos(pitch)   cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll)    cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll) ;
  sin(yaw)*cos(pitch)   sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll)    sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll) ;
  -sin(pitch)           cos(pitch)*sin(roll)                                cos(pitch)*cos(roll)                             ]
end


