function [roll, pitch, yaw] = rot2rpy(rot)
% From rotation matrix to roll, pitch, yaw.
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

A = rot(2,3)+rot(1,2);
B = rot(1,3)-rot(2,2);
sinpitch = -rot(3,1);
if sinpitch == 1
	yawplusroll = 0;
else
	yawplusroll = atan2(A/(sinpitch-1), B/(sinpitch-1));
end

A = rot(2,3)-rot(1,2);
B = rot(1,3)+rot(2,2);
if sinpitch == -1
	yawminusroll = 0;
else
	yawminusroll = atan2(A/(sinpitch+1), B/(sinpitch+1));
end

yaw = (yawplusroll + yawminusroll) / 2.0;
roll = (yawplusroll - yawminusroll) / 2.0;
pitch = atan2(-rot(3,1), rot(1,1)*cos(yaw)+rot(2,1)*sin(yaw));

end

