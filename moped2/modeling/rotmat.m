function H = rotmat(angle, dir)
% ROTMAT - Create a rotation matrix with ANGLE in dir DIR.
%
%   Usage: H = rotmat(angle, dir)
%   
%   Input:
%   angle - angle in radians
%   dir - Axis of rotation: 'x', 'y' or 'z' or any 3-vector to define axis.
%
%   Examples:
%   H = rotmat(angle, 'x');
%   H = rotmat(angle, [0 1 0]);
%   H = rotmat(angle, 'z');
%
% Copyright: Carnegie Mellon University & Intel Corporation
% Author: Alvaro Collet (acollet@cs.cmu.edu)

if ischar(dir)
    switch dir
        case 'x'
            dir = [1; 0; 0];
        case 'y'
            dir = [0; 1; 0];
        case 'z'
            dir = [0; 0; 1];
    end
else
    dir = dir(:)./norm(dir(:));
end

% Rotation across arbitrary direcion (see wikipedia)
H = [0 -dir(3) dir(2); dir(3) 0 -dir(1); -dir(2) dir(1) 0] * sin(angle) + cos(angle)*(eye(3)-dir*dir') + dir*dir';
