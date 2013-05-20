function [pts2D in_front] = projectPts(varargin)
% PROJECTPTS - Use the perspective projection to map pts in 3D to 2D.
%   Function to use in SFM to jointly optimize the camera poses and 3D
%   points. If you are using this function along with Levenberg-Marquardt
%   optimization, you will find the 'alternative usage' useful.
%
%   Usage: pts2D = projectPts(pts3D, cam_poses, int_mat, nViews);
%   Alternative usage: pts2D = projectPts(VectorizeVars(pts3D, ...
%                              cam_poses), int_mat, nViews);
%   Alternative usage 2: pts2D = projectPts(VectorizeVars(pts3D, ...
%                              cam_poses, int_mat), nViews);
%
%   Input:
%   pts3D - 3-by-N array of 3D points to be backprojected
%   cam_poses - 6-by-K array of 6-DOF camera poses [r1 r2 r3 t1 t2 t3]',
%               where [r1 r2 r3] is a rodrigues rotation vector (world to
%               camera transformations, i.e. Xc = R*Xw + T
%   nViews - number of different camera views we are projecting
%   int_mat - internal camera parameters: [fx fy px py]. 
%
%   Output:
%   pts2D - 2-by-KN array of 2D points backprojected to the camera plane
%   in_front - KN-by-1 array. 1 if the point is in front of the camera, 0
%              if the point is in the back.
%   See also VECTORIZEVARS, RODRIGUES
%
% Copyright: Carnegie Mellon University & Intel Corporation
% Author: Alvaro Collet (acollet@cs.cmu.edu)

% Analyze the inputs and extract the vars we need
[pts3D, cam_poses, int_mat, nViews] = checkargs(varargin);

nPts = size(pts3D, 2);

% Prealloc memory
pts2D = zeros(2, nViews*nPts);
in_front = zeros(1, nViews*nPts);

% Backproject for each camera pose
for i = 1:nViews

    % Get rotation and translation
    R = rodrigues(cam_poses(1:3, i));
    T = cam_poses(4:6,i);
    
    % Transform points in world coords to camera coords
    tx_pts3D = R*pts3D + repmat(T, [1 size(pts3D, 2)]);
    
    % Perspective projection (I use abs(Z) because I don't want points
    % behind the camera
    pts2D(1, (i-1)*nPts+1:i*nPts) = int_mat(1) .* tx_pts3D(1,:) ./ tx_pts3D(3,:) + int_mat(3);
    pts2D(2, (i-1)*nPts+1:i*nPts) = int_mat(2) .* tx_pts3D(2,:) ./ tx_pts3D(3,:) + int_mat(4);
    in_front(1, (i-1)*nPts+1:i*nPts) = tx_pts3D(3,:) > 0;
end
in_front = in_front(:);

% ----------------------------------------------------------------------- %
function [pts3D, cam_poses, int_mat, nViews] = checkargs(arguments)
% Analyze the inputs and extract the vars we need
% 4 args --> inputs = outputs
% 3 args --> pts3D, cam_poses are vectorized
% 2 args --> pts3D, cam_poses, int_mat are vectorized

switch length(arguments)
    case 4,
        pts3D = arguments{1};
        cam_poses = arguments{2};
        int_mat = arguments{3};
        nViews = arguments{4};
        
    case 3,
        nViews = arguments{3};
        int_mat = arguments{2};
        cam_poses = arguments{1}(1:6*nViews);
        cam_poses = reshape(cam_poses, [6 nViews]);
        
        pts3D = arguments{1}(6*nViews+1:end);
        pts3D = reshape(pts3D, [3 numel(pts3D)/3]);

    case 2,
        nViews = arguments{2};
        int_mat = arguments{1}(1:4);
        cam_poses = arguments{1}(4+(1:6*nViews));
        cam_poses = reshape(cam_poses, [6 nViews]);
        
        pts3D = arguments{1}(6*nViews+5:end);
        pts3D = reshape(pts3D, [3 numel(pts3D)/3]);
    otherwise
        display('Unrecognized options');
end
