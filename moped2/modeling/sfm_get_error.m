function rep_err = sfm_get_error (model)
% SFM_GET_ERROR - Compute average reprojection error of pts3D into pts2D
%
%   Usage:  rep_err = sfm_get_error (model);
%
%   Input:
%   model - Structure that contains all output data after running the sfm
%           algorithm. See sfm_model.m for details.
%
%   Output:
%   rep_err - N-by-1 list of average reprojection errors of pts3D across 
%             views. Each point's error is normalized by the number of
%             views it appears in.
%
% Copyright: Carnegie Mellon University & Intel Corporation
% Author: Alvaro Collet (acollet@cs.cmu.edu)

% Prealloc memory
TotalErr = zeros(size(model.pts3D,2),1);

for i = 1:model.nViews
    
    % Weights: 1 for points that appear in the image, zero otherwise
    idx_valid = find(model.pts2D(1,:,i) ~= 0);
        
    % Project 3D points onto the image plane
    proj_pts2D = projectPts(model.pts3D(:, idx_valid), model.cam_poses(:,i), model.int_mat, 1);

    % Reprojection error for each point
    TotalErr(idx_valid) = TotalErr(idx_valid) + sqrt(sum((model.pts2D(:,idx_valid,i)-proj_pts2D).^2, 1)');
end

% Count in how many 2D views a pts3D appears
num_views_pts = sum( (sum(model.pts2D, 1) ~= 0), 3);

% Average error per view
rep_err = TotalErr(:) ./ num_views_pts(:);



