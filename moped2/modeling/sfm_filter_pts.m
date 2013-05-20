function [model, idx_valid_pts] = sfm_filter_pts (model, threshold)
% SFM_FILTER_PTS - Remove points in the model with high reprojection error.
%   Filter 3D points by projecting them into each view and removing those
%   with a reprojection error higher than a certain value. If not threshold
%   is given, this function uses the value 'model.opts.reproj_th'.
%
%   Usage:  model = sfm_filter_pts(model, threshold);
%           [model idx_removed_pts] = sfm_filter_pts(model, threshold);
%
%   Input:
%   model - Structure that contains all output data after running the sfm
%           algorithm. See sfm_model.m for details.
%   threshold - (optional) limit reprojection error for a 3D point to be an
%               inlier (average per view).
%
%   Output:
%   model - Structure that contains all output data after filtering those
%           points with high reprojection error.
%   idx_valid_pts - list of indexes of those points that we keep.
%
% Copyright: Carnegie Mellon University & Intel Corporation
% Author: Alvaro Collet (acollet@cs.cmu.edu)

% Check arguments
if nargin < 2,
    threshold = model.opts.reproj_th;
end

% Get average reprojection error
TotalErr = sfm_get_error (model);

% Choose points to keep
idx_valid_pts = find(TotalErr < threshold);

% Remove them from the model
model = sfm_filter_idx(model, idx_valid_pts);

