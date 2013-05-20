function model = sfm_filter_idx (model, idx)
% SFM_FILTER_IDX - Remove points in the model according to an index list.
%
%   Usage:  model = sfm_filter_idx(model, idx);
%
%   Input:
%   model - Structure that contains all output data after running the sfm
%           algorithm. See sfm_model.m for details.
%   idx - List of indexes that will survive the filtering
%
%   Output:
%   model - Structure that contains all output data after filtering 
%
% Copyright: Carnegie Mellon University & Intel Corporation
% Author: Alvaro Collet (acollet@cs.cmu.edu)

% Remove them from the model
model.pts2D = model.pts2D(:, idx, :);
model.pts3D = model.pts3D(:, idx);
model.desc = model.desc(idx, :);

% Extended bundler models
if isfield(model, 'keys'), model.keys = model.keys(idx, :); end
if isfield(model, 'color3D'), model.color3D = model.color3D(:, idx); end
if isfield(model, 'pt_info'), model.pt_info = model.pt_info(idx); end
if isfield(model, 'num_views'), model.num_views = model.num_views(idx); end
if isfield(model, 'avg_err'), model.avg_err = model.avg_err(idx); end
