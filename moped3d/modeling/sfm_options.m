function opts = sfm_options
% SFM_OPTIONS - Generate an 'options' structure for SFM with def. params.
%
%   Usage: opts = sfm_options();
%
%   Output:
%   opts - Structure with parameters and options that the SFM algorithm
%          uses. Currently, the fields are:
%       .verbose {true} - status messages and scores appear on screen.
%       .reproj_th {2.5} - Threshold (in pixels) to filter points with high
%                          reprojection error.
%       .min_matched_views {3} - Min. number of views in which a keypoint
%                                appears to be added to the 3D model.
%
% Copyright: Carnegie Mellon University & Intel Corporation
% Author: Alvaro Collet (acollet@cs.cmu.edu)


% General parameters ------------------------------------------------------
% Verbose: if true, status messages and scores appear on screen
opts.verbose = true;

% Filter points with high reprojection error
opts.reproj_th = 2.5;

% Minimum number of views in which a keypoint appears to be added to the 3D
% model
opts.min_matched_views = 3;
