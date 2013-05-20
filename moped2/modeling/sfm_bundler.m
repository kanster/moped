function model = sfm_bundler(name, image_dir, output_dir, cam_K, cam_dist, imsize, opts)
% SFM_BUNDLER - Create SFM model using (internally) the Bundler software
%
%   Usage: model = sfm_bundler(name, image_dir, output_dir, cam_K, ...
%                              cam_dist, imsize, opts)
%
%   Input:
%   name - Model name
%   image_dir - Path to image directory (should be JPG files), with N images
%   output_dir - Directory where to store all relevant data for a model.
%                WARNING: BOTH IMAGE_DIR and OUTPUT_DIR need to be FULL 
%                PATHS!! (or ./, which is also fine).
%   cam_K - 3-by-3 intrinsic camera matrix, or 4-by-1 [fx fy cx cy] params.
%   cam_dist - 5-by-1 camera distortion parameters (same format as the
%              Bouguet's Camera Calibration Toolbox). If the input images
%              have been previously undistorted, enter zeros(5,1) or [].
%   imsize - Size of each image (must be all the same), in [width height]
%   (default: [640 480])
%   opts - (optional) sfm_options structure, as defined in sfm_options.m
%
%   Output:
%   model - SFM model, as defined in SFM_MODEL.M
%
% Copyright: Carnegie Mellon University & Intel Corporation
% Author: Alvaro Collet (acollet@cs.cmu.edu)

% Check inputs
if nargin < 7, opts = sfm_options; end
if nargin < 6 || isempty(imsize), imsize = [640 480]; end
if nargin < 5 || isempty(cam_dist), cam_dist = zeros(5,1); end

if numel(cam_K) == 9,
    cam_K = [cam_K(1,1), cam_K(2,2) cam_K(1,3) cam_K(2,3)];
end

bundler_file = 'bundle.out';

% Run Bundler
RunBundler(image_dir, output_dir, cam_K, cam_dist, imsize);

display('Importing camera poses and points from Bundler...');
[cam_poses, model] = ReadCamsBundler(fullfile(output_dir, bundler_file), true, imsize);

model.name = name;
model.version = 'Bundler v1';
model.int_mat = cam_K;
model.nViews = size(model.cam_poses, 2);
model.opts = opts;

% Filter points behind the camera
for i = 1:model.nViews,
    pts2D_view = model.pts2D(:, :, i);
    idx_view = find(pts2D_view(1, :) > 0);
    [proj_pts2D in_front] = projectPts(model.pts3D(:, idx_view), model.cam_poses(:,i), model.int_mat, 1);
    if any(~in_front),
        model.pts2D(:, idx_view(~in_front), i) = zeros(2, sum(~in_front));
    end
end

% If segmentation data is available, filter points outside the mask too
masks = dir(fullfile(image_dir, '*.mat'));
if model.nViews == length(masks),
    % Filter each view, keep only points within the mask
    for i = 1:model.nViews,
        load (fullfile(image_dir, masks(i).name), 'mask');
        pts2D_view = model.pts2D(:, :, i);
        idx_view = find(pts2D_view(1, :) > 0);
        idx_pts = sub2ind(size(mask), round(pts2D_view(2, idx_view)), round(pts2D_view(1, idx_view)));
        invalid_idx = find(mask(idx_pts) == 0);
        model.pts2D(:, idx_view(invalid_idx), i) = zeros(2, length(invalid_idx));
    end
    % Recompute number of views
    model.num_views = sum( (sum(model.pts2D, 1) ~= 0), 3);

    % Fill with zeros, just for the time being
    model.desc = zeros(size(model.pts3D, 2), 128);
else
    display (['Segmentation data cannot be read. Please make sure ', ...
             'that the image folder contains ONLY images, masks and ', ...
             'keypoints.'])
end

% Filter points with less than a certain number of views
model.cam_nPoints = sum( (sum(model.pts2D, 1) ~= 0), 2);
model.cam_nPoints = model.cam_nPoints(:);
valid_pts = model.num_views >= model.opts.min_matched_views;
model = sfm_filter_idx(model, valid_pts);

% Move pts3D to be (roughly) in the world center
mu = mean(model.pts3D, 2);
model = sfm_transform_ext(model, eye(3), -mu, 1);

% Read gzipped SIFT descriptors
files = dir(fullfile(image_dir, '*.key.gz'));
files = {files.name};
display ('Reading SIFT features for all images...');
for i = 1:length(files),
    [locs{i}, desc{i}] = read_gzipped_sift(fullfile(image_dir, files{i}));
end

% Add info about descriptors to the model
for i = 1:size(model.pts3D, 2),
    idx_views = find(model.keys(i,:) ~= 0);
    pt_locs = zeros(length(idx_views), 4);
    pt_desc = zeros(length(idx_views), 128);
    for j = 1:length(idx_views),
        pt_locs(j, :) = locs{idx_views(j)}(model.keys(i, idx_views(j)), :);
        pt_desc(j, :) = desc{idx_views(j)}(model.keys(i, idx_views(j)), :);
    end
    model.pt_info(i).locs = pt_locs;
    model.pt_info(i).desc = pt_desc;
    model.pt_info(i).cam_id = idx_views;
    
    % Create some pseudo-descriptor for compatibility with current modeling
    [cluster_labels model.desc(i,:)] = meanShift(pt_desc, 1);
    
    % Normalize descriptor
    model.desc(i,:) = model.desc(i,:) ./ norm(model.desc(i,:));
end

% Filter points with a high reprojection error
% Get average reprojection error
model.avg_err = sfm_get_error (model);
model.avg_err = model.avg_err'; % Just for consistency

% Choose points to keep
idx_valid_pts = find(model.avg_err < model.opts.reproj_th);

% Just keep idx_valid_pts in the model
model = sfm_filter_idx(model, idx_valid_pts);
