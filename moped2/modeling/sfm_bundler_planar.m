function model = sfm_bundler_planar(name, image_file, output_file, real_size)
% SFM_BUNDLER - Create SFM model using a single planar image
%
%   Usage: model = sfm_bundler(name, image, output_file, real_size)
%
%   Input:
%   name - Model name
%   image_file - Path to input image
%   output_file - Output file to export the model (.moped.xml). If not
%   given, the model won't be exported.
%   real_size - Real size of LONG EDGE of image, in pixels per meter. 
%     Default: 8000
%
%   Output:
%   model - SFM model, as defined in SFM_MODEL.M
%
% Copyright: Carnegie Mellon University
% Author: Alvaro Collet (acollet@cs.cmu.edu)

% Check inputs
if nargin < 4, real_size = 8000; end
if nargin < 3, export = false; else export = true; end

display('Extracting SIFT features from image...');
[image, desc, locs] = sift(image_file);

model = sfm_model();

% Create 3D points from 2D features
% We put +Z pointing UP (towards us), Y towards the long edge, X in the
% short edge.
nPts = size(locs,1);
model.pts3D = zeros(3, nPts);
model.pts3D(1:2,:) = locs(:,2:-1:1)' / real_size;

model.name = name;
model.version = 'Bundler v1';

% Add descriptors to the model
model.desc = desc;

% Move pts3D to be (roughly) in the world center
mu = mean(model.pts3D, 2);
model = sfm_transform_ext(model, eye(3), -mu, 1);

% Fill up other necessary data
model.num_views = ones(1, nPts);
model.avg_err = ones(1, nPts);
model.color3D = ones(3, nPts);

if export,
    sfm_export_xml(output_file, model);
end
