function model = sfm_bundler_book(name, front_image_file, back_image_file, ...
    spine_image_file, real_size, output_file)
% SFM_BUNDLER_BOOK - Create SFM model using 3 planar images
%
%   Usage: model = sfm_bundler_book(name, front_image, back_iamge, spine_image,
%              real_size, output_file)
%
%   Input:
%   name - Model name
%   *_image_file - Paths to input images
%   real_size - Real size [x y z] of the book, in meters. 
%   output_file - Output file to export the model (.moped.xml). If not
%                 given, the model won't be exported.
%
%   Output:
%   model - SFM model, as defined in SFM_MODEL.M
%
% Copyright: Carnegie Mellon University
% Authors: Alvaro Collet (acollet@cs.cmu.edu)
%          Tom Mullins (tmullins@andrew.cmu.edu)

    % Check inputs
    if nargin < 6, export = false; else export = true; end

    if size(real_size, 1) == 1, real_size = real_size'; end

    % Rotations done on each
    front_rot = [ 1  0  0;  0  0  1;  0 -1  0];
    back_rot =  [-1  0  0;  0  0 -1;  0 -1  0];
    spine_rot = [ 0  0  1; -1  0  0;  0 -1  0];

    % Translations done on each (after rotations above)
    front_trl = [-.5; -.5;  .5] .* real_size;
    back_trl =  [ .5;  .5;  .5] .* real_size;
    spine_trl = [-.5;  .5;  .5] .* real_size;

    model = sfm_model();

    model.name = name;
    model.version = 'Bundler v1';

    [front_pts, front_desc] = sift_transformed(front_image_file, real_size, ...
        front_rot, front_trl);
    [back_pts, back_desc] = sift_transformed(back_image_file, real_size, ...
        back_rot, back_trl);
    [spine_pts, spine_desc] = sift_transformed(spine_image_file, real_size, ...
        spine_rot, spine_trl);

    model.pts3D = [front_pts, back_pts, spine_pts];
    model.desc = [front_desc; back_desc; spine_desc];
    nPts = size(model.pts3D, 2);

    % Fill up other necessary data
    model.num_views = ones(1, nPts);
    model.avg_err = ones(1, nPts);
    model.color3D = ones(3, nPts);

    if export,
        sfm_export_xml(output_file, model);
    end

end

function [pts3D, desc] = sift_transformed(image_file, real_size, rot, transl)

    fprintf('Extracting SIFT features from %s...\n', image_file);
    [image, desc, locs] = sift(image_file);

    % Find real sizes in image frame
    real_size = abs(inv(rot) * real_size);

    % Assuming the image is the correct propotion, we can just use X scale
    % (But X and Y are swapped in the image)
    m_per_px = real_size(1) / size(image, 2);
    fprintf('Meters per pixel: %f\n', m_per_px);
    m_per_px_Y = real_size(2) / size(image, 1);
    if abs(m_per_px - m_per_px_Y) / m_per_px > .02
        fprintf('Warning: X scaling (%f) and Y scaling (%f) are different\n',...
            m_per_px, m_per_px_Y);
    end

    % Create 3D points from 2D features
    % We put +Z pointing UP (towards us), Y towards the long edge, X in the
    % short edge.
    nPts = size(locs,1);
    pts3D = zeros(3, nPts);
    pts3D([1 2],:) = locs(:,[2 1])' .* m_per_px;

    % Do transform for this plane on all points
    pts3D = rot* pts3D;
    pts3D = bsxfun(@plus, pts3D, transl);

end
