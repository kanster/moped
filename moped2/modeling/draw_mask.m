function [locs, desc, imgfiles] = draw_mask(imgPath, reuse_mask, idx_imgs)
% DRAW_MASK - Draw masks for a set of files (i.e. choose 1 object from image)
%
%   Usage: draw_mask(imgPath, reuse_mask, idx_imgs);
%
%   Input:
%   imgPath - path to the images folder. All files within the folder will
%       be processed. Alternatively, this can be a cell array containing a
%       set of files.
%   reuse_mask - If false, an interactive screen appears to choose the 
%       Region of Interest for every image.
%       If true, every saved mask will be reused, and an interactive screen 
%       will appear only if the mask is not available. Default: true.
%   idx_imgs - (optional) specifies the indexes of images to modify.
%
%   Output:
%   -NONE-, but files are saved with .mat extension and a 'mask' variable.
%
% Copyright 2011 Carnegie Mellon University and Intel Corporation
% Author: Alvaro Collet
% acollet@cs.cmu.edu

% Check arguments
error(nargchk(1, 3, nargin));
if nargin < 3, idx_imgs = []; end
if nargin < 2, reuse_mask = true; end

% Image path or array of images?
if ~iscell(imgPath),
    imgfiles = dir(fullfile(imgPath, '*.jpg'));
    imgfiles = {imgfiles.name};
else
    % We already have the cell array of images
    imgfiles = imgPath;
    imgPath = [];
end
nFiles = length(imgfiles);

% If the image list is empty, it means 'all'
if numel(idx_imgs) == 0, idx_imgs = 1:nFiles; end

for i = idx_imgs,
    display(['Processing image ' num2str(i)]);
    img = imread(fullfile(imgPath, imgfiles{i}));
    
    if reuse_mask && exist(fullfile(imgPath, [imgfiles{i}(1:end-4) '.mat']), 'file'),
        % We don't have to extract the mask again
        continue;
    end

    % Create a mask for the region of interest.
    mask = roipoly(img);
          
    % Save mask
    save(fullfile(imgPath, [imgfiles{i}(1:end-4) '.mat']), 'mask');
end
