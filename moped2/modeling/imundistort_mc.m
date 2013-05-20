function imundistort_mc(img_dir, cams, patterns)
% IMUNDISTORT_MC - Undistort an image folder with multi cams information.
%   This function will look for a different pattern for each camera (i.e.
%   patterns{i} <--> cam(i)) and undistort the images that match. Only JPG
%   images are considered for now.
%
%   Usage:  imundistort_mc(img_dir, cams, patterns)
%
%   Input:
%   img_dir - Folder where images are contained
%   cams - N-by-1 Camera structure, as defined in or_cam.m
%   patterns - N-by-1 cell array of string patterns. E.g. if files to be
%              undistorted with cams(1) start with 'img_01_' and the images
%              undistorted with cams(2) start with 'img_02_', then
%              patterns = {'img_01_', 'img_02'}.
%
%   Output:
%   -NONE-
%
% Copyright: Carnegie Mellon University & Intel Corporation
% Author: Alvaro Collet (acollet@cs.cmu.edu)

error(nargchk(3, 3, nargin));
if numel(cams) ~= numel(patterns),
    error('Number of elements in cams and patterns must match');
end

for i = 1:numel(cams),
    files = dir(fullfile(img_dir, ['*' patterns{i} '*']));
    files = {files.name};
    for j = 1:length(files),
        files{j} = fullfile(img_dir, files{j});
    end
    
    % Add full path
    imundistort_folder(files, cams(i).KK, cams(i).dist);
end
