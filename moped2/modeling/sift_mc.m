function [cimgs, Points] = sift_mc(imgs, cam, undistort, USE_FASTSIFT)
% SIFT2 - Extract SIFT features and keypoints from an image
%
%   Usage: cimgs = sift_mc(imgs, cam, undistort, USE_FASTSIFT);
%
%   Input:
%   imgs - N-by-1 cell array of input images. Grayscale images in double
%          format (pixel range 0..1) are preferred, but uint8 as well as 
%          RGB are also supported.
%   cam - N-by-1 struct of camera parameters for each camera, as defined
%          in or_cam.m
%   undistort{0} - (optional) if 1, undistort images according to their
%                  intrinsic parameters. If 0, images need to have been
%                  previously undistorted.
%   USE_FASTSIFT{1} - (Optional) if 1, use the libsiftfast library. If 0, 
%                     use regular sift.
%
%   Output:
%   cimgs - N-by-1 struct of output calibrated Images and keypoints. Each
%           item of the structure contains:
%       .img - Input image, potentially undistorted
%       .locs - 4-by-K matrix, in which each col has the 4 values for a
%               keypoint location (X, Y, scale, orientation).  The 
%               orientation is in the range [-PI, PI] radians.
%       .desc - a 128-by-K matrix, where each row gives an invariant
%               descriptor for one of the K keypoints. The descriptor is a 
%               vector of 128 values normalized to unit length.
%       .idxCam - 1-by-K array that contains the camera to which each point
%                 belongs.
%       .cam - struct of camera parameters for the corresponding image
%   Points - (Optional) N-by-1 struct of output keypoints. Each Points(i)
%            contains:
%       .locs - 4-by-1 keypoint location (X, Y, scale, orientation).
%       .desc - 128-by-1 SIFT descriptor
%       .idxCam - Index that specifies which camera this point belongs to. 
%       
%   USEFUL TIPS:
%   Get all locs from all images: all_locs = [cimgs.locs];
%   Get all descriptors from all images: all_desc = [cimgs.desc];
%   Get a list of indexes that specify which keypoint belongs to which
%       camera: cam_idxs = [cimgs.idxCam]; 
%
% Alvaro Collet
% acollet@cs.cmu.edu

% Default parameters
if nargin < 4, USE_FASTSIFT = true; end
if nargin < 3, undistort = false; end

% FAST sift or regular (more stable) sift?
USE_FASTSIFT = false;
if ~iscell(imgs), imgs = {imgs}; end

for i = 1:length(imgs),

    if undistort,
        cimgs(i).img = imundistort(imgs{i}, cam(i).KK, cam(i).dist);
        img = cimgs(i).img;
    else
        cimgs(i).img = imgs{i};
        img = cimgs(i).img;
    end
    
    if USE_FASTSIFT,
        % Check if image is grayscale or color
        if size(img,3) == 3,
            img = rgb2gray(img);
        end

        % Check if image is double or not
        if ~isa(img, 'double'),
            img = im2double(img);
        end

        % Run SIFT
        [locs, descriptors] = siftfast(img);

        % Why in the world do we get NaNs?
        descriptors(isnan(descriptors)) = 0;
    else
        % USE REGULAR SIFT
        imwrite(img, 'temp_img.pgm');
        img = 'temp_img.pgm';

        % Get SIFT keypoints
        [img, descriptors, locs] = sift(img);
        delete temp_img.pgm;

        locs = locs';
        descriptors = descriptors';
        locs([1 2], :) = locs([2 1], :);
    end
    
    % Fill out output struct
    cimgs(i).locs = locs;
    cimgs(i).desc = descriptors;
    cimgs(i).idxCam = i * ones(1, size(locs, 2));
    cimgs(i).cam = cam(i);
end

% Extract a secondary Points structure (it's slow to create, but pretty
% useful!)
if nargout > 1, 
    % Matlab can't convert a matrix to a struct directly, so we have to go
    % through a cell first
    AuxMat = [[cimgs.locs]; [cimgs.desc]; [cimgs.idxCam]];
    Points = mat2struct(AuxMat, {'locs', 'desc', 'idxCam'}, 1, [4 128 1], ones(size(AuxMat, 2), 1));
end