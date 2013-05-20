function [locs, descriptors] = sift2(img, switch_lowe, USE_FASTSIFT)
% SIFT2 - Extract SIFT features and keypoints from an image
%
%   Usage: [locs, descriptors] = sift2(img);
%          [locs, descriptors] = sift2(img, switch_lowe);
%          [locs, descriptors] = sift2(img, switch_lowe, USE_FASTSIFT);
%
%   Input:
%   img - Input image. Grayscale images in double format (pixel range 0..1)
%         are preferred, but uint8 as well as RGB are also supported 
%   switch_lowe {0} - (Optional) if 1, the keypoint locations are switched 
%                   from [X Y] to [row column], and both outputs (locs, 
%                   descriptors) are transposed. (Default: 0)
%   USE_FASTSIFT - (Optional) if 1, use the libsiftfast library. If 0, use
%                  regular sift.
%
%   Output:
%   locs: 4-by-K matrix, in which each row has the 4 values for a
%         keypoint location (X, Y, scale, orientation).  The 
%         orientation is in the range [-PI, PI] radians.
%   descriptors: a 128-by-K matrix, where each row gives an invariant
%         descriptor for one of the K keypoints.  The descriptor is a vector
%         of 128 values normalized to unit length.
%
% Alvaro Collet
% acollet@cs.cmu.edu

% Default parameters
if nargin < 3, USE_FASTSIFT = true; end
if nargin < 2, switch_lowe = 0; end

% FAST sift or regular (more stable) sift?
% USE_FASTSIFT = true;

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

    % Switch [X Y] to [row column] ?
    if switch_lowe,
        locs = locs';
        descriptors = descriptors';
        locs(:, [1 2]) = locs(:, [2 1]);
        
        % Why in the world do we get NaNs?
        descriptors(isnan(descriptors)) = 0;
    end
else
    % USE REGULAR SIFT
    imwrite(img, 'temp_img.pgm');
    img = 'temp_img.pgm';

    % Get SIFT keypoints
    [img, descriptors, locs] = sift(img);
    delete temp_img.pgm;
end