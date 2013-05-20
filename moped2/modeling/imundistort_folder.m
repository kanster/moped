function imundistort_folder(img_dir, cam_K, cam_dist, output_size)
% IMUNDISTORT_FOLDER - Undistort a whole folder of images (and masks)
%
%   Usage:  imundistort_folder(img_dir, cam_K, cam_dist, output_size)
%
%   Input:
%   img_dir - Folder where images are contained
%   cam_K - 4-by-1 internal camera parameters, [fx fy cx cy]
%   cam_dist - 5-by-1 Camera distortion parameters (Bouguet's tool)
%   output_size - (Optional) Resize images (and masks) when saving them,
%                 AFTER undistorting them. E.g. [480 640]
%
%   Output:
%   -NONE-
%
% USEFUL TIP: If you want to rescale a set of images and masks without
% undistorting them, you can use:
%    imundistort_folder(img_dir, cam_K, zeros(5,1), output_size)
%
% Copyright: Carnegie Mellon University & Intel Corporation
% Author: Alvaro Collet (acollet@cs.cmu.edu)

if nargin < 4, output_size = []; end
if nargin < 3, cam_dist = []; end

% Check if we received a list of files instead
if ~iscell(img_dir),
    files = dir(fullfile(img_dir, '*.jpg'));
    files = {files.name};
    new_files = files;
    for j = 1:length(files),
        new_files{j} = fullfile(img_dir, 'original', files{j});
        files{j} = fullfile(img_dir, files{j});
    end
else
    files = img_dir;
    img_dir = fileparts(files{1});
    new_files = files;
    for j = 1:length(files),
        [img_path, f, ext] = fileparts(files{j});
        new_files{j} = fullfile(img_path, 'original', [f ext]);
    end
end
if ~exist(fullfile(img_dir, 'original'), 'file'), mkdir(fullfile(img_dir, 'original')); end

for i = 1:length(files),
    display(['Undistorting ' files{i} '...']);
    
    % Copy file to 'original'
    if ~exist(new_files{i}, 'file'),
        copyfile(files{i}, new_files{i});
    end
    
    % Undistort
    img = imread(files{i});
    if ~isempty(cam_dist),
        img = imundistort(img, cam_K, cam_dist);
    end
    
    if ~isempty(output_size),
        img = imresize(img, output_size);
    end
    
    % Write again
    imwrite(img, files{i});
    
    % Is there an associated mask?
    if exist([files{i}(1:end-4) '.mat'], 'file'),
        display(['Undistorting ' [files{i}(1:end-4) '.mat'] '...']);
        
        % Copy file to 'original'
        if ~exist([new_files{i}(1:end-4) '.mat'], 'file'),
            copyfile([files{i}(1:end-4) '.mat'], [new_files{i}(1:end-4) '.mat']);
        end
        load ([files{i}(1:end-4) '.mat'], 'mask');
        
        % Undistort
        mask = uint8(mask);
        if ~isempty(cam_dist),
            mask = imundistort(repmat(mask, [1 1 3]), cam_K, cam_dist);
        end
        if ~isempty(output_size),
            mask = imresize(mask, output_size);
        end
        
        % Convert to logical
        mask = mask(:,:,1) > 0.5;
        
        % Rewrite
        save([files{i}(1:end-4) '.mat'], 'mask');
    end
end
