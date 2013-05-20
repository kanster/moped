function draw_camera_2d (R, t, int_mat, scale, opt)
% DRAW_CAMERA_2D - Project a camera representation onto an image plane
%   R = id and t = [0 0 0]' show a camera in the world center pointing 
%   towards the Z axis.
%
%   Usage: draw_camera_2d (R, t, scale, int_mat, opt);
%
%   Input:
%   R - 3x3 Rotation matrix or 3x1 Rodrigues rotation vector (world to
%       camera rotation)
%   t - 3x1 Translation vector (world to camera translation)
%   int_mat - 4-by-1 vector of internal camera parameters [fx fy cx cy]
%   scale - (optional) scaling parameter. Default: 0.1.
%   opt - Visualization option: (default: 'pyr') 
%         'pyr' shows an inverted pyramid in the camera direction
%         'axis' shows the 3 axis (red for X, green for Y, blue for Z)
%
%   See also RODRIGUES
%
% Copyright: Carnegie Mellon University & Intel Corporation
% Author: Alvaro Collet (acollet@cs.cmu.edu)

% Parameter check
error(nargchk(3,5,nargin));
if nargin < 5, opt = 'pyr'; end
if nargin < 4, scale = 0.1; end

% We need a rotation vector
if numel(R) == 9,
    R = rodrigues(R);
end
cam_pose = [R(:); t(:)]; % Make sure it's a column

% Five points that define the pyramid
pyr_points = scale * [0 0 0; -0.5 -0.5 1; 0.5 -0.5 1; 0.5 0.5 1; -0.5 0.5 1; NaN NaN NaN]';

% Order in which to draw the points, so that it looks like a pyramid
% 6 is a NaN value, which means the line is not drawn
pyr_idx = [1 2 6 1 3 6 1 4 6 1 5 6 2 3 4 5 2];
pyr_colored_line = [2 3];

% Four points that define the axis in 3-space
axis_points = scale * [0 0 0; 1 0 0; 0 1 0; 0 0 1]';

% Order in which to draw the points, so that it looks like 3 axis
axis_idx_x = [1 2];
axis_idx_y = [1 3];
axis_idx_z = [1 4];
axis_colors = 'rgb';

% Make sure we leave the 'hold' property as we found it
plot_held = ishold;

switch(lower(opt))
    case {'pyr'}  
        % Rotate pyramid and plot it
        tx_pyr_2d = projectPts(pyr_points, cam_pose, int_mat, 1);
        plot(tx_pyr_2d(1, pyr_idx), tx_pyr_2d(2, pyr_idx), 'b');
        hold on;
        % Add a colored line so we see rotations
        plot(tx_pyr_2d(1, pyr_colored_line), tx_pyr_2d(2, pyr_colored_line), 'g');
    case {'axis'}
        % Rotate the 3 axis and plot them
        tx_axis_2d = projectPts(axis_points, cam_pose, int_mat, 1);
        plot(tx_axis_2d(1, axis_idx_x), tx_axis_2d(2, axis_idx_x), axis_colors(1), 'LineWidth', 3);
        hold on;
        plot(tx_axis_2d(1, axis_idx_y), tx_axis_2d(2, axis_idx_y), axis_colors(2), 'LineWidth', 3);
        plot(tx_axis_2d(1, axis_idx_z), tx_axis_2d(2, axis_idx_z), axis_colors(3), 'LineWidth', 3);
        
    otherwise
        display('Unrecognized option');
end

% Return the 'hold' property to its previous state
if ~plot_held, hold off; end


