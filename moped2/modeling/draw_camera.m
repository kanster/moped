function draw_camera (R, t, scale, opt)
% DRAW_CAMERA - Draw a camera in world coords according to its rotation
%   and translation. R = id and t = [0 0 0]' show a camera in the world 
%   center pointing towards the Z axis.
%
%   Usage: draw_camera (R, t, scale, opt);
%
%   Input:
%   R - 3x3 Rotation matrix or 3x1 Rodrigues rotation vector (camera to
%       world rotation)
%   t - 3x1 Translation vector (camera to world translation)
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
error(nargchk(2,4,nargin));
if nargin < 4, opt = 'pyr'; end
if nargin < 3, scale = 0.1; end

% We need a rotation matrix, not a vector
if numel(R) == 3,
    R = rodrigues(R);
end
t = t(:); % Make sure it's a column

% Five points that define the pyramid
pyr_points = scale * [0 0 0; -0.5 -0.5 1; 0.5 -0.5 1; 0.5 0.5 1; -0.5 0.5 1; NaN NaN NaN]';

% Order in which to draw the points, so that it looks like a pyramid
% 6 is a NaN value, which means the line is not drawn
pyr_idx = [1 2 6 1 3 6 1 4 6 1 5 6 2 3 4 5 2];
pyr_colored_line = [2 3];
pyr_rect = [2 3 4 5];

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
        tx_pyr = R*pyr_points + repmat(t, [1 size(pyr_points,2)]);
        plot3(tx_pyr(1, pyr_idx), tx_pyr(2, pyr_idx), tx_pyr(3, pyr_idx), 'b');
        hold on;
        % Add a colored line so we see rotations
        plot3(tx_pyr(1, pyr_colored_line), tx_pyr(2, pyr_colored_line), tx_pyr(3, pyr_colored_line), 'g');
        fill3(tx_pyr(1, pyr_rect), tx_pyr(2, pyr_rect), tx_pyr(3, pyr_rect), 'g');
    case {'axis'}
        % Rotate the 3 axis and plot them
        tx_axis = R*axis_points + repmat(t, [1 size(axis_points,2)]);
        plot3(tx_axis(1, axis_idx_x), tx_axis(2, axis_idx_x), tx_axis(3, axis_idx_x), axis_colors(1), 'LineWidth', 3);
        hold on;
        plot3(tx_axis(1, axis_idx_y), tx_axis(2, axis_idx_y), tx_axis(3, axis_idx_y), axis_colors(2), 'LineWidth', 3);
        plot3(tx_axis(1, axis_idx_z), tx_axis(2, axis_idx_z), tx_axis(3, axis_idx_z), axis_colors(3), 'LineWidth', 3);
        
    otherwise
        display('Unrecognized option');
end

% Return the 'hold' property to its previous state
if ~plot_held, hold off; end


