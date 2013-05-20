function sfm_view (model, type, cameras)
% SFM_VIEW - Visualize data from a sfm model.
%
%   Usage: sfm_view(model, type, options);
%
%   Input:
%   model - SFM model to visualize
%   type - Choose what you want to see: 'pts3D', 'cam' or 'all'.
%   options - If type = {'all', 'cam'}, options is a list of camIDs that
%             specify which camera(s) to display. If not given, the default
%             if to display all cameras used in the training process.
%
%   Output:
%   -NONE-
%
% Copyright: Carnegie Mellon University & Intel Corporation
% Author: Alvaro Collet (acollet@cs.cmu.edu)

% Check arguments
error(nargchk(2, 3, nargin));

if nargin < 3, cameras = 1:model.nViews; end
% What do we want?
switch (lower(type))
    
    % Draw only the 3D points
    case { 'pts3d', '3d', 'pts' }
        plot3(model.pts3D(1,:), model.pts3D(2,:), model.pts3D(3,:), 'b+'); % Draw points
        axis equal;
        
    % Draw only the cameras
    case { 'cam' }
        hold on;
        axis equal;
        for i = cameras,
            % To use draw_camera we need the camera to world transf.
            R = rodrigues(model.cam_poses(1:3,i))';
            t = -R * model.cam_poses(4:6,i);
            draw_camera(R, t, 0.1, 'axis'); % Draw cameras
        end
        
    % Draw 3D points and cameras
    case { 'all' }   
	hold on;     
        % Get a proper scale for the cameras: the std after removing mean
        scale = mean(std(model.pts3D - repmat(mean(model.pts3D, 2), [1 size(model.pts3D,2)])));

       for i = cameras,
            % To use draw_camera we need the camera to world transf.
            R = rodrigues(model.cam_poses(1:3,i))';
            t = -R * model.cam_poses(4:6,i);
            draw_camera(R, t, scale, 'axis'); % Draw cameras
            %draw_camera(model.cam_poses(1:3,i), model.cam_poses(4:6,i), scale);
        end
        
        plot3(model.pts3D(1,:), model.pts3D(2,:), model.pts3D(3,:), 'b.'); % Draw points
        axis equal;
 
    % Oops
    otherwise
        display('Unknown option');
end
