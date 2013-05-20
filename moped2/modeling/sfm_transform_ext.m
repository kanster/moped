function model = sfm_transform_ext (model, R, t, scale)
% SFM_TRANSFORM_EXT - Apply a rigid 3D transformation to a SFM model.
%   This function is useful to align an SFM model (arbitrary axes) so that
%   it corresponds to a known scale and axis. The fields: 'pts3D' and
%   'cam_poses' are updated.
%
%   Usage: new_model = sfm_transform_ext (model, R, t, scale);
%          new_model = sfm_transform_ext (model, [R*scale t]);
%   Input:
%   model - SFM model structure we wish to apply the transformation on
%   R - 3x3 Rotation matrix or 3x1 Rodrigues rotation vector that rotates
%           from the model coord frame to the desired coord frame.
%           Alternatively, if only 2 parameters are given, 3-by-4 or 4-by-4 
%           transformation matrices are also accepted.
%   t - 3x1 Translation vector (model to world translation)
%   scale - Scaling parameter. Default: 1.
%
%   Output:
%   model - Model after applying the 3D transformation to the 3d cloud.
%
%   Alternatively, if only 2 parameters are given, 3-by-4 or 4-by-4 
%   transformation matrices are also valid 
%
%   See also RODRIGUES, SFM_ALIGNMENT_GUI
%
% Copyright: Carnegie Mellon University & Intel Corporation
% Author: Alvaro Collet (acollet@cs.cmu.edu)

% Parameter check
error(nargchk(2,4,nargin));

% Build 4-by-4 projection matrix from args --------------------------------
% Case 1: We already receive P
if nargin == 2,
    % We want a 4-by-4
    if size(R, 1) == 3,
        Proj = [R; 0 0 0 1];
    else
        Proj = R;
        InvProj = inv(R);
    end
else
    % Case 2: We receive R, t, scale
    if nargin == 4,
        
        % We need a rotation matrix, not a vector
        if numel(R) == 3,
            R = rodrigues(R);
        end
        
        % P = [R*s t]
        Proj = [R*scale t(:)*scale(:); 0 0 0 1];
        InvProj = [R'.*scale -R'*t(:).*scale; 0 0 0 scale];
    end
end
    
% Apply transformation to pts3D -------------------------------------------
if ~isempty(model.pts3D),
    pts3D = [model.pts3D; ones(1, size(model.pts3D, 2))];
    pts3D = Proj * pts3D;
    model.pts3D = pts3D(1:3, :);
end

% Apply transformation to cameras -----------------------------------------
% Camera poses are stored using camera-to-world transformations, we need to
% invert the projection matrix for this to work -> use InvProj
% Proj(1:3, 1:3) = Proj(1:3, 1:3)';
% Proj(1:3, 4) = -Proj(1:3, 1:3) * Proj(1:3, 4);

cposes = model.cam_poses;
for i = 1:size(cposes, 2),
    % Extract rotation matrix
    cam_rot = rodrigues(cposes(1:3, i));

    % Get camera's projection matrix
    p_cam = [cam_rot cposes(4:6, i); 0 0 0 1];
    
    % New camera projection matrix
    new_p_cam = p_cam * InvProj;
    
    % Make sure it's a true rotation!
    [u, s, v] = svd(new_p_cam(1:3, 1:3));
    cposes(1:3, i) = rodrigues(u*v');
    cposes(4:6, i) = new_p_cam(1:3, 4);
end

model.cam_poses = cposes;

