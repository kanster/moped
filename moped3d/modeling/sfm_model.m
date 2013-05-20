function model = sfm_model()
% SFM_MODEL - Create a structrure-from-motion model with empty fields
%
%   Usage: model = sfm_model();
%
%   Output:
%   model - Structure that contains all output data after running an sfm
%           algorithm. The following fields are used:
%       .name - Object name
%       .version - Model structure version, e.g. "Bundler v1"
%       .nViews - Number of views K used in the creation of this model.
%       .pts3D - 3-by-M array of M 3D points resulting of the SFM algorithm
%       .cam_poses - 6-by-K array of 6DOF camera poses [r1 r2 r3 t1 t2 t3]'
%                    where [r1 r2 r3] is a rodrigues rotation vector. This
%                    is a WORLD TO CAMERA transformation: x = K[R t]X.
%       .desc - T-by-128  matrix that contains N SIFT descriptors, one for 
%               each point in pts3D.
%       .int_mat - internal camera parameters: [fx fy px py].
%       .pts2D - 2-by-M-by-K array of M 2D points [X Y] in K views. A 
%                correspondence in views I and J is shown when the points 
%                (:, k, I) and (:, k, J) both are non-zero. A value
%                of [0 0] means the point is not present in that view.
%       .opts - Options structure that specifies how this model was
%               created (see SFM_OPTIONS).
%       .or_xml - OpenRAVE XML file associated with this model
%       .or_transf - 3-by-4 transformation between SFM and openRAVE models
%       .or_name - Object name in OpenRAVE
%
% Copyright: Carnegie Mellon University & Intel Corporation
% Author: Alvaro Collet (acollet@cs.cmu.edu)

% General info ------------------------------------------------------------
% Model name
model.name = '';

% Generator of the model (usually Bundler)
model.version = '';

% Output model ------------------------------------------------------------
% 3-by-N array of 3D points resulting of the SFM algorithm
model.pts3D = [];

% T-by-128 matrix that contains T SIFT descriptors, one or more for each 
% point in pts3D.
model.desc = [];

% 6-by-K array of 6DOF camera poses [r1 r2 r3 t1 t2 t3]'. where [r1 r2 r3] 
% is a rodrigues rotation vector. This is a WORLD TO CAMERA transformation, 
% i.e.: p = K[R t]P.
model.cam_poses = [];

% Internal camera parameters: [fx fy px py].
model.int_mat = [];

% Additional model information --------------------------------------------
% 2-by-M-by-K array of M 2D points [X Y] in K views. A correspondence in 
% views I and J is shown when the points (:, k, I) and (:, k, J) both are 
% non-zero. A value of [0 0] means the point is not present in that view.
model.pts2D = [];

% Model creation parameters -----------------------------------------------
% Options structure that specifies how this model was created (see SFM_OPTIONS)
model.opts = [];

% Number of views K used in the creation of this model.
model.nViews = [];

% Bundler extra parameters ------------------------------------------------
% 3-by-N array of RGB colors of each 3D point
model.color3D = [];

% N-by-M matrix of 2D-3D correspondences as read from Bundler (unnecessary)
model.keys = [];

% 1-by-N Number of images in which each 3D point appears
model.num_views = [];

% 1-by-N structure with extra info about all 2D points and descriptors 
% found for each 3D point (unnecessary for recognition, for now)
model.pt_info = [];

% 1-by-N array of average reprojection error for each 3D point
model.avg_err = [];

% K-by-1 array with the number of good 2D points that each camera has
model.cam_nPoints = [];

% OpenRAVE parameters -----------------------------------------------------
% XML file associated with this model
model.or_xml = '';

% 3-by-4 transformation between SFM and openRAVE models
model.or_transf = [eye(3) [0 0 0]'];

% Object name
model.or_name = '';
