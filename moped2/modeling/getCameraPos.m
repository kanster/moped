function cam_pose = getCameraPos(pts2D, pts3D, K, init_R, init_T)
% GETCAMERAPOS - Find camera position from a set of 2D-3D correspondences.
%
%   Usage: getCameraPos(pts2D, pts3D, K, init_R, init_T);
%
%   Input:
%   pts2D - 2-by-N array of 2D positions (in pixels)
%   pts3D - 3-by-N array of 3D positions (in world coordinates), each 
%           column corresponds to the pixel position column in pts2D.
%   K - 4-by-1 vector of camera calibration parameters [fx fy cx cy]
%   init_R - 3-by-3 rotation matrix for the initial camera pose.
%            Alternatively, it is also accepted a 6-by-1 pose [r1 r2 r3 tx
%            ty tz], where [r1 r2 r3] is a Rodrigues rotation vector.
%   init_T - 3-by-1 translation vector for the initial camera pose
%
%   Output:
%   cam_pos - 6-by-1 world-to-camera pose [r1 r2 r3 tx ty tz], where 
%             [r1 r2 r3] is a Rodrigues rotation vector.
%
% Alvaro Collet
% acollet@cs.cmu.edu

% Check arguments
if nargin < 5, init_T = [0 0 0.5]; end
if nargin < 4, init_R = [0 -1 0; 0 0 -1; 1 0 0]; end % Camera pointing down

% Get initial pose in compact form
if numel(init_R) == 5,
    init_cam_pose = init_R;
else
    % Do this in two steps to ensure init_cam_pose is a column vector
    init_cam_pose = rodrigues(init_R);
    init_cam_pose = [init_cam_pose(:); init_T(:)];
end
    
% % Apply calibration parameters -> pts2D_cam is in camera coordinates
% pts2D_cam = zeros(size(pts2D));
% pts2D_cam(1,:) = pts2D(1,:)/K(1) - K(3)/K(1);
% pts2D_cam(2,:) = pts2D(2,:)/K(2) - K(4)/K(2);
% 
% % pts2D = K[R T] * pts3D -> let's build A, b to get linear sol. Ax = b
% nPts = size(pts3D,2);
% A = zeros(nPts*3, 12);
% A(1:3:end, :) = [pts3D' zeros(nPts, 6) ones(nPts, 1) zeros(nPts, 2)];
% A(2:3:end, :) = [zeros(nPts, 3) pts3D' zeros(nPts, 4) ones(nPts, 1) zeros(nPts, 1)];
% A(3:3:end, :) = [zeros(nPts, 6) pts3D' zeros(nPts, 2) ones(nPts, 1)];
% 
% b = ones(nPts*3, 1);
% b(1:3:end) = pts2D_cam(1,:);
% b(2:3:end) = pts2D_cam(2,:);
% 
% % Now compute the linear solution
% RT = pinv(A)*b;
% R = reshape(RT(1:9), [3 3]);
% T = RT(10:12);
% 
% % Enforce R constraint using SVD: R'*R = R*R' = Id
% [u,s,v] = svd(R);
% R = u*v';
% 
% % Build compact 6-by-1 vector from the linear solution
% init_cam_pose = format_rot('rod', R, T);

init_cam_pose = format_rot('rod', init_R, init_T);

% Now go for nonlinear minimization
optim_opts = optimset('fminunc');
optim_opts.MaxIters = 10000;
optim_opts.MaxFunEvals = 10000;
cam_pose = fminunc(@(vec) perspectiveCost(vec, pts2D, pts3D, K), init_cam_pose, optim_opts);

% We want world to cam transformation
[R,T] = format_rot('RT', cam_pose);
cam_pose = format_rot('rod', R', -R'*T);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function score = perspectiveCost(cam_pose, pts2D, pts3D, KK)
% Cost function to optimize with LM (perspective projection).

pts2D_proj = projectPts(pts3D, cam_pose, KK, 1);
score = sum(sqrt(sum((pts2D - pts2D_proj).^2)));
fprintf('Total Score: %.3d. Average Error: %.3d\n', score, score/size(pts2D, 2));
