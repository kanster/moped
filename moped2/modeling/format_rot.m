function varargout = format_rot(varargin)
% FORMAT_ROT - Convert rotation + tx matrices into vectors and viceversa.
%   This function is useful to change the expression of rigid
%   transformations. It accepts the following formats both as input and
%   output: 6-by-1 (rodrigues + tx), R and T, quaternion and T, 3-by-4, and
%   4-by-4. It performs conversions between all of them.
% 
%   Usage:  output(s) = format_rot(output_type, param1, param2)
% 
%   Input:
%   output_type - It can be the following options:
%       '3x4' - Output is [R T].
%       '4x4' - Output is [R T; 0 0 0 1]
%       'RT'  - Output is R and T (two separate parameters)
%       'rod' - Output is [r1 r2 r3 tx ty tz], where [r1 r2 r3] is a
%               Rodrigues rotation vector.
%       'quat' - Output is [q1 q2 q3 q4] and (if provided) tx ty tz (7x1)
% 
%       NOTE: Adding an 'i' in front of output_type returns the INVERSE
%       TRANSFORMATION of the input. E.g.:
%       [Ri, Ti] = format_rot('iRT', rod_pose)
%       [R, T] = format_rot('RT', rod_pose)
%       The outputs of these functions relate as: Ri = R'; Ti = -R'*T;
% 
%   param1 - It can be either a 3x1 rotation vector, a 3x3 rotation matrix,
%            a 4x1 quaternion, a 3x4 projection matrix or a 4x4 projective 
%            matrix.
%   param2 - (optional) 3x1 translation vector [tx ty tz].
% 
%   Output:
%   out1, out2 - Output parameters after conversion, as requested with
%                'output_type'.
% 
%   Examples:
%           [rodrigues_vec] = format_rot('rod', R, T);
%           [R, T] = format_rot('RT', rodrigues_vec);
%           P = format_rot('3x4', rodrigues_vec); % P is 3-by-4
%           quat = format_rot('quat', R);
%           [quat, T] = format_rot('quat', P); % P is 3-by-4
%           (...)
%   
% Copyright: Carnegie Mellon University & Intel Corporation
% Author: Alvaro Collet (acollet@cs.cmu.edu)

% Parse input arguments
if nargin == 3, 
    use_tx = true; 
    param2 = varargin{3};
else
    use_tx = false; 
end
output_type = varargin{1};
param1 = varargin{2};

%% Transform any kind of input parameter to R, T --------------------------
switch (numel(param1)),
    
    case 3, % Rotation vector
        R = rodrigues(param1);
        if use_tx,
            T = param2(:);
        end
        
    case 4, % quaternion
        R = quat2rot(param1);
        if use_tx,
            T = param2(:);
        end

    case 6, % Rotation vector + tx
        R = rodrigues(param1(1:3));
        T = param1(4:6);
        T = T(:);

    case 7, % quaternion + tx
        R = quat2rot(param1(1:4));
        T = param1(5:7);
        T = T(:);

    case 9, % Rotation matrix
        R = param1;
        if use_tx,
            T = param2(:);
        end
        
    case { 12 } % 3x4 projection matrix

        R = param1(1:9);
        R = reshape(R, [3 3]);
        T = param1(10:12);
        T = T(:);
        
    case { 16 } % 4x4 projection matrix

        R = param1([1:3 5:7 9:11]);
        R = reshape(R, [3 3]);       
        T = param1(13:15);
        T = T(:);
        
    otherwise
        error('Unknown input types');
end

%% Optional inversion of the transformation -------------------------------
if lower(output_type(1)) == 'i',
    R = R';
    T = -R*T;
    output_type = output_type(2:end);
end
    
%% Transform input parameters into output parameters ----------------------
switch lower(output_type),
    case 'rod', % Output is [r1 r2 r3 tx ty tz]
        rod = rodrigues(R);
        if exist('T', 'var'), 
            varargout(1) = {[rod(:); T]};
        else
            varargout(1) = rod(:);
        end
        
    case 'quat', % Quaternion and T
        quat = rot2quat(R);
        varargout(1) = {quat};
        
        if exist('T', 'var'),
            if nargout == 2,
                varargout(2) = {T};
            else
                varargout(1) = {[quat(:); T]};
            end
        end
                
    case 'rt', % 3x3 Rotation matrix and T
        varargout(1) = {R};
        if nargout == 2,
            varargout(2) = {T};
        end
        
    case '3x4', % 3x4 projection matrix
        varargout(1) = {[R T]};
        
    case '4x4', % 4x4 projection matrix
        varargout(1) = {[R T; 0 0 0 1]};
        
    otherwise
        error('Unknown output type');
end
