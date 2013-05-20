function success = default(var_name, value)
% DEFAULT - Easily assign default arguments if var is empty or not defined.
%   The variable var_name will be created in the CALLER workspace (i.e. the
%   function calling 'default').
%   This command is *exactly* equivalent to:
%       if ~exist('var_name', 'var') || isempty(var_name), 
%            var_name = value;
%       end
%
%   Usage: success = default('var_name', value)
%
%   Input:
%   'var_name' - String containing the variable you want to create a
%                default for. If the variable is empty or not defined,
%                'expression' will be evaluated in the CALLER's workspace.
%   value - Value to assign to var_name for initialization.
%
%   Output:
%   success - true if the variable has been set with its default value,
%             false otherwise.
%          
%   Example:
%       MANUAL WAY:
%           if ~exist('sigma', 'var') || isempty(sigma), sigma = 1; end
%       ALTERNATIVE WAY:
%           default('sigma', 1);
%   Example 2:
%       MANUAL WAY:
%           if ~exist('mat', 'var') || isempty(mat), mat = magic(30); end
%       ALTERNATIVE WAY:
%           default('mat', magic(30));
%
% Copyright: Carnegie Mellon University & Intel Corporation
% Author: Alvaro Collet (acollet@cs.cmu.edu)

% Condition
str = ['~exist(''' var_name ''', ''var'') || isempty(' var_name ')'];
Cond = evalin('caller', str);

% Assign default value if Condition is met
if Cond,
    assignin('caller', var_name, value);
    success = true;
else
    success = false;
end
