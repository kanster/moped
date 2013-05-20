% Add relevant modeling paths to the Matlab path

% Copyright: Carnegie Mellon University & Intel Corporation
% Author: Alvaro Collet (acollet@cs.cmu.edu)

% Get the path of this file
fullpath = mfilename('fullpath');

% Assume everything is relative to this path, stored in p
[p, name, ext] = fileparts(fullpath);

% Add base folder
addpath(p);

% Add meanShift folder: ./meanshift
addpath(fullfile(p, 'meanshift'));

% Add bundler folder: ./bundler/bundler
addpath(fullfile(p, 'bundler', 'bundler'));