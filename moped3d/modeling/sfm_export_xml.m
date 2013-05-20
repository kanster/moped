function sfm_export_xml (filename, model, full_export, wt_append)
% SFM_EXPORT_MODEL - Export SFM model to file in XML format
%
%   Usage: sfm_export_xml(filename, model, full_export, 'w')
%
%   Input:
%   filename - Text file to write to.
%   model - SFM model to be exported.
%   full_export - Export EVERYTHING from a model, or just the most
%                 important data (i.e. OpenRAVE, pts3D and cam_poses).
%                 Default: false
%   wt_append - 'WRITE' or 'APPEND'. 'WRITE' will overwrite the file if it
%             already exists, while 'APPEND' will add to it. 
%             Default: 'WRITE'
%
%   Output:
%   -NONE- Just the output file.
%
%   File format:
%   <Model name="" version="Bundler v0.3"> % version defines the model creator 
%       <Openrave>
%           <name>"Openrave name"</name>
%           <xml>"Openrave model xml filename"</xml>
%           <transf>R(1,1); R(2,1); R(3,1); R(1,2); ... R(3,3); T(1); T(2);
%                   T(3)</transf> % Coord frame transformation in Openrave
%       </Openrave>
%       <Points>
%           <Point p3d="x;y;z" nviews="" avg_err="" color="R;G;B" desc_type="SIFT" desc="a;b;c;...">
%               <Observation camera_id="n" desc_type="SIFT" loc="x;y;scale;orientation" desc="a;b;c;...">
%               <Observation ...>
%               <Observation ...>
%           <Point p3d="x;y;z" nviews="" avg_err="" desc_type="SIFT" desc="a;b;c;...">
%               <Observation camera_id="n" desc_type="SIFT" loc="x;y;scale;orientation" desc="a;b;c;...">
%               <Observation ...>
%               <Observation ...>
%           ...
%       </Points>
%       <Cameras K="fx; fy; cx; cy;">
%           <Camera id="n" rot_type="quat" rot="w;x;y;z" tx="T(1);T(2);T(3)">
%           ...
%       </Cameras>
%   </Model>
%
% Copyright: Carnegie Mellon University & Intel Corporation
% Author: Alvaro Collet (acollet@cs.cmu.edu)

if nargin < 4, 
    opt = 'wt';
else
    if lower(wt_append(1)) == 'a',
        opt = 'at';
    else
        opt = 'wt';
    end
end

if nargin < 3, full_export = false; end

% If we get a filename, load the model
if ischar(model), load (model, 'model'); end

fid = fopen(filename, opt);

fprintf(fid, '<Model name="%s" version="%s">\n', model.name, model.version);
print_openrave(fid, model);
print_Points(fid, model, full_export);
if full_export,
    print_Cameras(fid, model);
end
fprintf(fid, '</Model>\n');

fclose(fid);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function print_openrave(fid, model)
% print_openrave - OpenRAVE data
fprintf(fid, '  <Openrave>\n');
fprintf(fid, '    <name>%s</name>\n', model.or_name);
fprintf(fid, '    <xml>%s</xml>\n', model.or_xml);
fprintf(fid, '    <transf>'); fprintf(fid, '%.6f ', model.or_transf(:)); fprintf(fid, '</transf>\n');
fprintf(fid, '  </Openrave>\n');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function print_Points(fid, model, full_export)
% print_Points - Print all Point3D entries

fprintf(fid, '  <Points>\n');
for i = 1:size(model.pts3D, 2),
    print_Point(fid, model, i);
    
    if full_export,
        for j = 1:size(model.pt_info(i).desc, 1),
            print_observ(fid, model.pt_info(i), j);
        end
    end
    fprintf(fid, '</Point>\n');
end
fprintf(fid, '  </Points>\n');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function print_observ(fid, pt, idx_pt)
% <Observation camera_id="n" desc_type="SIFT" loc="x;y;scale;orientation"
% desc="a;b;c;...">
fprintf(fid, '      <Observation ');
fprintf(fid, 'camera_id="%d" ', pt.cam_id(idx_pt));
fprintf(fid, 'desc_type="SIFT" ');
fprintf(fid, 'loc="'); fprintf(fid, '%.6f ', pt.locs(idx_pt,:)); fprintf(fid, '" ');
fprintf(fid, 'desc="'); fprintf(fid, '%.6f ', pt.desc(idx_pt,:));
fprintf(fid, '"/>\n');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function print_Point(fid, model, idx_pt)
% <Point p3d="x;y;z" nviews="" avg_err="" color="R;G;B" desc_type="SIFT"
% desc="a;b;c;...">
fprintf(fid, '    <Point ');
fprintf(fid, 'p3d="%.6f %.6f %.6f" ', model.pts3D(1, idx_pt), model.pts3D(2, idx_pt), model.pts3D(3, idx_pt));
fprintf(fid, 'nviews="%d" ', model.num_views(idx_pt));
fprintf(fid, 'avg_err="%.6f" ', model.avg_err(idx_pt));
fprintf(fid, 'color="%d %d %d" ', model.color3D(1,idx_pt), model.color3D(2,idx_pt), model.color3D(3,idx_pt));
fprintf(fid, 'desc_type="SIFT" ');
fprintf(fid, 'desc="'); fprintf(fid, '%.6f ', model.desc(idx_pt,:));
fprintf(fid, '">\n');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function print_Cameras(fid, model)
% print_Cameras - Print all Camera entries

fprintf(fid, '  <Cameras>\n');
for i = 1:size(model.cam_poses, 2),
    print_Camera(fid, model.cam_poses(:,i), i);
end
fprintf(fid, '  </Cameras>\n');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function print_Camera(fid, cpose, idx_cam)
% print_Camera - Camera entry

fprintf(fid, '    <Camera ');
fprintf(fid, 'id="%d" ', idx_cam);
fprintf(fid, 'rot_type="quat" ');
[q, tx] = format_rot('quat', cpose);
fprintf(fid, 'rot="'); fprintf(fid, '%.6f ', q); fprintf(fid, '" ');
fprintf(fid, 'tx="'); fprintf(fid, '%.6f ', tx); fprintf(fid, '"/>\n');
