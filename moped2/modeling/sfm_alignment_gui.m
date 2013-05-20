function varargout = sfm_alignment_gui(varargin)
% SFM_ALIGNMENT_GUI - Align a model with a predefined shape 
%    
%    Usage: sfm_alignment_gui(model);
%           sfm_alignment_gui(model, mesh);
%
%    Input:
%    model - SFM model you with to scale, rotate or translate
%    mesh - Structure that contains mesh.x, mesh.y, mesh.z, with points
%           [x(i) y(i) z(i)] that are drawn over the model
%
%    Output:
%    -NONE- The output model is saved through the GUI to the defined file,
%           in the current folder.
%
%    Example: Align a model with a cylinder of radius 0.05m and length of 
%    0.2m.
%
%    [x,y,z] = cylinder(0.05);
%    cyl.x = x;
%    cyl.y = y;
%    cyl.z = z;
%    sfm_alignment_gui(model, cylinder);
%
% Copyright: Carnegie Mellon University & Intel Corporation
% Author: Alvaro Collet (acollet@cs.cmu.edu)

% Edit the above text to modify the response to help sfm_alignment_gui

% Last Modified by GUIDE v2.5 24-Jul-2008 14:21:03

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @sfm_alignment_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @sfm_alignment_gui_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

% --- Executes just before sfm_alignment_gui is made visible.
function sfm_alignment_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to sfm_alignment_gui (see VARARGIN)

% Choose default command line output for sfm_alignment_gui
handles.output = hObject;

% ALVARO: Add models to handles structure
handles.sfm_model = varargin{1};
if length(varargin) > 1,
    handles.align_model = varargin{2};
else
    handles.align_model = false;
end

% ALVARO: Add rotation, scale and translation default values
handles.x_axis = 0;
handles.y_axis = 0;
handles.z_axis = 0;
handles.sc = 1;
handles.b_sc = 1;
handles.x_pos = 0;
handles.y_pos = 0;
handles.z_pos = 0;

% Update handles structure
guidata(hObject, handles);


% This sets up the initial plot - only do when we are invisible
% so window can get raised using sfm_alignment_gui.
if strcmp(get(hObject,'Visible'),'off')
    plot_models(handles);
end

% UIWAIT makes sfm_alignment_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = sfm_alignment_gui_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --------------------------------------------------------------------
function FileMenu_Callback(hObject, eventdata, handles)
% hObject    handle to FileMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function OpenMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to OpenMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
file = uigetfile('*.fig');
if ~isequal(file, 0)
    open(file);
end

% --------------------------------------------------------------------
function PrintMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to PrintMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
printdlg(handles.figure1)

% --------------------------------------------------------------------
function CloseMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to CloseMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selection = questdlg(['Close ' get(handles.figure1,'Name') '?'],...
                     ['Close ' get(handles.figure1,'Name') '...'],...
                     'Yes','No','Yes');
if strcmp(selection,'No')
    return;
end

delete(handles.figure1)


% --- Executes on slider movement.
function slider_xaxis_Callback(hObject, eventdata, handles)
% hObject    handle to slider_xaxis (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

handles.x_axis = get(hObject,'Value');

% Set corresponding value in text:
set(handles.txt_x_axis, 'String', num2str(handles.x_axis));

% Update handles structure
guidata(hObject, handles);

% Update plot
plot_models(handles);

% --- Executes during object creation, after setting all properties.
function slider_xaxis_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_xaxis (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_yaxis_Callback(hObject, eventdata, handles)
% hObject    handle to slider_yaxis (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.y_axis = get(hObject,'Value');

% Set corresponding value in text:
set(handles.txt_y_axis, 'String', num2str(handles.y_axis));

% Update handles structure
guidata(hObject, handles);

% Update plot
plot_models(handles);

% --- Executes during object creation, after setting all properties.
function slider_yaxis_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_yaxis (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_zaxis_Callback(hObject, eventdata, handles)
% hObject    handle to slider_zaxis (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.z_axis = get(hObject,'Value');

% Set corresponding value in text:
set(handles.txt_z_axis, 'String', num2str(handles.z_axis));

% Update handles structure
guidata(hObject, handles);

% Update plot
plot_models(handles);

% --- Executes during object creation, after setting all properties.
function slider_zaxis_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_zaxis (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function base_scale_Callback(hObject, eventdata, handles)
% hObject    handle to base_scale (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of base_scale as text
%        str2double(get(hObject,'String')) returns contents of base_scale as a double
handles.b_sc = str2double(get(hObject,'String'));

% Set corresponding value in text:
set(handles.txt_scale, 'String', num2str(handles.b_sc * handles.sc));

% Update handles structure
guidata(hObject, handles);

% Update plot
plot_models(handles);

% --- Executes during object creation, after setting all properties.
function base_scale_CreateFcn(hObject, eventdata, handles)
% hObject    handle to base_scale (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider_scale_Callback(hObject, eventdata, handles)
% hObject    handle to slider_scale (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.sc = get(hObject,'Value');

% Set corresponding value in text:
set(handles.txt_scale, 'String', num2str(handles.b_sc * handles.sc));

% Update handles structure
guidata(hObject, handles);

% Update plot
plot_models(handles);

% --- Executes during object creation, after setting all properties.
function slider_scale_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_scale (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in save_model.
function save_model_Callback(hObject, eventdata, handles)
% hObject    handle to save_model (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Compute rotation from the three angles
R = rotmat(handles.x_axis * pi/180, 'x') * rotmat(handles.y_axis * pi/180, 'y') * rotmat(handles.z_axis * pi/180, 'z');

% Perform the transformation on the whole model
model = sfm_transform_ext(handles.sfm_model, R, [handles.x_pos handles.y_pos handles.z_pos]', handles.sc*handles.b_sc);

% Get output model name
modelname = get(handles.out_modelname,'String');

% Save it!
fprintf('Saving model on %s... ', modelname);
save(modelname, 'model');
fprintf('Done!\n');


function out_modelname_Callback(hObject, eventdata, handles)
% hObject    handle to out_modelname (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of out_modelname as text
%        str2double(get(hObject,'String')) returns contents of out_modelname as a double


% --- Executes during object creation, after setting all properties.
function out_modelname_CreateFcn(hObject, eventdata, handles)
% hObject    handle to out_modelname (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over save_model.
function save_model_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to save_model (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)






function txt_x_axis_Callback(hObject, eventdata, handles)
% hObject    handle to txt_x_axis (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_x_axis as text
%        str2double(get(hObject,'String')) returns contents of txt_x_axis as a double

handles.x_axis = str2double(get(hObject,'String'));

% Set corresponding value in slider:
set(handles.slider_xaxis, 'Value', handles.x_axis);

% Update handles structure
guidata(hObject, handles);

% Update plot
plot_models(handles);

% --- Executes during object creation, after setting all properties.
function txt_x_axis_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_x_axis (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_y_axis_Callback(hObject, eventdata, handles)
% hObject    handle to txt_y_axis (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_y_axis as text
%        str2double(get(hObject,'String')) returns contents of txt_y_axis as a double
handles.y_axis = str2double(get(hObject,'String'));

% Set corresponding value in slider:
set(handles.slider_yaxis, 'Value', handles.y_axis);

% Update handles structure
guidata(hObject, handles);

% Update plot
plot_models(handles);

% --- Executes during object creation, after setting all properties.
function txt_y_axis_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_y_axis (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_scale_Callback(hObject, eventdata, handles)
% hObject    handle to txt_scale (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_scale as text
%        str2double(get(hObject,'String')) returns contents of txt_scale as a double

handles.sc = 1;
handles.b_sc = str2double(get(hObject,'String'));

% Set corresponding value in slider:
set(handles.slider_scale, 'Value', handles.sc);

% Set corresponding value in base scale
set(handles.base_scale, 'String', num2str(handles.b_sc));

% Update handles structure
guidata(hObject, handles);

% Update plot
plot_models(handles);

% --- Executes during object creation, after setting all properties.
function txt_scale_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_scale (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_x_pos_Callback(hObject, eventdata, handles)
% hObject    handle to txt_x_pos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_x_pos as text
%        str2double(get(hObject,'String')) returns contents of txt_x_pos as a double
handles.x_pos = str2double(get(hObject,'String'));

% Update handles structure
guidata(hObject, handles);

% Update plot
plot_models(handles);

% --- Executes during object creation, after setting all properties.
function txt_x_pos_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_x_pos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_y_pos_Callback(hObject, eventdata, handles)
% hObject    handle to txt_y_pos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_y_pos as text
%        str2double(get(hObject,'String')) returns contents of txt_y_pos as a double
handles.y_pos = str2double(get(hObject,'String'));

% Update handles structure
guidata(hObject, handles);

% Update plot
plot_models(handles);

% --- Executes during object creation, after setting all properties.
function txt_y_pos_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_y_pos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_z_pos_Callback(hObject, eventdata, handles)
% hObject    handle to txt_z_pos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_z_pos as text
%        str2double(get(hObject,'String')) returns contents of txt_z_pos as a double
handles.z_pos = str2double(get(hObject,'String'));

% Update handles structure
guidata(hObject, handles);

% Update plot
plot_models(handles);

% --- Executes during object creation, after setting all properties.
function txt_z_pos_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_z_pos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_z_axis_Callback(hObject, eventdata, handles)
% hObject    handle to txt_z_axis (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_z_axis as text
%        str2double(get(hObject,'String')) returns contents of txt_z_axis as a double
handles.z_axis = str2double(get(hObject,'String'));

% Set corresponding value in slider:
set(handles.slider_zaxis, 'Value', handles.z_axis);

% Update handles structure
guidata(hObject, handles);

% Update plot
plot_models(handles);

% --- Executes during object creation, after setting all properties.
function txt_z_axis_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_z_axis (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% ----------------------------------------------------------------------- %
% -------------------------- Alvaro's Code ------------------------------ %
% ----------------------------------------------------------------------- %

function plot_models(handles)
% PLOT_MODELS - Draw the two models to align in the same plot3
% handles    structure with handles and user data (see GUIDATA)

% Choose a figure for plots
figure(100);
hold off;
cla;

% Plot alignment model ----------------------------------------------------
if isstruct(handles.align_model)
    surf(handles.align_model.x, handles.align_model.y, handles.align_model.z);
end

% plot sfm model ----------------------------------------------------------
% 3D points in homogenous coords
pts3D = [handles.sfm_model.pts3D; ones(1, size(handles.sfm_model.pts3D, 2))];

% Transform model according to current values
R = rotmat(handles.x_axis * pi/180, 'x') * rotmat(handles.y_axis * pi/180, 'y') * rotmat(handles.z_axis * pi/180, 'z');
Proj = [R*handles.b_sc*handles.sc [handles.x_pos handles.y_pos handles.z_pos]'*handles.b_sc*handles.sc; 0 0 0 1];

% Apply transformation
pts3D = Proj * pts3D;

hold on;

% Draw points
plot3(pts3D(1,:), pts3D(2,:), pts3D(3,:), 'b.');
axis equal;

% Get proper scale for axis:
plot_scale = mean(std(pts3D(1:3,:) - repmat(mean(pts3D(1:3,:), 2), [1 size(pts3D(1:3,:),2)])));
% Draw axis
draw_camera(eye(3), [0 0 0]', plot_scale, 'axis');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
