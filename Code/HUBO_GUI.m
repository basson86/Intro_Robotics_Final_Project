%Usage:         HUBO_GUI M-file for HUBO_GUI.fig. HUBO_GUI was used to
%               demonstrate the forward/inverse kinematics and forward dynamics of the
%               HUBO biped robot. 
%Author:        Yan Yan

function varargout = HUBO_GUI(varargin)

clc;
% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @HUBO_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @HUBO_GUI_OutputFcn, ...
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


% --- Executes just before HUBO_GUI is made visible.
function HUBO_GUI_OpeningFcn(hObject, eventdata, handles, varargin)

handles.output = hObject;
set(hObject,'toolbar','figure'); % add toolbar to the figures 
guidata(hObject, handles);


% UIWAIT makes HUBO_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = HUBO_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


function x_right_Callback(hObject, eventdata, handles)
global x_right
x_right=get(handles.x_right,'String');
x_right=str2num(x_right);
guidata(hObject, handles);

function x_right_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function z_right_Callback(hObject, eventdata, handles)
global z_right
z_right=get(handles.z_right,'String');
z_right=str2num(z_right);
guidata(hObject, handles);

function z_right_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function y_right_Callback(hObject, eventdata, handles)
global y_right
y_right=get(handles.y_right,'String');
y_right=str2num(y_right);
guidata(hObject, handles);
function y_right_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function ang1_right_Callback(hObject, eventdata, handles)
global ang1_right
ang1_right=get(handles.ang1_right,'String');
ang1_right=str2num(ang1_right);
guidata(hObject, handles);

function ang1_right_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function ang3_right_Callback(hObject, eventdata, handles)
global ang3_right
ang3_right=get(handles.ang3_right,'String');
ang3_right=str2num(ang3_right);
guidata(hObject, handles);

function ang3_right_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function ang2_right_Callback(hObject, eventdata, handles)
global ang2_right
ang2_right=get(handles.ang2_right,'String');
ang2_right=str2num(ang2_right);
guidata(hObject, handles);

function ang2_right_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function ang4_right_Callback(hObject, eventdata, handles)
global ang4_right
ang4_right=get(handles.ang4_right,'String');
ang4_right=str2num(ang4_right);
guidata(hObject, handles);

function ang4_right_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ang6_right_Callback(hObject, eventdata, handles)
global ang6_right
ang6_right=get(handles.ang6_right,'String');
ang6_right=str2num(ang6_right);
guidata(hObject, handles);

function ang6_right_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function ang5_right_Callback(hObject, eventdata, handles)
global ang5_right
ang5_right=get(handles.ang5_right,'String');
ang5_right=str2num(ang5_right);
guidata(hObject, handles);

function ang5_right_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function x_left_Callback(hObject, eventdata, handles)

global x_left
x_left=get(handles.x_left,'String');
x_left=str2num(x_left);
guidata(hObject, handles);

function x_left_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function y_left_Callback(hObject, eventdata, handles)

global y_left
y_left=get(handles.y_left,'String');
y_left=str2num(y_left);
guidata(hObject, handles);

function y_left_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function z_left_Callback(hObject, eventdata, handles)
global z_left
z_left=get(handles.z_left,'String');
z_left=str2num(z_left);
guidata(hObject, handles);

function z_left_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function ang1_left_Callback(hObject, eventdata, handles)
global ang1_left
ang1_left=get(handles.ang1_left,'String');
ang1_left=str2num(ang1_left);
guidata(hObject, handles);

function ang1_left_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function ang2_left_Callback(hObject, eventdata, handles)
global ang2_left
ang2_left=get(handles.ang2_left,'String');
ang2_left=str2num(ang2_left);
guidata(hObject, handles);

function ang2_left_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function ang3_left_Callback(hObject, eventdata, handles)
global ang3_left
ang3_left=get(handles.ang3_left,'String');
ang3_left=str2num(ang3_left);
guidata(hObject, handles);

function ang3_left_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function ang4_left_Callback(hObject, eventdata, handles)
global ang4_left
ang4_left=get(handles.ang4_left,'String');
ang4_left=str2num(ang4_left);
guidata(hObject, handles);

function ang4_left_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function ang5_left_Callback(hObject, eventdata, handles)
global ang5_left
ang5_left=get(handles.ang5_left,'String');
ang5_left=str2num(ang5_left);
guidata(hObject, handles);

function ang5_left_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function ang6_left_Callback(hObject, eventdata, handles)
global ang5_left
ang5_left=get(handles.ang5_left,'String');
ang5_left=str2num(ang5_left);
guidata(hObject, handles);

function ang6_left_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function tau1_right_Callback(hObject, eventdata, handles)
global tau1_right
tau1_right=get(handles.tau1_right,'String');
tau1_right=str2num(tau1_right);
guidata(hObject, handles);
function tau1_right_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function tau3_right_Callback(hObject, eventdata, handles)
global tau3_right
tau3_right=get(handles.tau3_right,'String');
tau3_right=str2num(tau3_right);
guidata(hObject, handles);
function tau3_right_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function tau2_right_Callback(hObject, eventdata, handles)
global tau2_right
tau2_right=get(handles.tau2_right,'String');
tau2_right=str2num(tau2_right);
guidata(hObject, handles);

function tau2_right_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function tau4_right_Callback(hObject, eventdata, handles)
global tau4_right
tau4_right=get(handles.tau4_right,'String');
tau4_right=str2num(tau4_right);
guidata(hObject, handles);

function tau4_right_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function tau6_right_Callback(hObject, eventdata, handles)
global tau6_right
tau6_right=get(handles.tau6_right,'String');
tau6_right=str2num(tau6_right);
guidata(hObject, handles);
function tau6_right_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function tau5_right_Callback(hObject, eventdata, handles)
global tau5_right
tau5_right=get(handles.tau5_right,'String');
tau5_right=str2num(tau5_right);
guidata(hObject, handles);
function tau5_right_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function tau1_left_Callback(hObject, eventdata, handles)
global tau1_left
tau1_left=get(handles.tau1_left,'String');
tau1_left=str2num(tau1_left);
guidata(hObject, handles);

function tau1_left_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function tau3_left_Callback(hObject, eventdata, handles)
global tau3_left
tau3_left=get(handles.tau3_left,'String');
tau3_left=str2num(tau3_left);
guidata(hObject, handles);
function tau3_left_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function tau2_left_Callback(hObject, eventdata, handles)
global tau2_left
tau2_left=get(handles.tau2_left,'String');
tau2_left=str2num(tau2_left);
guidata(hObject, handles);
function tau2_left_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function tau4_left_Callback(hObject, eventdata, handles)
global tau4_left
tau4_left=get(handles.tau4_left,'String');
tau4_left=str2num(tau4_left);
guidata(hObject, handles);
function tau4_left_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function tau6_left_Callback(hObject, eventdata, handles)
global tau6_left
tau6_left=get(handles.tau6_left,'String');
tau6_left=str2num(tau6_left);
guidata(hObject, handles);
function tau6_left_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function tau5_left_Callback(hObject, eventdata, handles)
global tau5_left
tau5_left=get(handles.tau5_left,'String');
tau5_left=str2num(tau5_left);
guidata(hObject, handles);
function tau5_left_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function plot_torque_init(hObject, eventdata, handles)

cla(handles.axes2,'reset')
axes(handles.axes2)
plot(0,0);
guidata(hObject, handles);


function pushbutton_Init_Callback(hObject, eventdata, handles)

global x_left y_left z_left ang1_left ang2_left ang3_left ang4_left ang5_left ang6_left...
    x_right y_right z_right ang1_right ang2_right ang3_right ang4_right ang5_right ang6_right...
    L1 L4 L5 L7 tf Nr g 

L1=0.125;
L4=0.46;
L5=0.467;
L7=0.15;
g=9.81;
tf=5; Nr=20;

x_left=-L1; y_left=0; z_left=-L4-L5-L7;
ang1_left=0; ang2_left=0; ang3_left=0;
ang4_left=0; ang5_left=0; ang6_left=0;

x_right=L1; y_right=0; z_right=-L4-L5-L7;
ang1_right=0; ang2_right=0; ang3_right=0;
ang4_right=0; ang5_right=0; ang6_right=0;

ang_0_left=[ang1_left; ang2_left; ang3_left; ang4_left; ang5_left; ang6_left]*pi/180;
ang_0_right=[ang1_right; ang2_right; ang3_right; ang4_right; ang5_right; ang6_right]*pi/180;

cla(handles.axes1,'reset')
axes(handles.axes1)
plot3([0.125 -0.125],[0 0], [0 0]);
hold on;
ellipsoid(0,0,0,0.02,0.02,0.03,8);
colormap pink
hold on;
PLOTFWD_left(ang_0_left);
hold on;
PLOTFWD_right(ang_0_right);

xlim([-0.5 0.5]);
ylim([-1 1]);
zlim([-1.2 0]);
axis square
plot_torque_init(hObject, eventdata, handles)
% ang_0_left=[0;0;0;0;0;0];
% % ang_0_left=[0;0;pi/60;-pi/60;0;0];
% ang_f_left=[ang1_left; ang2_left; ang3_left; ang4_left; ang5_left; ang6_left]*pi/180;
% vel_f_left=zeros(6,1);
% vel_0_left=zeros(6,1);
% [ang_left, vel_left, acc_left]=Traj_Generate(ang_0_left, ang_f_left, vel_0_left, vel_f_left);
% tau_left= FWD_Dynamics(ang_left, vel_left, acc_left);
% tinc=tf/(Nr-1);
% cla(handles.axes2,'reset')
% axes(handles.axes2)
% for t=1:Nr
%     plot(t*tinc,tau_left(t,1),'o','MarkerFaceColor','b','MarkerSize',6);
%     hold on; plot(t*tinc,tau_left(t,2),'o','MarkerFaceColor','r','MarkerSize',6); 
%     hold on; plot(t*tinc,tau_left(t,3),'o','MarkerFaceColor','k','MarkerSize',6);
%     hold on; plot(t*tinc,tau_left(t,4),'o','MarkerFaceColor','y','MarkerSize',6); 
%     hold on; plot(t*tinc,tau_left(t,5),'o','MarkerFaceColor','g','MarkerSize',6); 
%     hold on; plot(t*tinc,tau_left(t,6),'o','MarkerFaceColor','m','MarkerSize',6);
%     h=gca;
%     F=getframe(h);
%     grid on;
%     xlim([0 tf]);
%     xlabel('time (s)')
%     ylabel('torque (Nm)')
%     legend('\tau_{1}','\tau_{2}','\tau_{3}','\tau_{4}','\tau_{5}','\tau_{6}','Location','NorthOutside','Orientation','horizontal');
% end

SetHandles(hObject, eventdata, handles)
guidata(hObject, handles);

function pushbutton_Pose_Callback(hObject, eventdata, handles)

global x_left y_left z_left ang1_left ang2_left ang3_left ang4_left ang5_left ang6_left...
    x_right y_right z_right ang1_right ang2_right ang3_right ang4_right ang5_right ang6_right...
    L1 L4 L5 L7 tf Nr g 

plot_torque_init(hObject, eventdata, handles)
%======= Left leg trajectory generation =======================
ang_0_left=[0;0;0;0;0;0];
% ang_0_left=[0;0;pi/60;-pi/60;0;0];
ang_f_left=[ang1_left; ang2_left; ang3_left; ang4_left; ang5_left; ang6_left]*pi/180;
vel_f_left=zeros(6,1);
vel_0_left=zeros(6,1);
[ang_left, vel_left, acc_left]=Traj_Generate(ang_0_left, ang_f_left, vel_0_left, vel_f_left);
tau_left= FWD_Dynamics(ang_left, vel_left, acc_left);
 T1=DHF([0, -0.125, 0, ang_left(1)]); % homogeneous transformation from base frame to Frame {1}
 T2=DHF([pi/2, 0, 0, pi/2+ang_left(2)]); % homogeneous transformation from Frame {1} to Frame {2}
 T3=DHF([pi/2, 0, 0, ang_left(3)]); % homogeneous transformation from Frame {2} to Frame {3}
 T4=DHF([0, -0.46, 0, ang_left(4)]); % homogeneous transformation from Frame {3} to Frame {4}
 T5=DHF([0, -0.467, 0, ang_left(5)]); % homogeneous transformation from Frame {4} to Frame {5}
 T6=DHF([-pi/2, 0, 0, ang_left(6)]); % homogeneous transformation from Frame {5} to Fame {6}
 T7=DHF([0, -0.15, 0, 0]); % homogeneous transformation from Frame {6} to tool frame
 P7_left=T1*T2*T3*T4*T5*T6*T7*[x_left; y_left; z_left;1];
 x_left=P7_left(1); y_left=P7_left(2); z_left=P7_left(3);
%======= Right leg trajectory generation =======================
ang_0_right=[0;0;0;0;0;0];
% ang_0_right=[0;0;pi/60;-pi/60;0;0];
ang_f_right=[ang1_right; ang2_right; ang3_right; ang4_right; ang5_right; ang6_right]*pi/180;
vel_0_right=zeros(6,1); 
vel_f_right=zeros(6,1); 
[ang_right, vel_right, acc_right]=Traj_Generate(ang_0_right, ang_f_right, vel_0_right, vel_f_right);
tau_right= FWD_Dynamics(ang_right, vel_right, acc_right);

 T1=DHF([0, 0.125, 0, ang_right(1)]); % homogeneous transformation from base frame to Frame {1}
 T2=DHF([pi/2, 0, 0, pi/2+ang_right(2)]); % homogeneous transformation from Frame {1} to Frame {2}
 T3=DHF([pi/2, 0, 0, ang_right(3)]); % homogeneous transformation from Frame {2} to Frame {3}
 T4=DHF([0, -0.46, 0, ang_right(4)]); % homogeneous transformation from Frame {3} to Frame {4}
 T5=DHF([0, -0.467, 0, ang_right(5)]); % homogeneous transformation from Frame {4} to Frame {5}
 T6=DHF([-pi/2, 0, 0, ang_right(6)]); % homogeneous transformation from Frame {5} to Fame {6}
 T7=DHF([0, -0.15, 0, 0]); % homogeneous transformation from Frame {6} to tool frame
 P7_right=T1*T2*T3*T4*T5*T6*T7*[x_right; y_right; z_right;1];
 x_right=P7_right(1); y_right=P7_right(2); z_right=P7_right(3);

%=========== plot stick link================
tvect=0:tf/(Nr-1):tf; % in seconds
cla(handles.axes1,'reset')
axes(handles.axes1)
for t=1:Nr
plot3([0.125 -0.125],[0 0], [0 0]);
hold on;
ellipsoid(0,0,0,0.02,0.02,0.03,8);
colormap pink
hold on;
PLOTFWD_left(ang_left(t,:))
hold on; PLOTFWD_right(ang_right(t,:))
h=gca;
set(h,'nextplot','replacechildren');
xlim([-0.5 0.5]);
ylim([-1 1]);
zlim([-1.2 0]);
axis square
f=gca;
F=getframe(f);
end
SetHandles(hObject, eventdata, handles)

%======= plot torque ===================
tinc=tf/(Nr-1);
cla(handles.axes2,'reset')
axes(handles.axes2)
% figure(20);
for t=1:Nr
    plot(t*tinc,tau_left(t,1),'o','MarkerFaceColor','b','MarkerSize',6);
    hold on; plot(t*tinc,tau_left(t,2),'o','MarkerFaceColor','r','MarkerSize',6); 
    hold on; plot(t*tinc,tau_left(t,3),'o','MarkerFaceColor','k','MarkerSize',6);
    hold on; plot(t*tinc,tau_left(t,4),'o','MarkerFaceColor','y','MarkerSize',6); 
    hold on; plot(t*tinc,tau_left(t,5),'o','MarkerFaceColor','g','MarkerSize',6); 
    hold on; plot(t*tinc,tau_left(t,6),'o','MarkerFaceColor','m','MarkerSize',6);
    h=gca;
    F=getframe(h);
    grid on;
    xlim([0 tf]);
    xlabel('time (s)')
    ylabel('torque (Nm)')
    legend('\tau_{1}','\tau_{2}','\tau_{3}','\tau_{4}','\tau_{5}','\tau_{6}','Location','NorthOutside','Orientation','horizontal');
end


guidata(hObject, handles);

function pushbutton_Walk_Callback(hObject, eventdata, handles)

global x_left y_left z_left ang1_left ang2_left ang3_left ang4_left ang5_left ang6_left...
    x_right y_right z_right ang1_right ang2_right ang3_right ang4_right ang5_right ang6_right...
    L1 L4 L5 L7 tf Nr g 

plot_torque_init(hObject, eventdata, handles)
AngR=zeros(6,7);
AngL=zeros(6,7);

AngR(3:5,1)= [pi*30/180;-pi*20/180;pi*0/180];
AngR(3:5,2)= [pi*25/180;-pi*45/180;pi*10/180];
AngR(3:5,3)= [pi*18/180;-pi*48/180;pi*20/180];
AngR(3:5,4)= [pi*-15/180;-pi*18/180;pi*35/180];
AngR(3:5,5)= [pi*30/180;-pi*68/180;pi*45/180];
AngR(3:5,6)= [pi*50/180;-pi*70/180;pi*20/180];
AngR(3:5,7)= AngR(3:5,1);

AngL(3:5,1)= AngR(3:5,4);
AngL(3:5,2)= AngR(3:5,5); 
AngL(3:5,3)= AngR(3:5,6);
AngL(3:5,4)= AngR(3:5,1);
AngL(3:5,5)= AngR(3:5,2);
AngL(3:5,6)= AngR(3:5,3);
AngL(3:5,7)= AngL(3:5,1);

for k=1:6
    %======= Left leg trajectory generation =======================
    ang_0_left=AngL(:,k);
    ang_f_left=AngL(:,k+1);
ang1_left=ang_f_left(1)*180/pi; ang2_left=ang_f_left(2)*180/pi; ang3_left=ang_f_left(3)*180/pi;
ang4_left=ang_f_left(4)*180/pi; ang5_left=ang_f_left(5)*180/pi; ang6_left=ang_f_left(6)*180/pi;
    vel_f_left=zeros(6,1);
    vel_0_left=zeros(6,1);
    [ang_left, vel_left, acc_left]=Traj_Generate(ang_0_left, ang_f_left, vel_0_left, vel_f_left);
    tau_left= FWD_Dynamics(ang_left, vel_left, acc_left);
 T1=DHF([0, -0.125, 0, ang_left(1)]); % homogeneous transformation from base frame to Frame {1}
 T2=DHF([pi/2, 0, 0, pi/2+ang_left(2)]); % homogeneous transformation from Frame {1} to Frame {2}
 T3=DHF([pi/2, 0, 0, ang_left(3)]); % homogeneous transformation from Frame {2} to Frame {3}
 T4=DHF([0, -0.46, 0, ang_left(4)]); % homogeneous transformation from Frame {3} to Frame {4}
 T5=DHF([0, -0.467, 0, ang_left(5)]); % homogeneous transformation from Frame {4} to Frame {5}
 T6=DHF([-pi/2, 0, 0, ang_left(6)]); % homogeneous transformation from Frame {5} to Fame {6}
 T7=DHF([0, -0.15, 0, 0]); % homogeneous transformation from Frame {6} to tool frame
 P7_left=T1*T2*T3*T4*T5*T6*T7*[x_left; y_left; z_left;1];
 x_left=P7_left(1); y_left=P7_left(2); z_left=P7_left(3);
    %======= Right leg trajectory generation =======================
    ang_0_right=AngR(:,k);
    ang_f_right=AngR(:,k+1);
ang1_right=ang_f_right(1)*180/pi; ang2_right=ang_f_right(2)*180/pi; ang3_right=ang_f_right(3)*180/pi;
ang4_right=ang_f_right(4)*180/pi; ang5_right=ang_f_right(5)*180/pi; ang6_right=ang_f_right(6)*180/pi;
    vel_0_right=zeros(6,1); 
    vel_f_right=zeros(6,1); 
    [ang_right, vel_right, acc_right]=Traj_Generate(ang_0_right, ang_f_right, vel_0_right, vel_f_right);
    tau_right= FWD_Dynamics(ang_right, vel_right, acc_right);
 T1=DHF([0, 0.125, 0, ang_right(1)]); % homogeneous transformation from base frame to Frame {1}
 T2=DHF([pi/2, 0, 0, pi/2+ang_right(2)]); % homogeneous transformation from Frame {1} to Frame {2}
 T3=DHF([pi/2, 0, 0, ang_right(3)]); % homogeneous transformation from Frame {2} to Frame {3}
 T4=DHF([0, -0.46, 0, ang_right(4)]); % homogeneous transformation from Frame {3} to Frame {4}
 T5=DHF([0, -0.467, 0, ang_right(5)]); % homogeneous transformation from Frame {4} to Frame {5}
 T6=DHF([-pi/2, 0, 0, ang_right(6)]); % homogeneous transformation from Frame {5} to Fame {6}
 T7=DHF([0, -0.15, 0, 0]); % homogeneous transformation from Frame {6} to tool frame
 P7_right=T1*T2*T3*T4*T5*T6*T7*[x_right; y_right; z_right;1];
 x_right=P7_right(1); y_right=P7_right(2); z_right=P7_right(3);
    cla(handles.axes1,'reset')
    axes(handles.axes1)
    for t=1:Nr
    plot3([0.125 -0.125],[0 0], [0 0]);
    hold on;
    ellipsoid(0,0,0,0.03,0.03,0.03,8);
    colormap pink
    hold on;
    PLOTFWD_left(ang_left(t,:))
    hold on; PLOTFWD_right(ang_right(t,:))
    h=gca;
    set(h,'nextplot','replacechildren');
    view(90,0);
    xlim([-0.5 0.5]);
    ylim([-1 1]);
    zlim([-1.2 0.1]);
    axis square
    F=getframe(h);
    end
    g=gca;
    G(k)=getframe(g);
    SetHandles(hObject, eventdata, handles)
    %======= plot torque ===================
    tinc=tf/(Nr-1);
    cla(handles.axes2,'reset')
    axes(handles.axes2)

    for t=1:Nr
         plot(t*tinc,tau_left(t,1),'o','MarkerFaceColor','b','MarkerSize',6);
    hold on; plot(t*tinc,tau_left(t,2),'o','MarkerFaceColor','r','MarkerSize',6); 
    hold on; plot(t*tinc,tau_left(t,3),'o','MarkerFaceColor','k','MarkerSize',6);
    hold on; plot(t*tinc,tau_left(t,4),'o','MarkerFaceColor','y','MarkerSize',6); 
    hold on; plot(t*tinc,tau_left(t,5),'o','MarkerFaceColor','g','MarkerSize',6); 
    hold on; plot(t*tinc,tau_left(t,6),'o','MarkerFaceColor','m','MarkerSize',6);
        h=gca;
        F=getframe(h);
        grid on;
        xlim([0 tf]);
        xlabel('time')
        ylabel('torque')
        legend('\tau_{1}','\tau_{2}','\tau_{3}','\tau_{4}','\tau_{5}','\tau_{6}','Location','NorthOutside','Orientation','horizontal');
    end
end

cla(handles.axes2,'reset')
cla(handles.axes1,'reset')
axes(handles.axes1)
movie(G,5,1)
cla(handles.axes1,'reset')

guidata(hObject, handles);

function pushbutton_Kick_Callback(hObject, eventdata, handles)

global x_left y_left z_left ang1_left ang2_left ang3_left ang4_left ang5_left ang6_left...
    x_right y_right z_right ang1_right ang2_right ang3_right ang4_right ang5_right ang6_right...
    L1 L4 L5 L7 tf Nr g 

plot_torque_init(hObject, eventdata, handles)
%======= Left leg trajectory generation =======================
ang_0_left=[0;0;pi/60;-pi/60;0;0];
P_f_left=[x_left; y_left; z_left];
ang_f_left=INV_kinematics(ang_0_left, P_f_left);
ang1_left=ang_f_left(1)*180/pi; ang2_left=ang_f_left(2)*180/pi; ang3_left=ang_f_left(3)*180/pi;
ang4_left=ang_f_left(4)*180/pi; ang5_left=ang_f_left(5)*180/pi; ang6_left=ang_f_left(6)*180/pi;
vel_f_left=zeros(6,1);
vel_0_left=zeros(6,1);
[ang_left, vel_left, acc_left]=Traj_Generate(ang_0_left, ang_f_left, vel_0_left, vel_f_left);
tau_left= FWD_Dynamics(ang_left, vel_left, acc_left);

%======= Right leg trajectory generation =======================
ang_0_right=[0;0;pi/60;-pi/60;0;0];
P_f_right=[x_right; y_right; z_right];
ang_f_right=INV_kinematics_right(ang_0_right, P_f_right);
ang1_right=ang_f_right(1)*180/pi; ang2_right=ang_f_right(2)*180/pi; ang3_right=ang_f_right(3)*180/pi;
ang4_right=ang_f_right(4)*180/pi; ang5_right=ang_f_right(5)*180/pi; ang6_right=ang_f_right(6)*180/pi;
vel_0_right=zeros(6,1); 
vel_f_right=zeros(6,1); 
[ang_right, vel_right, acc_right]=Traj_Generate(ang_0_right, ang_f_right, vel_0_right, vel_f_right);
tau_right= FWD_Dynamics(ang_right, vel_right, acc_right);

tvect=0:tf/(Nr-1):tf; % in seconds
% figure(1), plot(tvect,tau);
cla(handles.axes1,'reset')
axes(handles.axes1)
for t=1:Nr
plot3([0.125 -0.125],[0 0], [0 0]);
hold on;
ellipsoid(0,0,0,0.02,0.02,0.03,8);
colormap pink
hold on;
PLOTFWD_left(ang_left(t,:))
hold on; PLOTFWD_right(ang_right(t,:))
h=gca;
set(h,'nextplot','replacechildren');
xlim([-0.5 0.5]);
ylim([-1 1]);
zlim([-1.2 0]);
axis square
F=getframe(h);
end
SetHandles(hObject, eventdata, handles)
%======= plot torque ===================
tinc=tf/(Nr-1);
cla(handles.axes2,'reset')
axes(handles.axes2)

for t=1:Nr
     plot(t*tinc,tau_left(t,1),'o','MarkerFaceColor','b','MarkerSize',6);
    hold on; plot(t*tinc,tau_left(t,2),'o','MarkerFaceColor','r','MarkerSize',6); 
    hold on; plot(t*tinc,tau_left(t,3),'o','MarkerFaceColor','k','MarkerSize',6);
    hold on; plot(t*tinc,tau_left(t,4),'o','MarkerFaceColor','y','MarkerSize',6); 
    hold on; plot(t*tinc,tau_left(t,5),'o','MarkerFaceColor','g','MarkerSize',6); 
    hold on; plot(t*tinc,tau_left(t,6),'o','MarkerFaceColor','m','MarkerSize',6);
    h=gca;
    F=getframe(h);
    grid on;
    xlim([0 tf]);
    xlabel('time')
    ylabel('torque')
    legend('\tau_{1}','\tau_{2}','\tau_{3}','\tau_{4}','\tau_{5}','\tau_{6}','Location','NorthOutside','Orientation','horizontal');
end


guidata(hObject, handles);


function SetHandles(hObject, eventdata, handles)

global x_left y_left z_left ang1_left ang2_left ang3_left ang4_left ang5_left ang6_left...
    x_right y_right z_right ang1_right ang2_right ang3_right ang4_right ang5_right ang6_right

set(handles.x_left,'String', num2str(x_left));
set(handles.y_left,'String', num2str(y_left));
set(handles.z_left,'String', num2str(z_left));
set(handles.ang1_left,'String', num2str(ang1_left));
set(handles.ang2_left,'String', num2str(ang2_left));
set(handles.ang3_left,'String', num2str(ang3_left));
set(handles.ang4_left,'String', num2str(ang4_left));
set(handles.ang5_left,'String', num2str(ang5_left));
set(handles.ang6_left,'String', num2str(ang6_left));

set(handles.x_right,'String', num2str(x_right));
set(handles.y_right,'String', num2str(y_right));
set(handles.z_right,'String', num2str(z_right));
set(handles.ang1_right,'String', num2str(ang1_right));
set(handles.ang2_right,'String', num2str(ang2_right));
set(handles.ang3_right,'String', num2str(ang3_right));
set(handles.ang4_right,'String', num2str(ang4_right));
set(handles.ang5_right,'String', num2str(ang5_right));
set(handles.ang6_right,'String', num2str(ang6_right));

guidata(hObject, handles);
