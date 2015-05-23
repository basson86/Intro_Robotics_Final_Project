% One example of inverse kinematics and forward dynamics 
% Author: Yan Yan

close all;
global L1 L4 L5 L7 tf Nr g
L1=0.125;
L4=0.46;
L5=0.467;
L7=0.15;
g=9.81;

tf=5; Nr=20;
P_f=[-L1;0;-L4-L5-L7]; %cartisian coordinates
ang_0=[0;0;pi/60;-pi/60;0;0];
% ang_f=[pi/8; -pi/4; pi/2; -pi/4; pi/3 ; pi];
ang_f=INV_kinematics(ang_0, P_f);
P_f_real=FWD(ang_f)
vel_0=zeros(6,1);
vel_f=zeros(6,1); 
[ang, vel, acc]=Traj_Generate(ang_0, ang_f, vel_0, vel_f);
tau= FWD_Dynamics(ang, vel, acc);

tvect=0:tf/(Nr-1):tf; % in seconds

for t=1:Nr

PLOTFWD_left(ang(t,:))
h=gcf;
% view(90,0);
% ylim([-1,1]);
% zlim([-1.3,0.4]);
% axis equal
set(h,'nextplot','replacechildren');
xlim([-0.5 0.5]);
ylim([-1 1]);
zlim([-1.5 0]);
% axis fixed
F=getframe(h);

end

% movie(F,20)

