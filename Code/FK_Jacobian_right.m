function [Fk,J]= FK_Jacobian(V_A)
syms x1 x2 x3 x4 x5 x6;

T1=[cos(x1),-sin(x1),0,0.125; sin(x1),cos(x1),0,0;0,0,1,0;0,0,0,1];
T2=[-sin(x2),-cos(x2),0,0; 0,0,1,0;cos(x2),-sin(x2),0,0;0,0,0,1];
T3=[cos(x3),-sin(x3),0,0; 0,0,1,0;sin(x3),cos(x3),0,0;0,0,0,1];
T4=[cos(x4),-sin(x4),0,-0.46; sin(x4),cos(x4),0,0;0,0,1,0;0,0,0,1];
T5=[cos(x5),-sin(x5),0,-0.467; sin(x5),cos(x5),0,0;0,0,1,0;0,0,0,1];
T6=[cos(x6),-sin(x6),0,0; 0,0,1,0;-sin(x6),-cos(x6),0,0;0,0,0,1];
T7=[1,0,0,-0.15; 0,1,0,0;0,0,1,0;0,0,0,1];

T07=T1*T2*T3*T4*T5*T6*T7;

Fk= T07(1:3,4); % forward kinematics function

v=[x1 x2 x3 x4 x5 x6]; % theta

J1 = jacobian(Fk,v); % Jacobian matrix


