function [ang, vel, acc]=Traj_Generate(ang_0, ang_f, vel_0, vel_f)
%Usage:     Generate trajectories (ang, vel and acc) of all the joints for
%           6R manipulator by using Hermite Polynomials interpolation
%Input:     initial and final joint angles and joint angle velocities
%           ang_0, ang_f, vel_0, vef_f (6*1 vectors) for a 6R manipulator;
%           the length of running time tf;
%           the number of sample points Nr.
%           Example: 
%           ang_0=zeros(6,1); 
%           ang_f=[pi/4; pi/4; pi/2; -pi/4; pi/3 ; pi]; vel_0=zeros(6,1);
%           vel_f=zeros(6,1); tf=5; Nr=50;
%Output:    ang (position), vel (velocity) and acc (acceleration) matrices of the
%           the joints. Each column of ang, vel and acc is a vector with
%           respect to time. Each row of ang, vel and acc represents ang,
%           vel and acc of all the joints at time t.
%Author:    Yan Yan

global tf Nr
%find third-order Hermite Polynomial coefficients matrix A. 
a=HermitePoly(ang_0, ang_f, vel_0, vel_f);

tvect=0:tf/(Nr-1):tf; % in seconds
tvect=tvect';
ang=zeros(Nr,6);
vel=zeros(Nr,6);
acc=zeros(Nr,6);
    for i=1:6
        ang(:,i)=a(1,i)*ones(Nr,1)+a(2,i)*tvect+a(3,i)*(tvect.*tvect-ones(Nr,1))+a(4,i)*(tvect.*tvect.*tvect-3*tvect);
        vel(:,i)=a(2,i)+2*a(3,i)*tvect+a(4,i)*(3*tvect.*tvect-3*ones(Nr,1));
        acc(:,i)=2*a(3,i)*ones(Nr,1)+6*a(4,i)*tvect;
    end
