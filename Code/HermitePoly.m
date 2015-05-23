function a=HermitePoly(ang_0, ang_f, vel_0, vel_f)
%Usage:     find Hermite Polynomials to interpolate the inital and final joint angles and velocities 
%Input:     initial and final joint angles and joint angle velocities.
%           ang_0, ang_f, vel_0, vef_f are 6*1 vectors for a 6R manipulator
%Output:    third-order Hermite Polynomial coefficients matrix A. 
%           A is a 4*6 matrix. Each column of A represents the Hermite
%           Polynomial coefficients for each joint angle.
%Author:    Yan Yan

global tf

if nargin<3
    vel_0=zeros(6,1); %default initial joint angle velocities are zeros
end
if nargin<4
    vel_f=zeros(6,1); %default final joint angle velocities are zeros
end
% if nargin<5,
%     tf=10; %default running time is 10 
% end;
if length(ang_0)~=6, %the 6R manipulator should have 6 joint angles
  error('length(ang_0)~=6');
end;
if length(ang_f)~=6,
  error('length(ang_f)~=6');
end;
if length(vel_0)~=6,
  error('length(vel_0)~=6');
end;
if length(vel_f)~=6,
  error('length(vel_f)~=6');
end;

%calculate the Hermite polynomials from Joint 1 to Joint 6
H=[1 1 -1 0;
    1 1 tf^2-1 tf^3-3*tf;
    0 1 0 -3;
    0 1 2*tf 3*tf^3-3];
a=zeros(4,6);
for i=1:6
    a(:,i)=inv(H)*[ang_0(i); ang_f(i); vel_0(i); vel_f(i)];
end


