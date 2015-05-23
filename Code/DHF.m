function T=DHF(p)
%       input: DH parameters p=[alpha; a; d; theta]
%       output: the corresponding 4 by 4 homogeneous transformation matrix from Frame {i-1} to Frame {i}
%       author: Yan Yan
alpha=p(1);
a=p(2);
d=p(3);
theta=p(4);

Rx=[1 0 0 0                            %rotation around x axis
    0 cos(alpha) -sin(alpha) 0  
    0 sin(alpha) cos(alpha) 0
    0 0 0 1];
Dx=[1 0 0 a;                           %transition along x axis
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];
Rz=[cos(theta) -sin(theta) 0 0;        %rotation around z axis
    sin(theta) cos(theta) 0 0;
    0 0 1 0;
    0 0 0 1];
Dz=[1 0 0 0;                           %transition along z axis
    0 1 0 0;
    0 0 1 d;
    0 0 0 1];
T=Rx*Dx*Rz*Dz;
