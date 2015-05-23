function ang_f=INV_kinematics(ang_0,Pdesire)

%Usage:     calculate the inverse dynamics of 6R manipulator
%Usage:     calculate the inverse kinematics of the 6R manipulator
%Input:     initial configuration of the joint angles and the desired
%           effector position in Cartisian coordinates 
%Output:    the joint angles at the desired person 
%Author:    Tze-yuan Cheng

global L1 L4 L5 L7 tf Nr g

iV_J=ang_0;
P0= FWD(iV_J);
t_inc=tf/(Nr-1);
V_A=zeros(6,Nr);
P= zeros(3,Nr);

V_A(:,1)=iV_J;
P(:,1)= P0;

i=0;

% calculate the inverse kinemcatics by linear interpolation 
for t=0:t_inc:tf; 

i=i+1;

J= Jaco(V_A(:,i));

Pdot= (1/tf)*(Pdesire - P(:,1));
IJ=pinv(J);
IJ(:,2)=-IJ(:,2);

V_A(:,i+1)=V_A(:,i)+t_inc*IJ*Pdot(:,1);
P(:,i+1)= FWD(V_A(:,i+1));

end
ang_f=V_A(:,Nr);

