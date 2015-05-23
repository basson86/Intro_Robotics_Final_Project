function ang_f=INV_kinematics_right(ang_0,Pdesire)

global L1 L4 L5 L7 tf Nr g

iV_J=ang_0;
% iV_J=[0;0;0;0;0;0];

P0= FWD_right(iV_J);

% tf=1;
% t_inc=0.1;
t_inc=tf/(Nr-1);
V_A=zeros(6,Nr);
P= zeros(3,Nr);

V_A(:,1)=iV_J;
P(:,1)= P0;

i=0;

for t=0:t_inc:tf;

i=i+1;
% view(90,0);
% ylim([-1,1]);
% zlim([-1.3,0.4]);
% F(i) = getframe;

%close all;
J= Jaco_right(V_A(:,i));

Pdot= (1/tf)*(Pdesire - P(:,1));
%IJ35=pinv(J35);
%J(2,:)=-J(2,:);
IJ=pinv(J);
IJ(:,2)=-IJ(:,2);


%V_A(3:5,i+1)=V_A(3:5,i)+ t_inc*IJ35*Pdot(1:3,1);
V_A(:,i+1)=V_A(:,i)+t_inc*IJ*Pdot(:,1);
P(:,i+1)= FWD_right(V_A(:,i+1));

end
    ang_f=V_A(:,Nr);
% Pdesire
% Pfinal=P(:,i)

% movie(F,20)
