clear all;
close all;

iV_J=[0;0;pi/6;-pi/6;0;0];
% iV_J=[0;0;1;0;0;0];

P0= FWD(iV_J);
Pdesire=[-0.125;0.4;-1.2154];


tf=5;
Nr=20;
t_inc=tf/(Nr-1);

V_A=zeros(6,(tf/t_inc)+1);
P= zeros(3,(tf/t_inc)+1);

V_A(:,1)=iV_J;
P(:,1)= P0;

i=0;

for t=0:t_inc:tf;
i=i+1;
view(90,0);
ylim([-1,1]);
zlim([-1.3,0.4]);
F(i) = getframe;

%close all;
J= Jaco(V_A(:,i));

Pdot= (1/tf)*(Pdesire - P(:,1));
%IJ35=pinv(J35);
%J(2,:)=-J(2,:);
IJ=pinv(J);
IJ(:,2)=-IJ(:,2);

%V_A(3:5,i+1)=V_A(3:5,i)+ t_inc*IJ35*Pdot(1:3,1);
V_A(:,i+1)=V_A(:,i)+ t_inc*IJ*Pdot(:,1);

P(:,i+1)= FWD(V_A(:,i+1));
h=gcf;
% view(90,0);
% ylim([-1,1]);
% zlim([-1.3,0.4]);
% axis equal
set(h,'nextplot','replacechildren');
xlim([-0.5 0.5]);
ylim([-1 1]);
zlim([-1.5 0]);
end
    
Pdesire

Pfinal=P(:,i)
ang_f=V_A(:,i);
% movie(F,20)
