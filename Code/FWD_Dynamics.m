function tau = FWD_Dynamics(ang, vel, acc)
%Usage:     calculate the forward dynamics of 6R manipulator
%Input:     ang (position), vel (velocity) and acc (acceleration) matrices of the
%           the joints. Each column of ang, vel and acc is a vector with
%           respect to time. Each row of ang, vel and acc represents ang,
%           vel and acc of all the joints at time t.
%Output:    the joint troques for 6R manipulator required to cause the motion.
%           tau is a 6*1 vector.
%Author:    Yan Yan

global L1 L4 L5 L7 g 

Nr=length(ang(:,1));

%========define parameters of the 6R manipulator========
ang=[zeros(1,Nr); ang'; ang(:,6)']';
vel=[zeros(1,Nr); vel'; vel(:,6)']';
acc=[zeros(1,Nr); acc'; acc(:,6)']';
%the center of mass of each link
Pc1=[0; 0; 0]; %base frame {0}
Pc2=[-L1; 0 ;0];
Pc3=[-L1; 0 ;0];
Pc4=[-L1; 0 ;0];
Pc5=[-L1; 0; -L4/2];
Pc6=[-L1; 0; -L4-L5/2];
Pc7=[-L1; 0; -L4-L5/2];
% Pc8=[-L1; 0; -L4-L5/2-L7];
Pc=[zeros(1,3); Pc1'; Pc2'; Pc3'; Pc4'; Pc5'; Pc6'; Pc7']';


%=========initialize variables===========================
f=zeros(3,8); n=zeros(3,8); %f(:,7)=0; n(:,7)=0; no forces act on the end effector
w=zeros(3,8); w_dot=zeros(3,8); %w(:,1)=0; w_dot(:,1)=0; the base frame of the robot is not rotating
v=zeros(3,8); v_dot=zeros(3,8); v_dot(:,1)=[0; 0; g]; %include gravity force
vc_dot=zeros(3,8);
P=zeros(4,7);
F=zeros(3,8); N=zeros(3,8);
tau=zeros(Nr,7);
Z0=[0; 0; 1];

%========= the moment of inertia ===========================
m=[100 30 30 30 30 30 20 0]; %mass of each link
L=[0 L1 L1 L1 L4 L5 L5 L7]; %link length
r=0.115;
I=zeros(3,3);
for i=2:8
I(:,:,i)=diag([m(i)*(3*r^2+L(i)^2)/12, m(i)*(3*r^2+L(i)^2)/12, m(i)*r^2/2]); % moment of inertia of cylinders
end
%=========calculate joint torques at each time t
for t=1:Nr  
    %rotation between successive link frames R^{i}_{i+1}
    clear R P
    R=Rotation(0, 0, ang(t,1));
    R(:,:,2)=Rotation(pi/2, 0, pi/2+ang(t,2));
    R(:,:,3)=Rotation(pi/2, 0, ang(t,3));
    R(:,:,4)=Rotation(0, 0, ang(t,4));
    R(:,:,5)=Rotation(-pi/2, 0, ang(t,5));
    R(:,:,6)=Rotation(0, 0, ang(t,6));
    R(:,:,7)=eye(3,3);
    %position vector P^{i}_{i+1}
    P0=[0; 0; 0; 1];
    T01=DHF([0, -L1, 0, ang(t,1)]); P(:,1)=T01*P0; 
    T12=DHF([pi/2, 0, 0, pi/2+ang(t,2)]); P(:,2)=T12*P0;
    T23=DHF([pi/2, 0, 0, ang(t,3)]); P(:,3)=T23*P0;
    T34=DHF([0, -L4, 0, ang(t,4)]); P(:,4)=T34*P0;
    T45=DHF([0, -L5, 0, ang(t,5)]); P(:,5)=T45*P0;
    T56=DHF([pi/2, 0, 0, ang(t,6)]); P(:,6)=T56*P0;
    P(:,7)=DHF([0, -L7, 0, 0])*P0;
    P(4,:)=[];
%====outward iteration for link 1======
    for i=1:7      
        w(:,i+1)=R(:,:,i)'*w(:,i)+vel(t,i+1)*Z0;
        w_dot(:,i+1)=R(:,:,i)'*w_dot(:,i)+R(:,:,i)'*cross(w(:,i),vel(t,i+1)*Z0)+acc(t,i+1)*Z0;
        v(:,i+1)=R(:,:,i)'*(cross(w_dot(:,i),P(:,i))+cross(w(:,i),cross(w(:,i),P(:,i)))+v_dot(:,i));
        vc_dot(:,i+1)=cross(w_dot(:,i+1),Pc(:,i+1))+cross(w(:,i+1),cross(w(:,i+1),Pc(:,i+1)))+v_dot(:,i+1);
        F(:,i+1)=m(i+1)*vc_dot(:,i+1);
        N(:,i+1)=I(:,:,i+1)*w_dot(:,i+1)+cross(w(:,i+1),I(:,:,i+1)*w(:,i+1));
    end
%====inward iteration for link 1======
    for i=7:-1:1
        f(:,i)=R(:,:,i)*f(:,i+1)+F(:,i);
        n(:,i)=N(:,i)+R(:,:,i)*n(:,i+1)+cross(Pc(:,i),F(:,i));
        tau(t,i)=n(:,i)'*Z0;
    end

end








