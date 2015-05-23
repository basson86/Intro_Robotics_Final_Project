function[P7]= FWD(V_J)
% 2010/10/06--commented by Tze-Yuan Cheng
% Using DH parameters, compute the homogeneous tranformation matrix for each link frame
% upright configurations : V_J=[0;0;0;0;0;0];


Tr0= eye(4); % base frame: Identity Matrix
Tr1= Tr0*DHF([0,-0.125,0,V_J(1,:)]');
Tr2= Tr1*DHF([pi/2,0,0,(pi/2+V_J(2,:))]');
Tr3= Tr2*DHF([pi/2,0,0,V_J(3,:)]');
Tr4= Tr3*DHF([0,-0.46,0,V_J(4,:)]');
Tr5= Tr4*DHF([0,-0.467,0,V_J(5,:)]');
Tr6= Tr5*DHF([-pi/2,0,0,V_J(6,:)]');
Tr7= Tr6*DHF([0,-0.15,0,0]');

% Get the origin point of each frame with respect to the base frame
O0=Tr0(:,4);
O1=Tr1(:,4);
O2=Tr2(:,4);
O3=Tr3(:,4);
O4=Tr4(:,4);
O5=Tr5(:,4);
O6=Tr6(:,4);
O7=Tr7(:,4);

P7=O7(1:3,:);
% using plotv4, plot the link by connecting the origin from frame i-1 to 
% the origin of frame i
% plotv4(O0,O1,'k');
% hold on;
% plotv4(O1,O2,'r');
% hold on;
% plotv4(O2,O3,'g');
% hold on;
% plotv4(O3,O4,'k');
% hold on;
% plotv4(O4,O5,'m');
% hold on;
% plotv4(O5,O6,'b');
% hold on;
% plotv4(O6,O7,'r');
% % using plotsmf, plot the link by connecting the origin from frame i-1 to
% % the origin of frame i
% plotsmf(Tr0);
% hold on;
% plotsmf(Tr1);
% hold on;
% plotsmf(Tr2);
% hold on;
% plotsmf(Tr3);
% hold on;
% plotsmf(Tr4);
% hold on;
% plotsmf(Tr5);
% hold on;
% plotsmf(Tr6);
% hold on;
% plotsmf(Tr7);
% hold on;
% axis equal;
% 
% % label the length unit: meters
% xlabel('x(m)');
% ylabel('y(m)');
% zlabel('z(m)');
