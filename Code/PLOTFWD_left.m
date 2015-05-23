function PLOTFWD_right(ang)
%input:     ang=[theta1, theta2, theta3, theta4, theta5, theta6]^T, where
%           theta1, theta2, theta3, theta4, theta5 and theta6 are six joint angles.   
%output:    (a) plot a complete "stick figure" of the robot 
%           (b) overlay plots of the individual link frames on the stick figure
%author:    Yan Yan
theta1=ang(1);
theta2=ang(2);
theta3=ang(3);
theta4=ang(4);
theta5=ang(5);
theta6=ang(6);

 T1=DHF([0, -0.125, 0, theta1]); % homogeneous transformation from base frame to Frame {1}
 T2=DHF([pi/2, 0, 0, pi/2+theta2]); % homogeneous transformation from Frame {1} to Frame {2}
 T3=DHF([pi/2, 0, 0, theta3]); % homogeneous transformation from Frame {2} to Frame {3}
 T4=DHF([0, -0.46, 0, theta4]); % homogeneous transformation from Frame {3} to Frame {4}
 T5=DHF([0, -0.467, 0, theta5]); % homogeneous transformation from Frame {4} to Frame {5}
 T6=DHF([-pi/2, 0, 0, theta6]); % homogeneous transformation from Frame {5} to Fame {6}
 T7=DHF([0, -0.15, 0, 0]); % homogeneous transformation from Frame {6} to tool frame
 
x0=[0; 0; 0; 1]; 
P1=T1*x0; % the origin of Frame {1} written in base frame
P2=T1*T2*x0; % the origin of Frame {2} written in base frame
P3=T1*T2*T3*x0; % the origin of Frame {3} written in base frame
P4=T1*T2*T3*T4*x0; % the origin of Frame {4} written in base frame
P5=T1*T2*T3*T4*T5*x0; % the origin of Frame {5} written in base frame
P6=T1*T2*T3*T4*T5*T6*x0; % the origin of Frame {6} written in base frame
P7=T1*T2*T3*T4*T5*T6*T7*x0; % the origin of tool frame written in base frame
%========plot a complete "stick figure" of the robot ===========
plotv4(x0,P1); %plot from the origin of base frame to Frame {1}
hold on; plotv4(P1,P2); %plot from the origin of Frame {1} to {2}
hold on; plotv4(P2,P3); %plot from the origin of Frame {2} to {3}
hold on; plotv4(P3,P4); %plot from the origin of Frame {3} to {4}
hold on; plotv4(P4,P5); %plot from the origin of Frame {4} to {5}
hold on; plotv4(P5,P6); %plot from the origin of Frame {5} to {6}
hold on; plotv4(P6,P7); %plot from the origin of Frame {6} to {7}
% %========plot the individual link frames on the stick figure ===========
% hold on; plotsmf([1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]); %base frame
% hold on; plotsmf(T1); %Frame {1}
% hold on; plotsmf(T1*T2); %Frame {2}
% hold on; plotsmf(T1*T2*T3); %Frame {3}
% hold on; plotsmf(T1*T2*T3*T4); %Frame {4}
% hold on; plotsmf(T1*T2*T3*T4*T5); %Frame {5}
% hold on; plotsmf(T1*T2*T3*T4*T5*T6); %Frame {6}
% hold on; plotsmf(T1*T2*T3*T4*T5*T6*T7); %Frame {7}
grid on;
