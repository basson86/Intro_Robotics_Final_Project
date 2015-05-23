% Matlab script for Lab04 1(b)
% Dynamics Simulation
% 2010/11/30--commented by Tze-Yuan Cheng

clear all;

% system constants
m = 1; % in kg
l= 0.467; % m


% Structure K value (in the spring-mass model)
Kp1=469.4;
Kp2=322.3;

% Kv value for PD controller
% before applying the D control , Kv=0
Kv1=0;
Kv2=0;
% Conditions for critical damping : Kv = 2*sqrt(Kp)
Kv1c= 2*sqrt(Kp1)
Kv2c= 2*sqrt(Kp2)


t_inc = 0.01; % in seconds
t_f = 1; % in seconds

% initial conditions of joints 1 and 2 (Roll & Pitching) without applying D
% controller
u1= 2;
u2= 1;
p1 = 4;	% intial positions
p2 = 2;
v1 = 0;	% initial velocities
v2 = 0;
v1_last = 0;	% storage for numerical integration
v2_last = 0;
a1_last = 0;
a2_last = 0;

% initial condition for joints 1 and 2 (Roll & Pitching) after applying D controller
p1c = 4;	% intial positions
p2c = 2;
v1c = 0;	% initial velocities
v2c = 0;
v1c_last = 0;	% storage for numerical integration
v2c_last = 0;
a1c_last = 0;
a2c_last = 0;

i = 0;

for t=0:t_inc:t_f % in seconds
   i = i+1;
   
   % store p1 & p2 (without control),p1c & p2c (with control) into the array of d1 & d2,d1c & d2c for plotting
   d1(i) = p1;
   d2(i) = p2;
   d1c(i) = p1c;
   d2c(i) = p2c;

   
   % compute acceleration with/without applying derivative controller
   a1 = (-Kp1*(p1-u1)-Kv1*(v1))
   a2 = (-Kp2*(p2-u2)-Kv2*(v2));
   
   a1c = (-Kp1*(p1c-u1)-Kv1c*(v1c))
   a2c = (-Kp2*(p2c-u2)-Kv2c*(v2c));
   
   
   % integrate to obtain new position and velocity with/without applying
   % Derivative controller
   v1 = v1 + 0.5*(a1_last + a1)*t_inc;
   v2 = v2 + 0.5*(a2_last + a2)*t_inc;
   p1 = p1 + 0.5*(v1_last + v1)*t_inc;
   p2 = p2 + 0.5*(v2_last + v2)*t_inc;
   
   v1c = v1c + 0.5*(a1c_last + a1c)*t_inc;
   v2c = v2c + 0.5*(a2c_last + a2c)*t_inc;
   p1c = p1c + 0.5*(v1c_last + v1c)*t_inc;
   p2c = p2c + 0.5*(v2c_last + v2c)*t_inc;
 
 
   
   % update last position and acceleration for each link with/without
   % applying derivative controller
   v1_last = v1;
   v2_last = v2;
   a1_last = a1;
   a2_last = a2;   
   
   v1c_last = v1c;
   v2c_last = v2c;
   a1c_last = a1c;
   a2c_last = a2c;  
   

end

%plot the values of d1,d2,d1c,d2c over time
figure(2);
plot(0:t_inc:t_f,d1,'--b');
hold on;
plot(0:t_inc:t_f,d2,'--r');
hold on;
plot(0:t_inc:t_f,d1c,'b');
hold on;
plot(0:t_inc:t_f,d2c,'r');

hx=xlabel('time (sec)');
hy=ylabel('joint angle (rad)');
h = legend('Roll (w/ PD controller)','Pitching','Roll (w/ PD controller)','Pitching',2);
% ht=title('Roll & Pitching angle over time');
hold on;


