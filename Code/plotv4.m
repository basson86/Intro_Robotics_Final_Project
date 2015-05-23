% 
% usage: 	plotv4(X0_vec, X1_vec);
% input: 	X0_vec 4x1 vector e.g.: X0_vec = [1;0;0;1]
%        	X1_vec 4x1 vector e.g.: X1_vec = [1;1;0;1]
%
%		optional additional argument: 
%               plotv4(X0_vec, X1_vec, 'r');
%
% output:	plots a vector starting at X0_vec to X1_vec
% in the example it will be a RED vector parallel to the y-axis
% starting 	at x=1,y=0,z=0 
% ending 	at x=1,y=1,z=0
% 

function plotv4(X0_vec, X1_vec, linespec)

% check argument dimension
[rows, cols] = size(X0_vec);
if ((rows ~= 4) | (cols ~= 1))
  error('PLOTV4 requires a 4x1 vector argument. Check your dimensions.');
end

% check argument dimension
[rows, cols] = size(X1_vec);
if ((rows ~= 4) | (cols ~= 1))
  error('PLOTV4 requires a 4x1 vector argument. Check your dimensions.');
end

% assemble a matrix with the two position vectors
m = [X0_vec(1:3,1)'; X1_vec(1:3,1)'];

% check if the given value for the linestyle exists
if 0 == exist('linespec')
   linespec = 'r';
end;

plot3(m(:,1), m(:,2), m(:,3));
r1=[m(1,1); m(1,2); m(1,3)]';
n=length(m(:,1));
r2=[m(n,1); m(n,2); m(n,3)]';
% cylinder2P([0.05 0.04 0.05],5,r1,r2);
% cylinder2P(0.05,5,r1,r2);
Cylinder(r1,r2,0.05,10,'b',0,lines);
% h=gcf;
% cylinder(h);


