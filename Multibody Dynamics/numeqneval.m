function y = numeqneval(x)
global theta2
l1 = 10 ; l2 = 2 ; l4 = 15 ; % kinematic dimensions given numerically

% declaration of the variables of the input vector
x2 = x(1); y2 = x(2);  
x3 = x(3); y3 = x(4); theta3 = x(5); 
x4 = x(6); y4 = x(7); theta4 = x(8); 

% these are the equations copy-pasted from the symbolic computation
eq1a = x2 - (l2*cos(theta2))/2 ; 
eq1b = y2 - (l2*sin(theta2))/2 ; 

eq2a = x2 - x3 + (l2*cos(theta2))/2 ; 
eq2b = y2 - y3 + (l2*sin(theta2))/2 ; 

eq3a = x4 - (l4*cos(theta4))/2 ; 
eq3b = l1 + y4 - (l4*sin(theta4))/2 ; 

eq4a = cos(theta3)*(y3 - y4 + (l4*sin(theta4))/2) - ...
sin(theta3)*(x3 - x4 + (l4*cos(theta4))/2); 
eq4b = theta3-theta4 ; 
% this is the output vector
% for the root the output vector should be zero
y = [eq1a; eq1b; eq2a; eq2b; eq3a; eq3b; eq4a; eq4b] ; 