function out = pend_dae(t, q) 

global m l g
out1 = q(3) ; 
out2 = q(4) ; 
out3 = -(q(5)/m)*(q(1)/l) ;
out4 = g - (q(5)/m)*(q(2)/l) ; 
%out5 = (m/l)*(g*q(2)+q(3)^2+q(4)^2)-q(5) ; 

out5 = (m*l)*(g*q(2)+q(3)^2+q(4)^2)/(q(1)^2+q(2)^2)-q(5); 
% this equation equates the tension in the string (q(5))
% to a function of the state variables. This expression 
% can be obtained by solving the MBD equations as a 
% matrix-algebra problem. 
out = [out1; out2; out3; out4; out5] ; 
