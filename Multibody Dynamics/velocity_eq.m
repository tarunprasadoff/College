clear all
syms l1 l2 l4 
syms x2(t) y2(t) theta2(t) x3(t) y3(t) theta3(t) x4(t) y4(t) theta4(t)

syms x2dot y2dot theta2dot x3dot y3dot theta3dot x4dot y4dot theta4dot

O1 = [0; 0] ; % Coordinates of O in coordinate system 1 
O2 = [-l2/2; 0] ; % Coordinates of O in coordinate system 2 
eq1= loc2global(theta2(t))*O2+[x2(t); y2(t)]-O1; 

% A is a common point in link2  and link3 
A2 = [l2/2; 0] ; % coordinates of A in coordinate system 2 
A3 = [0; 0] ; % coordinates of A in coordinate system 3 
lhseq2 = loc2global(theta2(t))*A2+[x2(t); y2(t)] ; 
rhseq2 = loc2global(theta3(t))*A3+[x3(t); y3(t)] ; 
eq2 = lhseq2-rhseq2  ; 

% Q is a common point in link4 and link 1 
Q1 = [0; -l1] ; % coordinates of Q in the local coordinate system 1 
Q4 = [-l4/2; 0] ; % coordinates of Q in the local coordinate system 4 
lhseq3 = loc2global(theta4(t))*Q4+[x4(t); y4(t)] ; 
rhseq3 = Q1 ;
eq3 = lhseq3-rhseq3 ;

% sliding vector in body 3 
u3 = [1; 0] ; 
% transform the vector into  global coordinate system 
u1 = loc2global(theta3(t))*u3 ;
% point on body 3 on the sliding axis = A
A3 = [0; 0] ; % in the coordinate system 3 
A1 = [x3(t); y3(t)] ; % in the coordinate system 1 
% point on body 4 on the sliding axis = Q 
Q1_n =  loc2global(theta4(t))*Q4+[x4(t); y4(t)] ; 
v1 = A1-Q1_n ; % this is also a sliding vector in the coordinate system 1
eq4 = [-u1(2), u1(1)]*v1 ; 
eq5 = theta3(t)-theta2(t); 

eq = [eq1 ; eq2; eq3; eq4; eq5] ; 
% these are the kinematic constraint equations
for count = 1:length(eq)
    veq(count) = diff(eq(count), t) ; 
    % differentiate each equation to determine the velocity constraint
    % equation
end 
% Next recast the velocity equation into a system of linear equations 
% and find the Jacobian matrix (required for subsequent calculations)
veleq = subs(veq, [diff(x2(t), t), diff(y2(t), t), diff(theta2(t), t), ...
                      diff(x3(t), t), diff(y3(t), t), diff(theta3(t), t), ...
                      diff(x4(t), t), diff(y4(t), t), diff(theta4(t), t)],...
                      [x2dot, y2dot, theta2dot, x3dot, y3dot, theta3dot, ...
                      x4dot, y4dot, theta4dot]) ;
  r = [x2dot, y2dot, theta2dot, x3dot, y3dot, theta3dot, x4dot, y4dot, theta4dot];                  
                  
for count = 1:length(veleq) 
     veqn = veleq(count)  ;
    for ijk = 1:length(r)         
       c = coeffs(veqn, r(ijk));  
       s = size(c) ; 
       if s(2)==2
       Dval(count, ijk) = c(2); 
       else
       Dval(count, ijk) = 0 ; 
       end
       clear c
    end 
end 
% the matrix Dval is the Jacobian Matrix