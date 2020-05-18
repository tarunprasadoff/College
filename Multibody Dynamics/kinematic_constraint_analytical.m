clear all
close all
% This file determines the kinematic 
% constraint equations in analytical form
% O is common point in link 1 and link 2 
syms l1 l2 l4 
syms theta2 x2 y2 x3 y3 theta3 x4 y4 theta4
%syms x2(t) y2(t) theta2(t) x3(t) y3(t) theta3(t) x4(t) y4(t) theta4(t)
O1 = [0; 0] ; % Coordinates of O in coordinate system 1 
O2 = [-l2/2; 0] ; % Coordinates of O in coordinate system 2 
eq1= loc2global(theta2)*O2+[x2; y2]-O1
% A is a common point in link2  and link3 
A2 = [l2/2; 0] ; % coordinates of A in coordinate system 2 
A3 = [0; 0] ; % coordinates of A in coordinate system 3 
lhseq2 = loc2global(theta2)*A2+[x2; y2] ; 
rhseq2 = loc2global(theta3)*A3+[x3; y3] ; 
eq2 = lhseq2-rhseq2 
% Q is a common point in link4 and link 1 
Q1 = [0; -l1] ; % coordinates of Q in the local coordinate system 1 
Q4 = [-l4/2; 0] ; % coordinates of Q in the local coordinate system 4 
lhseq3 = loc2global(theta4)*Q4+[x4; y4] ; 
rhseq3 = Q1 ;
eq3 = lhseq3-rhseq3 

% sliding vector in body 3 
u3 = [1; 0] ; 
% transform the vector into  global coordinate system 
u1 = loc2global(theta3)*u3 ;
% point on body 3 on the sliding axis = A
A3 = [0; 0] ; % in the coordinate system 3 
A1 = [x3; y3] ; % in the coordinate system 1 
% point on body 4 on the sliding axis = Q 
Q1_n =  loc2global(theta4)*Q4+[x4; y4] ; 
v1 = A1-Q1_n ; % this is also a sliding vector in the coordinate system 1
eq4 = tildeop(u1)'*v1
