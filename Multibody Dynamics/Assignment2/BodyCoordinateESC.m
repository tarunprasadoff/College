clear all
close all
% This file determines the kinematic 
% constraint equations in analytical form
% O is common point in link 1 and link 2 
syms r l e 
syms theta2 x2 y2 x3 y3 theta3 x4 theta4
O1 = [0; 0] ; % Coordinates of O in coordinate system 1 
O2 = [-r/2; 0] ; % Coordinates of O in coordinate system 2 
eq1= loc2global(theta2)*O2+[x2; y2]-O1
% A is a common point in link2  and link3 
A2 = [r/2; 0] ; % coordinates of A in coordinate system 2 
A3 = [-l/2; 0] ; % coordinates of A in coordinate system 3 
lhseq2 = loc2global(theta2)*A2+[x2; y2] ; 
rhseq2 = loc2global(theta3)*A3+[x3; y3] ; 
eq2 = lhseq2-rhseq2 
% B is a common point in link4 and link 3 
B3 = [l/2; 0] ; % coordinates of B in the local coordinate system 3
B4 = [0; 0] ; % coordinates of B in the local coordinate system 4 
lhseq3 = loc2global(theta4)*B4+[x4; e] ; 
rhseq3 = loc2global(theta3)*B3+[x3; y3] ;
eq3 = lhseq3-rhseq3 


