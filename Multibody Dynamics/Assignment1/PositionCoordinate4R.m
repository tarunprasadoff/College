clear all
close all
% This file determines the kinematic 
% constraint equations in analytical form
% O is common point in link 1 and link 2 
syms l1 l2 l3 l4 
syms xa ya xb yb
O = [0; 0] ; % Coordinates of O 
A = [xa; ya] ; % Coordinates of A 
dOA=A-O;
eq1= dOA'*dOA-l2^2
B = [xb; yb] ; % Coordinates of B
dAB=B-A;
eq2= dAB'*dAB-l3^2
Q = [l4; 0] ; % Coordinates of C
dBQ=Q-B;
eq3= dBQ'*dBQ-l4^2


