clear all
close all
% This file determines the kinematic 
% constraint equations in analytical form
% O is common point in link 1 and link 2 
syms l r e 
syms xa ya xb
O = [0; 0] ; % Coordinates of O 
A = [xa; ya] ; % Coordinates of A 
dOA=A-O;
eq1= dOA'*dOA-r^2
B = [xb; e] ; % Coordinates of B
dAB=B-A;
eq2= dAB'*dAB-l^2



