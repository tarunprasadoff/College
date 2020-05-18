clear all
close all
global k l2 l3 m2 m3 J2 J3 c l0 g 

g = 1 ; 
k = 500 ; l2 =1; l3 = 1;
m2=0.2; m3 = 0.15; 
J2 = m2*l2*l2/12 ;
J3 = m3*l3*l3/12 ;
c = 20000 ; 
l0 = 0.2 ; 

x0 = [0 ; -l2/2-l2/10; 3*pi/2; 0; -l2-l3/2-l3/5; 3*pi/2] ; 
v0 = [0; 0; 1; 0; 0; 2] ;

xin = [x0; v0] ; 
options = odeset('RelTol', 0.01, 'AbsTol', 1e-03); 
[t, x] = ode45('doub_pend_func', [0, 0.5], xin, options) ; 

% load handel
% sound(y, Fs)

% ==== Now the animation ==== 
x2v = x(:, 1) ; y2v = x(:, 2) ; theta2v = x(:, 3) ; 
x3v = x(:, 4) ; y3v = x(:, 5) ; theta3v = x(:, 6) ; 
rA_2 = [l2/2; 0] ; rB_3 = [-l3/2; 0] ; rC_2 = [-l2/2; 0] ; 
rD_3 = [l3/2; 0] ; 
for count = 1:length(t) 
    x2 = x2v(count) ; y2 = y2v(count) ; theta2 = theta2v(count) ; 
    x3 = x3v(count) ; y3 = y3v(count) ; theta3 = theta3v(count) ; 
    
    rA = loc2global(theta2)*rA_2+[x2; y2]; xA = rA(1); yA = rA(2) ; 
    rC = loc2global(theta2)*rC_2+[x2; y2]; xC = rC(1); yC = rC(2) ; 
    rB = loc2global(theta3)*rB_3+[x3; y3] ; xB = rB(1); yB = rB(2) ; 
    rD = loc2global(theta3)*rD_3+[x3; y3] ; xD = rD(1); yD = rD(2) ;
    clf
    line([xC, xA], [yC, yA]) ; 
    hold on ; 
    line([xB, xD], [yB, yD], 'color','r')
    patch([0, 0.01, 0.01, 0], [0 , 0, -0.01, -0.01], 'g')
    axis equal
  axis([-0.4, 0.4, -2.5, 0]);
    pause(0.5)
end 