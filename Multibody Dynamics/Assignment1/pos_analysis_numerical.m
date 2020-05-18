clear all
global theta2 
l1 = 10; l2 = 2 ; l4 = 15 ; 
% loop over a range of theta2
theta2r = pi/2:pi/20:9*pi/2; 
x0 = [0; 1; 0; 2; pi/2; 0; -2.5; pi/2];
ansr = [] ; eflag = [] ; 

vidObj = VideoWriter('motion.avi');
vidObj.FrameRate=15;     
open(vidObj);
    
for count = 1:length(theta2r)
theta2 = theta2r(count) ; 
 
[x, fval, exitflag] = fsolve('numeqneval', x0, optimoptions('fsolve','Display','off')); 
ansr = [ansr; transpose(x)] ;
eflag = [eflag; transpose(exitflag)] ; 
x0 = x ;
% ================ Animation of the mechanism =====
 x2 = x(1) ; y2 = x(2); % theta2 is already available
 x3 = x(3); y3 = x(4); theta3 = x(5) ; 
 x4 = x(6); y4 = x(7); theta4 = x(8) ; 
 
% % local coordinates of A in body 2 
 A2 = [l2/2; 0] ; 
% % global coordinates of A 
 A1 = loc2global(theta2)*A2+[x2; y2] ; 
 % A1 = 2*[x2; y2] ; 
 % % local coordinates of B in body 4 
 B4 = [l4/2; 0] ; 
% % global coordinates of B 
 B1 = loc2global(theta4)*B4+[x4; y4] ; 
% % global coordinates of Q 
 Q = [0; -l1]; 
 clf
 line([0, A1(1)], [0, A1(2)]) ;
hold on 
line([Q(1), B1(1)], [Q(2), B1(2)], 'color', 'r')
  axis equal
  axis([-5, 5, -10, 8]);
   currFrame = getframe;
    writeVideo(vidObj,currFrame); 
  pause(0.2)
% ========================================
end
close(vidObj);
