function xdot = doub_pend_func(t, x)

global k l2 l3 m2 m3 J2 J3 c l0 g
x = transpose(x) ; 
x_2 = x(1) ; y_2 = x(2); theta_2 = x(3) ; 
x_3 = x(4) ; y_3 = x(5); theta_3 = x(6) ; 
for count = 1:6
    xdot(count) = x(6+count) ; 
end 

load formulation

x2dotdot = double(subs(qdotdot(1))); 
y2dotdot = double(subs(qdotdot(2))); 
theta2dotdot = double(subs(qdotdot(3))) ; 
x3dotdot = double(subs(qdotdot(4))) ; 
y3dotdot = double(subs(qdotdot(5))) ;
theta3dotdot = double(subs(qdotdot(6))) ; 

xdot(7) = x2dotdot ; 
xdot(8) = y2dotdot ; 
xdot(9) = theta2dotdot ; 
xdot(10) = x3dotdot ; 
xdot(11) = y3dotdot ; 
xdot(12) = theta3dotdot ; 

xdot = transpose(xdot) ;
