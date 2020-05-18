function PE=PE(P,i,j)
L3=P(1);
L4 = P(2);
K1 = P(3);
d = P(4);
alpha = i*(pi/180) ;
beta = j*(pi/180) ;

L1 = 200;
L2 = 100;
M1 = 0.18;
M2 = 0.09;
M3 = 0.15;
M4 = 0.15;
g = 9800;

L0 = ((L3 - L1)^2 + d^2)^.5;
theta =  alpha + atan((-L1*sin(alpha) + L3)/(d + L1*cos(alpha)));
y = (d + L1*cos(alpha))/(cos(theta-alpha)) - L0 ;
delta = theta + beta + asin(-sin(theta + beta)*(L0+y)/L4);
PE = real((M1*g*L1*sin(alpha)/2) + (M2*g*(L1*sin(alpha)-L2/2*sin(alpha+beta))) + (M4*g*(L3+L4/2*sin(delta+alpha-theta))) + (M3*g*(L3-((L0+y)/2)*sin(theta-alpha))) +K1*y*y/2);
end