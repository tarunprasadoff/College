function thetadot = pendulum_ode(t, theta) 
global l g 
thetadot1 = theta(2) ; 
thetadot2 = -(g/l)*sin(theta(1)) ; 
thetadot = [thetadot1; thetadot2] ; 