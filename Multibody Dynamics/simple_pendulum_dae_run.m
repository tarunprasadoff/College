clear all
% to solve the pendulum problem as a DAE 
global m l g

m = 1; l=1 ; g = 1 ;
x0 = 0 ; y0 = l; xdot0 = 10 ; ydot0 = 0 ; 
T0 = (m/l)*(g*y0+xdot0^2+ydot0^2) ; 

M = [1, 0, 0, 0, 0 ; 0, 1, 0, 0, 0; 0, 0, 1, 0, 0; 0, 0, 0, 1, 0; 0, 0, 0, 0, 0];

% y0 = [1; 0; 0];
% tspan = [0 4*logspace(-6,6)];
% M = [1 0 0; 0 1 0; 0 0 0];

options = odeset('Mass',M,'RelTol',1e-9,'AbsTol',[1e-12 1e-12 1e-12 1e-12 1e-12]);

options2 = odeset('Reltol',1e-6,'Abstol',[1e-12, 1e-12]); 

Tspan = [0, 20] ; 

[t,y] = ode15s(@pend_dae, Tspan,[x0; y0; xdot0; ydot0; T0],options);
%ode15s and ode23t both work
[t2, sol2] = ode45(@pendulum_ode, Tspan, [0; sqrt(xdot0^2+ydot0^2)/l], options2) ; 
x = l*sin(sol2(:, 1)); 
ysol = l*cos(sol2(:, 1));

subplot(211)
plot(t, y(:, 1)); hold on ; plot(t2, x, 'r.') ; grid on 
title('Comparison of ODE and DAE solution')
xlabel('Time'); ylabel('x')
legend('DAE solution', 'ODE solution')

subplot(212)
plot(t, y(:, 2)); hold on ; plot(t2, ysol, 'r.') ; grid on 
xlabel('Time'); ylabel('y')
legend('DAE solution', 'ODE solution')