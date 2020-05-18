clc; clear;
t=[0 50];
options = odeset('RelTol',1e-5,'AbsTol',[1e-5 1e-5]);
n=0.2;
for i=-2:n:2
    for j=-2:n:2
    [t,x]=ode45(@model2,t,[i,j],options);
%     plot(x(:,1),x(:,2),'k','LineWidth',0.5);
    u = gradient(x(:,1));
    v = gradient(x(:,2));
    quiver(x(:,1),x(:,2),u,v,'k','LineWidth',0.5);
    hold on;
    end
end
axis equal;
xlim([-2 2]);
ylim([-2 2]);
str = '\textbf{Phase Portrait: $$ \frac{dx}{dt} = y $$; $$ \frac{dy}{dt} = x - x^3 $$}';
title(str,'Interpreter','latex');
xlabel('x','FontWeight','bold');
ylabel('y','FontWeight','bold');