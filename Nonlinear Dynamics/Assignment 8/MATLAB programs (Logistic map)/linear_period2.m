clear all
alpha=pi;
t=[0:0.01:80*pi];
x=0.8*cos(alpha*t)-sin(t);
xdot=-0.8*alpha*sin(alpha*t)+cos(t);
figure(1);plot(x,xdot);hold on;plot(0.8,1,'ro');hold off
figure(2);plot(t,x)