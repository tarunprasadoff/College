clear all
global r
r=3.8; %mu=r/4
x=[0:0.00001:1];
figure(2); hold off
plot(x,lgt(x),'b');hold on;plot(x,x,'r');hold off
%plot(x,lgt(lgt(lgt(lgt(lgt(lgt(lgt(lgt(x)))))))),'b');hold on;plot(x,x,'r')
