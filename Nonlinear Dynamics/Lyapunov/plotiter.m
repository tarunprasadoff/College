r=0.628; %mu=r/4
x=0:0.00001:1;
figure(2); hold off
plot(x,itermap(x,r),'b');hold on;plot(x,x,'r');hold off