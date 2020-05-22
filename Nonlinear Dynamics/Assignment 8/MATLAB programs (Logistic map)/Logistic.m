clear all
global r
mu=0.893;
nmax=100;
x=zeros(nmax,1);
gcf=figure(2);
hold off
x(1)=0.2;i=1;eps=1;
r=4*mu;
while and(eps>1e-16,i<nmax)
    i=i+1;
    x(i)=logist(x(i-1));
    eps=abs(x(i)-x(i-1));
end
plot([1:i],x(1:i),'o','MarkerSize',5)
hold off
x0=x;
x(1)=0.2+1e-8;i=1;eps=1;
r=4*mu;
while and(eps>1e-16,i<nmax)
    i=i+1;
    x(i)=logist(x(i-1));
    eps=abs(x(i)-x(i-1));
end
semilogy([1:i],abs(x0(1:i)-x(1:i)),'o','MarkerSize',5)
hold off