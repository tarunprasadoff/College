clear all
rmin=0;
rmax=1; %r=4*mu;
r=[rmin:0.0001:rmax];r=r';
nseed=200; %Number of seeds
nstep=1000; %Number of iterations
[sr dummy]=size(r);
xprev=zeros(nseed,sr);
x=rand(nseed,sr);
err=zeros(nseed,sr);
%M=zeros(nstep);
gcf=figure(1);
for n=2:nstep
    %x(:,n)=r.*x(:,n-1).*(1-x(:,n-1));
    plot(r,x,'.','MarkerSize',3)
    str={'n=',n};
    text((rmin+rmax)/2,0.9,str)
    axis([rmin rmax 0 1])
    drawnow;
    %M(n)=getframe(gcf);
    xprev=x;
    err=abs(xprev-x);
    x=r'.*sin(pi.*x);
end
%fig=figure;
%movie(fig,M,2)