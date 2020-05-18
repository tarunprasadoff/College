%% Generating Obstacles
obs=[0,0;25,0;50,0;75,0;0,25;25,25;50,25;75,25;0,50;25,50;50,50;75,50;0,75;25,75;50,75;75,75]+5*randi([1,5],[16,2]);
gen=obs/5;
obs=-2.5*ones(16,2)+obs;
plot(obs(:,1),obs(:,2), 's', 'MarkerSize',20);
hold on;
axis([0 100 0 100]);
%% State Initialisation and Obstacle Memory
x=50;
y=20;
dest=[100;100];
next=((gen(:,1)-1)*20)+(20-(gen(:,2)-1));
od=zeros(20);
mem=od;
od(next)=1;
%[dist,point]=ob(0,0,pi/6,obs)
% mem=upd(mem,od,35,25);
%% Defining Parameters and Weights
Wo=[10,10,25,25];
% Wo5=0;
% Wo6=0;
Wi=[.005,.005];
% duMax=.25;
% duMin=-.25;
uMax=2.5;
uMin=-2.5;
%% Initialisation
uPrev=[0;0];
maxTime=100;
Y_SAVE1=zeros(maxTime+1,1);
Y_SAVE2=zeros(maxTime+1,1);
Y_SAVE3=zeros(maxTime+1,1);
Y_SAVE4=zeros(maxTime+1,1);
% Y_SAVE5=zeros(maxTime+1,1);
% Y_SAVE6=zeros(maxTime+1,1);
T_SAVE=zeros(maxTime+1,1);
U_SAVE1=zeros(maxTime,1);
U_SAVE2=zeros(maxTime,1);
%% Implementation
for k=1:maxTime+1
    mem=upd(mem,od,x,y);
    ySP=compSP(dest,x,y,mem);
    Y_SAVE1(k)=x;
    Y_SAVE2(k)=y;
    fun=@(u)(Wo*((ySP-[x+u(1);y+u(2);x+u(1);y+u(2)]).^2))+Wi*[u(1);u(2)].^2;
    u = fmincon(fun,[0;0],[],[],[],[],[uMin;uMin],[uMax;uMax]);
    x=x+u(1);
    y=y+u(2);
    T_SAVE(k+1)=k;
end
%% Plotting
plot(Y_SAVE1,Y_SAVE2);
%% Obstacle Memory Update
function memnew=upd(mem,od,x,y)
    [x,y]=trans(x,y);
     if y==1
        ily=0;
    elseif y==2
        ily=1;
    else
        ily=2;
    end
    if y==20
        igy=0;
    elseif y==19
        igy=1;
    else
        igy=2;
    end
    if x==1
        ilx=0;
    elseif x==2
        ilx=1;
    else
        ilx=2;
    end
    if x==20
        igx=0;
    elseif x==19
        igx=1;
    else
        igx=2;
    end
    mem(y-ily:y+igy,x-ilx:x+igx)=od(y-ily:y+igy,x-ilx:x+igx);
    memnew=mem;
end
%% Setpoint Function
function ySP=compSP(dest,x,y,mem)
    [x,y]=trans(x,y);
    ySP=[dest;0;0];
    iy=find(mem(x,:)==1);
    iyl=find(iy<y, 1, 'last');
    iyg=find(iy>y, 1, 'first');
    if (size(iyl)==[1,0])
        iyl=1;
    elseif (size(iyl)==0)
        iyl=1;
    end
    if (size(iyg)==[1,0])
        iyg=20;
    elseif (size(iyg)==0)
        iyg=20;
    end
    ySP(4)=(21-(iyg+iyl)/2)*5;
    ix=find(mem(y,:)==1);
    ixl=find(ix<x, 1, 'last');
    ixg=find(ix>x, 1, 'first');
    if (size(ixl)==[1,0])
        ixl=1;
    elseif (size(ixl)==0)
        ixl=1;
    end
    if (size(ixg)==[1,0])
        ixg=20;
    elseif (size(ixg)==0)
        ixg=20;
    end
    ySP(3)=(-1+(ixg+ixl)/2)*5;
end
%% Transform Fn.
function [xn,yn]=trans(x,y)
     if (rem(([x,y]/5),1)==0)
        xn=x/5+1;
        yn=20-(y/5);
    else
        p=ceil(([x,y]/5));
        xn=p(1);
        yn=20-p(2);
    end
    xn=min(xn,20);
    yn=min(yn,20);
    xn=max(xn,1);
    yn=max(yn,1);
end