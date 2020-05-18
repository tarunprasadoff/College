%% Generating Obstacles
length=100;
st=0;
gl=20;
gs=5;
os=25;
obs=[0,0;25,0;50,0;75,0;0,25;25,25;50,25;75,25;0,50;25,50;50,50;75,50;0,75;25,75;50,75;75,75]+5*randi([1,5],[16,2]);
gen=obs/5;
obs=-2.5*ones(16,2)+obs;
plot(obs(:,1),obs(:,2), 's', 'MarkerSize',10);
hold on;
axis([-50 150 -50 150]);
%% State Initialisation and Obstacle Memory
x=25;
y=-25;
ySP=[120;120];
next=((gen(:,1)-1)*20)+(20-(gen(:,2)-1));
od=zeros(20);
mem=od;
od(next)=1;
%[dist,point]=ob(0,0,pi/6,obs)
% mem=upd(mem,od,35,25);
%% Defining Parameters and Weights
Wo=[1,1,2];
% Wo5=0;
% Wo6=0;
Wi=[.05,.05]*0;
% duMax=.25;
% duMin=-.25;
uMax=2.5;
uMin=-2.5;
%% Initialisation
uPrev=[0;0];
maxTime=80;
Y_SAVE1=zeros(maxTime+1,1);
Y_SAVE2=zeros(maxTime+1,1);
Y_SAVE3=zeros(maxTime+1,1);
% Y_SAVE4=zeros(maxTime+1,1);
% Y_SAVE5=zeros(maxTime+1,1);
% Y_SAVE6=zeros(maxTime+1,1);
T_SAVE=zeros(maxTime+1,1);
U_SAVE1=zeros(maxTime,1);
U_SAVE2=zeros(maxTime,1);
%% Implementation
nonlcon=@mag;
for k=1:maxTime+1
    mem=upd(mem,od,x,y);
    [dist,a]=compOB(x,y,mem);
    Y_SAVE1(k)=x;
    Y_SAVE2(k)=y;
    fun=@(u)(Wo*[(ySP-[x+u(1);y+u(2)]).^2;(-1*(sum(a.*(dist-[u(1);u(2);-u(1);-u(2)]).^2)))])+(Wi*u).^2;
    u = fmincon(fun,[0;0],[],[],[],[],[],[],nonlcon);
    x=x+u(1);
    y=y+u(2);
    Y_SAVE3(k)=sum(a.*(dist-[u(1);u(2);-u(1);-u(2)]).^2);
    U_SAVE1(k)=u(1);
    U_SAVE2(k)=u(2);
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
function [dist,a]=compOB(x,y,mem)
    a=ones(4,1);
    [x,y]=trans(x,y);
    iy=find(mem(:,x)==1);
    iyl=iy(find(iy<y, 1, 'last'));
    iyg=iy(find(iy>y, 1, 'first'));
    if (size(iyl)==[0,1])
        iyl=1;
        a(2)=0;
    elseif (size(iyl)==0)
        iyl=1;
        a(2)=0;
    end
    if (size(iyg)==[0,1])
        iyg=20;
        a(4)=0;
    elseif (size(iyg)==0)
        iyg=20;
        a(4)=0;
    end
    dyp=(y-iyl)*5;
    dyn=(iyg-y)*5;
    ix=find(mem(y,:)==1);
    ixl=ix(find(ix<x, 1, 'last'));
    ixg=ix(find(ix>x, 1, 'first'));
    if (size(ixl)==[1,0])
        ixl=1;
        a(3)=0;
    elseif (size(ixl)==0)
        ixl=1;
        a(3)=0;
    end
    if (size(ixg)==[1,0])
        ixg=20;
        a(1)=0;
    elseif (size(ixg)==0)
        ixg=20;
        a(1)=0;
    end
    dxn=(x-ixl)*5;
    dxp=(ixg-x)*5;
    dist=[dxp;dyp;dxn;dyn];
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
%% Non-Linear Constraint Function
function [c,ceq] = mag(u)
    c = u(1)^2 + u(2)^2 - (2.5^2);
    ceq = [];
end