%% Generating Obstacles
l=100;
st=0;
gl=20;
gs=5;
os=25;
MarkerSize=5;
mps=(400*MarkerSize)/(2*l);
obs=[st,st;st+os,st;st+2*os,st;st+3*os,st;st,st+os;st+os,st+os;st+2*os,st+os;st+3*os,st+os;st,st+2*os;st+os,st+2*os;st+2*os,st+2*os;st+3*os,st+2*os;st,st+3*os;st+os,st+3*os;st+2*os,st+3*os;st+3*os,st+3*os]+5*randi([1,5],[16,2]);
gen=obs/gs;
obs=-(gs/2)*ones(size(obs))+obs;
plot(obs(:,1),obs(:,2), 's', 'MarkerSize',mps);
hold on;
axis([-(l/2) (3*l/2) -(l/2) (3*l/2)]);
%% State Initialisation and Obstacle Memory
x=25;
y=-25;
ySP=[120;120];
next=((gen(:,1)-1)*gl)+(gl-(gen(:,2)-1));
od=zeros(gl);
mem=od;
od(next)=1;
%[dist,point]=ob(0,0,pi/6,obs)
% mem=upd(mem,od,35,25);
%% Defining Parameters and Weights
Wo=[1,1,.5];
% Wo5=0;
% Wo6=0;
Wi=[.05,.05]*0;
% duMax=.25;
% duMin=-.25;
global Umagmax;
Umagmax=2.5;
sr=3;
%% Initialisation
maxTime=120;
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
    mem=upd(mem,od,x,y,st,l);
    [dist,a]=compOB(x,y,mem,sr,gl,gs);
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
function memnew=upd(mem,od,x,y,st,l)
    [x,y]=trans(x,y);
    %[b,e]=trans(st,st+l);
     if y==0
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
function [dist,a]=compOB(x,y,mem,sr,gl,gs)
    a=ones(4,1);
    [x,y]=trans(x,y);
    iy=find(mem(max(y-sr,1):min(y+sr,20),x)==1);
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
    dyp=(y-iyl)*gs;
    dyn=(iyg-y)*gs;
    ix=find(mem(y,max(x-sr,1):min(x+sr,20))==1);
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
    dxn=(x-ixl)*gs;
    dxp=(ixg-x)*gs;
    dist=[dxp;dyp;dxn;dyn];
end
%% Transform Fn.
function [xn,yn]=trans(x,y)
     if (rem((x/5),1)==0)
        xn=x/5+1;
    else
        px=ceil(x/5);
        xn=px;
    end
    if (rem((y/5),1)==0)
        yn=20-(y/5);
    else
        py=ceil(y/5);
        yn=20-py;
    end
    xn=min(xn,20);
    yn=min(yn,20);
    xn=max(xn,1);
    yn=max(yn,1);
end
%% Non-Linear Constraint Function
function [c,ceq] = mag(u)
    global Umagmax;
    c = u(1)^2 + u(2)^2 - (Umagmax^2);
    ceq = [];
end