%% Generating Obstacles
obs=[0,0;25,0;50,0;75,0;0,25;25,25;50,25;75,25;0,50;25,50;50,50;75,50;0,75;25,75;50,75;75,75]+5*randi([1,5],[16,2]);
gen=obs/5;
obs=-2.5*ones(16,2)+obs;
plot(obs(:,1),obs(:,2), 's', 'MarkerSize',20);
hold on;
axis([0 100 0 100]);
%% State Initialisation and Obstacle Memory
x=0;
y=0;
%theta=0;
next=((gen(:,1)-1)*20)+(20-(gen(:,2)-1));
od=zeros(20);
mem=od;
od(next)=1;
%[dist,point]=ob(0,0,pi/6,obs)
% mem=upd(mem,od,35,25);
%% Input-Output Relation
h=1;
n=100;
y11=ones(n,1);
y12=zeros(n,1);
y21=y12;
y22=y11;
y31=y11;
y32=y12;
y41=y21;
y42=y22;
% y51=y11/(2^.5);
% y52=y11/(2^.5);
% y61=y11/(2^.5);
% y62=-y11/(2^.5);
% S=zeros(6*n,2);
S=zeros(4*n,2);
for i=1:n
%     S(6*i-5,1)=y11(i);
%     S(6*i-5,2)=y12(i);
%     S(6*i-4,1)=y21(i);
%     S(6*i-4,2)=y22(i);
%     S(6*i-3,1)=y31(i);
%     S(6*i-3,2)=y32(i);
%     S(6*i-2,1)=y41(i);
%     S(6*i-2,2)=y42(i);
%     S(6*i-1,1)=y51(i);
%     S(6*i-1,2)=y52(i);
%     S(6*i-0,1)=y61(i);
%     S(6*i-0,2)=y62(i);
      S(4*i-3,1)=y11(i);
      S(4*i-3,2)=y12(i);
      S(4*i-2,1)=y21(i);
      S(4*i-2,2)=y22(i);
      S(4*i-1,1)=y31(i);
      S(4*i-1,2)=y32(i);
      S(4*i,1)=y41(i);
      S(4*i,2)=y42(i);
end
%% Defining Parameters and Weights
%ySP=[.4;-.4];          % Setpoint-Set it in the loop
m=5;            % Control horizon
p=15;           % Prediction horizon
Wo1=20;            % Output weight1
Wo2=20;            % Output weight2
Wo3=1;
Wo4=1;
% Wo5=0;
% Wo6=0;
Wi1=.005;         % Input weight1
Wi2=.005;         % Input weight2
duMax=.25;
duMin=-.25;
uMax=100;
uMin=-100;
%% Initialisation
uPrev=[0;0];
% Yk0=zeros(6*n,1);
% Yk0=zeros(4*n,1);
Yk0=repmat([x;y;x;y],[n 1]);
maxTime=300;

Y_SAVE1=zeros(maxTime+1,1);
Y_SAVE2=zeros(maxTime+1,1);
Y_SAVE3=zeros(maxTime+1,1);
Y_SAVE4=zeros(maxTime+1,1);
% Y_SAVE5=zeros(maxTime+1,1);
% Y_SAVE6=zeros(maxTime+1,1);
T_SAVE=zeros(maxTime+1,1);
U_SAVE1=zeros(maxTime,1);
U_SAVE2=zeros(maxTime,1);
%% Pre-Computing Matrices
%bigR=repmat(ySP,[p 1]); Set it in Loop
% bigSu=zeros(6*p,2*m);
bigSu=zeros(4*p,2*m);
% gammaY=zeros(6*p);
gammaY=zeros(4*p);
Im=eye(2*m);
Jm=zeros(2*m);
for i=1:p
    if i~=1
%         bigSu(6*i-5:6*i,3:2*m)=bigSu(6*i-11:6*i-6,1:2*m-2);
        bigSu(4*i-3:4*i,3:2*m)=bigSu(4*i-7:4*i-4,1:2*m-2);
    end
%     bigSu(6*i-5:6*i,1:2)=S(6*i-5:6*i,:);
    bigSu(4*i-3:4*i,1:2)=S(4*i-3:4*i,:);
%     gammaY(6*i-5,6*i-5)=Wo1;
%     gammaY(6*i-4,6*i-4)=Wo2;
%     gammaY(6*i-3,6*i-3)=Wo3;
%     gammaY(6*i-2,6*i-2)=Wo4;
%     gammaY(6*i-1,6*i-1)=Wo5;
%     gammaY(6*i,6*i)=Wo6;
    gammaY(4*i-3,4*i-3)=Wo1;
    gammaY(4*i-2,4*i-2)=Wo2;
    gammaY(4*i-1,4*i-1)=Wo3;
    gammaY(4*i,4*i)=Wo4;
end
gammaU=zeros(2*m);
for i=1:m
    if i~=1
%         bigSu(6*i-5:6*i,3:2*m)=bigSu(6*i-11:6*i-6,1:2*m-2);
        Jm(2*i-1:2*i,3:2*m)=Jm(2*i-3:2*i-2,1:2*m-2);
    end
%     bigSu(6*i-5:6*i,1:2)=S(6*i-5:6*i,:);
    gammaU(2*i-1,2*i-1)=Wi1;
    gammaU(2*i,2*i)=Wi2;
    Jm(2*i-1:2*i,1:2)=eye(2);
end
Hess=bigSu'*gammaY*bigSu + gammaU;
LHS=[Im;-Im;Jm;-Jm];
%% Implementation
for k=1:maxTime+1
    mem=upd(mem,od,x,y);
    ySP=compSP(x,y,mem);
    bigR=repmat(ySP,[p 1]);
    time=(k-1)*h;
    if (k==1)
        YHAT=Yk0;
    end
    predErr=YHAT(5:4*p+4)-bigR;
    grad=bigSu'*gammaY*predErr;
    %big_dU=-inv(Hess)*grad;
    RHS=[repmat(duMax,2*m,1); repmat(-duMin,2*m,1);repmat( (uMax-uPrev),m,1);repmat(-(uMin-uPrev),m,1)];
    big_dU=quadprog(Hess,grad,LHS,RHS);
    uk=uPrev+big_dU(1:2);
    uPrev=uk;
    Y_SAVE1(k)=x;
    Y_SAVE2(k)=y;
    x=x+big_dU(1);
    y=y+big_dU(2);
    U_SAVE1(k)=uk(1);
    U_SAVE2(k)=uk(2);
    YHAT=[YHAT(5:end);YHAT(end-3:end)] + S*big_dU(1:2);
    yk=YHAT(1:4);
%     Y_SAVE1(k+1)=yk(1);
%     Y_SAVE2(k+1)=yk(2);
%     Y_SAVE3(k+1)=yk(3);
%     Y_SAVE4(k+1)=yk(4);
    T_SAVE(k+1)=k*h;
end
%% Plotting
plot(Y_SAVE1,Y_SAVE2);
% subplot(1,2,1);
% plot(T_SAVE(1:maxTime),Y_SAVE1(1:maxTime),'-b','linewidth',1); 
% ylabel('Output, x_k');
% subplot(1,2,2);
% plot(T_SAVE(1:maxTime),Y_SAVE2(1:maxTime),'-b','linewidth',1); 
% ylabel('Output, y_k');
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
%     y
%     x
%     ily
%     igy
%     ilx
%     igx
    mem(y-ily:y+igy,x-ilx:x+igx)=od(y-ily:y+igy,x-ilx:x+igx);
    memnew=mem;
end
%% Setpoint Function
function ySP=compSP(x,y,mem)
    [x,y]=trans(x,y);
    ySP=[100;100;0;0];
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