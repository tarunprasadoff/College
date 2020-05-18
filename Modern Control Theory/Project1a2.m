%% Generating the step response matrix for MIMO System
G11=tf(1.7,[60.48 15.6 1]); 
G12=tf(.5,[19.36 7.04 1]); 
G21=tf(.3,[10.89 4.62 1]); 
G22=tf(1,[36 12 1]); 
h=2;
n=25;
y11=step(G11,0:h:n*h);
y12=step(G12,0:h:n*h);
y21=step(G21,0:h:n*h);
y22=step(G22,0:h:n*h);
S11=y11(2:end);
S12=y12(2:end);
S21=y21(2:end);
S22=y22(2:end);
S=zeros(2*n,2);
for i=1:n
    S(2*i-1,1)=S11(i);
    S(2*i-1,2)=S12(i);
    S(2*i,1)=S21(i);
    S(2*i,2)=S22(i);
end
%% Defining Parameters and Weights
ySP=[.4;-.4];          % Setpoint
m=4;            % Control horizon
p=12;           % Prediction horizon
Wo1=100;            % Output weight1
Wo2=100;            % Output weight2
Wi1=.05;         % Input weight1
Wi2=.05;         % Input weight2
duMax=.05;       % Input Change Constraints
duMin=-.05;
uMax=.5;         % Input Magnitude Constraints
uMin=-.5;
%% Initialisation
uPrev=[0;0];
Yk0=zeros(2*n,1);
maxTime=50;

Y_SAVE1=zeros(maxTime+1,1);
Y_SAVE2=zeros(maxTime+1,1);
T_SAVE=zeros(maxTime+1,1);
U_SAVE1=zeros(maxTime,1);
U_SAVE2=zeros(maxTime,1);

%% Pre-Computing Matrices
bigR=repmat(ySP,[p 1]); %Bigger Setpoint Matrix
bigSu=zeros(2*p,2*m);
gammaY=zeros(2*p);
Im=eye(2*m);
Jm=zeros(2*m);
for i=1:p
    if i~=1
        bigSu(2*i-1:2*i,3:2*m)=bigSu(2*i-3:2*i-2,1:2*m-2);
    end
    bigSu(2*i-1:2*i,1:2)=S(2*i-1:2*i,:);
    gammaY(2*i-1,2*i-1)=Wo1;
    gammaY(2*i,2*i)=Wo2;
end
gammaU=zeros(2*m);
for i=1:m
    if i~=1
        Jm(2*i-1:2*i,3:2*m)=Jm(2*i-3:2*i-2,1:2*m-2);
    end
    gammaU(2*i-1,2*i-1)=Wi1;
    gammaU(2*i,2*i)=Wi2;
    Jm(2*i-1:2*i,1:2)=eye(2);
end
Hess=bigSu'*gammaY*bigSu + gammaU; %Computing Hess
LHS=[Im;-Im;Jm;-Jm]; %Left Hand Constraint
%% Implementation
for k=1:maxTime+1
    time=(k-1)*h;
    if (k==1)
        YHAT=Yk0;
    end
    predErr=YHAT(3:2*p+2)-bigR; %Error for MIMO
    grad=bigSu'*gammaY*predErr;
    %du=-inv(Hess)*grad;
    RHS=[repmat(duMax,2*m,1); repmat(-duMin,2*m,1);repmat( (uMax-uPrev),m,1);repmat(-(uMin-uPrev),m,1)];
    big_dU=quadprog(Hess,grad,LHS,RHS);
    uk=uPrev+big_dU(1:2);
    uPrev=uk;
    U_SAVE1(k)=uk(1);
    U_SAVE2(k)=uk(2);
    YHAT=[YHAT(3:end);YHAT(end-1:end)] + S*big_dU(1:2); %2Step Forward Shift
    yk=YHAT(1:2);
    Y_SAVE1(k+1)=yk(1);
    Y_SAVE2(k+1)=yk(2);
    T_SAVE(k+1)=k*h;
end
%% Plotting
subplot(2,2,1);
plot(T_SAVE(1:maxTime),Y_SAVE1(1:maxTime),'-b','linewidth',1); 
ylabel('Output, y1_k');
subplot(2,2,2);
plot(T_SAVE(1:maxTime),Y_SAVE2(1:maxTime),'-b','linewidth',1); 
ylabel('Output, y2_k');
subplot(2,2,3);
stairs(T_SAVE(1:maxTime),U_SAVE1(1:maxTime),'-b','linewidth',1); 
ylabel('Input, u1_k'); xlabel('time, t')
subplot(2,2,4);
stairs(T_SAVE(1:maxTime),U_SAVE2(1:maxTime),'-b','linewidth',1); 
ylabel('Input, u2_k'); xlabel('time, t')

