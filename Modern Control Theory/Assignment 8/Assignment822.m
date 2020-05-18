clf
clear all
%% Simulations of dead-beat estimator
% Load system matrices (A, B, Bw, C, H, R1, R2)  
load sysMat

% Load inputs (L, V), measurements (ym) and expected controlled outputs (yc)
load kfExample

% Initialization of x(0) and xHat(0)
x0=zeros(20,1);
xhat0=zeros(20,1);
P=0.01*eye(20);
%% ===== END OF FIXED SECTION OF THE CODE =====

%% ===== KALMAN FILTER SIMULATIONS (with w(k) as integrated white noise) =====
% First, augment the model to handle IWN
YHAT=ones(2,200);%Estimated Y
AA=[A,Bw;zeros(2,20),eye(2)];
BB=[B;zeros(2,2)];
N=[zeros(20,2);eye(2)];
CC=[C,zeros(2,2)];
z0=zeros(22,1);
zhat0=zeros(22,1);
Pz=0.01*eye(22);
SSE1=0;
SSE2=0;
for i=1:200
    zhatp=AA*zhat0+BB*[L(i);V(i)];
    Pzhatp=AA*Pz*AA'+N*R1*N';
    K=Pzhatp*CC'*inv(CC*Pzhatp*CC'+R2);
    Pz=(eye(22)-K*CC)*Pzhatp;
    zhat0=zhatp+K*(ym(i,:)'-CC*zhatp);
    YHAT(:,i)=H*zhat0(1:20,1);
    e1=yc(i,1)-YHAT(1,i);
    SSE1=SSE1+e1'*e1;
    e2=yc(i,2)-YHAT(2,i);
    SSE2=SSE2+e2'*e2;
end



%% ===== CALCULATIONS AND PLOTTING =====
% Please report the sum of square error in yc_1 as SSE1
% SSE1 = sum((yc(:,1)-YHAT(:,1)).^2);  % Sums over all time-points for the first output
disp(SSE1);      % Displays SSE

% SSE2 = sum((yc(:,2)-YHAT(:,2)).^2);  % Sums over all time-points for the second output
disp(SSE2);      % Displays SSE

subplot(2,1,1)  % Plot of x1
plot(1:200,yc(:,1),'--b',1:200,YHAT(1,:),'-r'); xlabel('Distillate, x_D(k)');
subplot(2,1,2)  % Plot of x2
plot(1:200,yc(:,2),'--b',1:200,YHAT(2,:),'-r'); xlabel('Bottoms, x_B(k)');