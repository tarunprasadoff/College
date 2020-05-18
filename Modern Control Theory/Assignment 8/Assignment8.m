clf
clear all
% System Matrices
A=[1, 0.1; -0.3, 0.65];
B=[1; 0.2];
C=[1, 0];

% Tuning Matrices
Q=eye(2);
R=1;

% Solve using MATLAB toolbox to directly give LQR solution
% Report the values in the variables Linf and Sinf
x=ones(2,201);%Actual x
xh=zeros(2,201);%Estimator X using deadbeat case
xkf=zeros(2,201);%Estimator X using KF
L=[1.65;3.925];%Dead Beat Observor Case obtained from Assignment 6
k=linspace(1,201,201);
KF=[0.3365;0.4610];
e=zeros(2,200);
ekf=zeros(2,200);
s=0;
skf=0;
y=C*x(:,1)+normrnd(0,0.2^.5);
for i=1:200
    x(:,i+1)=A*x(:,i)+[0;1]*normrnd(0,1);
    xh(:,i+1)=A*xh(:,i)+L*(y-C*xh(:,i));
    xkfp=A*xkf(:,i);
    y=C*x(:,i+1)+normrnd(0,0.2^.5);
    xkf(:,i+1)=xkfp+KF*(y-C*xkfp);
    e(:,i)=x(:,i+1)-xh(:,i+1);
    ekf(:,i)=x(:,i+1)-xkf(:,i+1);
    s=s+e(:,i)'*e(:,i);
    skf=skf+ekf(:,i)'*ekf(:,i);
end
s
skf
subplot(2,1,1);
plot(k,xh(1,:),'-b','linewidth',1);
hold on
plot(k,x(1,:),'-r','linewidth',1); 
hold on
plot(k,xkf(1,:),'-g','linewidth',1); 
ylabel('First State');
xlabel('Time');
legend('Deadbeat Estimator','Actual State','Kalman Estimator')
subplot(2,1,2);
plot(k,xh(2,:),'-b','linewidth',1);
hold on
plot(k,x(2,:),'-r','linewidth',1); 
hold on
plot(k,xkf(2,:),'-g','linewidth',1);
ylabel('time, Second State');
xlabel('Time');
legend('Deadbeat Estimator','Actual State','Kalman Estimator')