%Setting up the model
beta=0;
G11=tf(4.05+2.11*beta,[50 1],'InputDelay',27); 
G12=tf(1.77-0.39*beta,[60 1],'InputDelay',28); 
G13=tf(5.88+0.59*beta,[50 1],'InputDelay',27); 
G21=tf(5.39+3.29*beta,[50 1],'InputDelay',18); 
G22=tf(5.72-0.57*beta,[60 1],'InputDelay',14); 
G23=tf(6.9+0.89*beta,[40 1],'InputDelay',15); 
G31=tf(4.38+3.11*beta,[33 1],'InputDelay',20); 
G32=tf(4.42-0.73*beta,[44 1],'InputDelay',22); 
G33=tf(7.2+1.33*beta,[19 1]); 
G14=tf(1.2+0.12*beta,[45 1],'InputDelay',27); 
G15=tf(1.44+0.16*beta,[40 1],'InputDelay',27); 
G24=tf(1.52+.13*beta,[25 1],'InputDelay',15); 
G25=tf(1.83+.13*beta,[20 1],'InputDelay',15); 
G34=tf(1.14+.18*beta,[27 1]); 
G35=tf(1.26+0.18*beta,[32 1]); 
G=[G11 G12 G13 G14 G15; G21 G22 G23 G24 G25; G31 G32 G33 G34 G35];
model=ss(G);
model.TimeUnit = 'minutes';

%Naming and defining the different inputs, outputs and states
model.InputName = {'u1','u2','u3','d1','d2'};
model.OutputName = {'y1','y2','y7'};
model.StateName = {'x1','x2','x3','x4','x5','x6','x7','x8','x9','x10','x11','x12','x13','x14','x15'};
model. InputGroup.MV = [1,2,3];
model.InputGroup.UD = [4,5];
model.OutputGroup.MO = [1,2,3];
old_status = mpcverbosity('off');

%Setting up parameters of mpc
Ts=2;
p=40;
m=10;

%Setting up Constraints
MV(1:3)=struct('Min',-.5,'Max',.5,'RateMin',-.05,'RateMax',.05);

%Setting up Weights
W=struct('MVRate',[1.5 .15 1.5],'OV',[1 1 0]);

%Setting up the MPC
MPCobj=mpc(model,Ts,p,m,W,MV);
MPCobj.OV(3).Min=-.5;
MPCobj.OV(1).Min=-.5;
MPCobj.OV(1).Max=.5;
T = 50;
sp = [0,0,0];
d=[-.5,-.5];
op=mpcsimopt(MPCobj);
op.unmeas=d;
 
%Running the MPC
sim(MPCobj,T,sp,op)