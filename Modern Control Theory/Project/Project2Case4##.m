%Setting up the model
beta=1;
G11=tf(4.05+2.11*beta,[50 1],'InputDelay',27); 
G12=tf(1.77-0.39*beta,[60 1],'InputDelay',28); 
G13=tf(5.88+0.59*beta,[50 1],'InputDelay',27); 
G21=tf(5.39+3.29*beta,[50 1],'InputDelay',18); 
G22=tf(5.72-0.57*beta,[60 1],'InputDelay',14); 
G23=tf(6.9+0.89*beta,[40 1],'InputDelay',15); 
G31=tf(4.38+3.11*beta,[33 1],'InputDelay',20); 
G32=tf(4.42-0.73*beta,[44 1],'InputDelay',22); 
G33=tf(7.2+1.33*beta,[19 1]); 
G=[G11 G12 G13; G21 G22 G23; G31 G32 G33];
model=ss(G);
model.TimeUnit = 'minutes';

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
G=[G11 G12 G13; G21 G22 G23; G31 G32 G33];
modelp=ss(G);
modelp.TimeUnit = 'minutes';

%Naming and defining the different inputs, outputs and states
model.InputName = {'u1','u2','u3'};
model.OutputName = {'y1','y2','y7'};
model.StateName = {'x1','x2','x3','x4','x5','x6','x7','x8','x9'};
model. InputGroup.MV = [1,2,3];
model.OutputGroup.MO = [1,2,3];
modelp. InputGroup.MV = [1,2,3];
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
T = 1000;
sp = [0,0,0];
d=[-.5,-.5];
op=mpcsimopt(MPCobj);
op.OutputNoise=[-.5,-.5,-.5];
op.Model = modelp;
 
%Running the MPC
sim(MPCobj,T,sp,op)