%Setting up the model
G11=tf(1.7,[60.48 15.6 1]); 
G12=tf(.5,[19.36 7.04 1]); 
G21=tf(.3,[10.89 4.62 1]); 
G22=tf(1,[36 12 1]); 
G=[G11 G12;G21 G22];
model=ss(G);
%Naming and defining the different inputs, outputs and states
model.InputName = {'u1','u2'};
model.OutputName = {'y1','y2'};
model.StateName = {'x1','x2','x3','x4','x5','x6','x7','x8'};
model.InputGroup.MV = 2;
model.OutputGroup.MO = 2;
old_status = mpcverbosity('off');
%Setting up parameters of mpc
Ts=2;
p=12;
m=4;
%Setting up Constraints
MV=struct('Min',-.5,'Max',.5,'RateMin',-.05,'RateMax',.05);
%Setting up Weights
W=struct('MVRate',[1 1],'OV',[.01 .01]);
%Setting up the MPC
MPCobj=mpc(model,Ts,p,m,W,MV);
T = 25;
sp = [.4,-.4];
%Running the MPC
sim(MPCobj,T,sp)
