% Load system matrices (A, B, Bw, C, H, R1, R2)  
load sysMat
Qbar=diag([10, 1]);
R=diag([0.25 0.25]);

%% ===== END OF FIXED SECTION OF THE CODE =====
% Obtain phi, gamma, psi, xi and Q matrices
phi=[A,Bw;zeros(2,20),eye(2)];
gamma=[B;zeros(2,2)];
psi=[zeros(20,2);eye(2)];
xi=[C,zeros(2,2)];
Hbar=[H,zeros(2,2)];
Q=Hbar'*Qbar*Hbar;
S=eye(22);
for i=1:50
    L=inv(gamma'*S*gamma+R)*gamma'*S*phi;
    S=phi'*S*phi+Q-phi'*S*gamma*L;
end

    