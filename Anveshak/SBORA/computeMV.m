function [vp] = computeMV(P)
S=0;
V=0;
for i=0:90
    for j=0:180
        S=S+PE(P,i,j);
    end
end
M=S/((i+1)*(j+1));
for i=0:90
    for j=0:180
        V=V+(PE(P,i,j)-M)^2;
    end
end
V=V/((i+1)*(j+1));
SD=V^0.5;
vp=(SD/M)*100;
end
