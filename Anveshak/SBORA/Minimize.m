limits=[0,0.1;0,0.6;0,10;0,0.05];
PL=genP(limits);
l=zeros(size(PL,1),1);
for i=1:size(PL,1)
    [M,SD]=computeMV(PL(i,:));
    l(i)=(SD/M)*100;
end
m=min(l);
for i=1:size(PL,1)
    if l(i)==m
        in=i;
    end
end
fprintf("Least Variation per= %f, y=%f, h=%f, k=%f, d=%f \n", m,PL(in,1), PL(in,2), PL(in,3), PL(in,4));
