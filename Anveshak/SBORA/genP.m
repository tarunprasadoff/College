function PL=genP(limits)
n=1000;
step=(limits(:,2)-limits(:,1))/n;
PL=zeros(n+1,size(limits,1));
A=limits(:,1);
counter=1;
for i=0:n
   b1=A(2);
   for j=0:n
       c1=A(3);
       for k=0:n
           d1=A(4);
           for l=0:n
               PL(counter,:)=[A(1),b1,c1,d1];
               d1=d1+step(size(limits,1));
               counter=counter+1;
           end
           c1=c1+step(size(limits,1)-1);
       end
       b1=b1+step(size(limits,1)-2);
   end
   A(1)=A(1)+step(size(limits,1)-3);
end
end