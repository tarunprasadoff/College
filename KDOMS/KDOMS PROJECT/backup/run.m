%Parameters of plot
axis(gca,'equal');%Aspect ratio of the plot
axis([-40 40 -40 40]);%Limits of axes
[a,b] = ginput(30);
%plot(a,b);
p=a;
d=b;
L=20;
P1=[-L,0];
for j=1:5
  for i=1:30
    x=p(i);
    y=d(i);
    P2=[x,y];
    
    %Calculations of P3
    e=(x*x+y*y-L*L)/(2*y);
    f=(-L-x)/y;
    A=1+f*f;
    B=2*(e*f+L);
    C=e*e;
    x1=(-B-(sqrt(B*B-4*A*C)))/(2*A);
    y1=e+f*x1;
    P3=[x1,y1];
    
    m=(x1+L)/y1;
    g=sqrt(1+m*m);
     if y>0
       y2=2*L/g;
     else
        y2=-2*L/g; 
     end
    x2=(y2*m-L);
    P4=[x2,y2];
    
    k=(x2*x2+y2*y2-x*x-y*y)/(2*x2-2*x);
    n=(y-y2)/(x2-x);
    U=1+n*n;
    V=2*(k*n-x*n-y);
    W=k*k-2*k*x+x*x+y*y-L*L;
    if y>0
       y3=(-V+(sqrt(V*V-4*U*W)))/(2*U);
    else
       y3=(-V-(sqrt(V*V-4*U*W)))/(2*U);
    end
    x3=k+n*y3;
    P5=[x3,y3];
    
    r=(x3-x2)/(y3-y2);
    s=sqrt(1+r*r);
    if y<0
       y4=y2+2*L/s;
    else
        y4=y2-2*L/s; 
    end
       x4=x2+r*(y4-y2);
    P6=[x4,y4]
    
    P3_circ = viscircles(P3,0.1);
    P1_circ = viscircles(P1,0.1);
    P2_circ = viscircles([p(i),d(i)],0.1);
    P4_circ = viscircles(P4,0.1);
    P5_circ = viscircles(P5,0.1);
    P6_circ = viscircles(P6,0.1);
    
    segment1=line([P1(1) P3(1)], [P1(2) P3(2)]);
    segment2=line([P2(1) P3(1)], [P2(2) P3(2)]);
    segment3=line([P4(1) P3(1)], [P4(2) P3(2)]);
    segment4=line([P4(1) P5(1)], [P4(2) P5(2)]);
    segment5=line([P6(1) P5(1)], [P6(2) P5(2)]);
    segment6=line([P2(1) P5(1)], [P2(2) P5(2)]);
    
    pause(0.1);
    
    delete(segment1);
    delete(segment2);
      delete(segment3);
      delete(segment4);
      delete(segment5);
      delete(segment6);
    
    delete(P3_circ);
    %delete(P2_circ);
    delete(P1_circ);
    delete(P4_circ);
     delete(P5_circ);
  end
  %pause(0.1);
end

