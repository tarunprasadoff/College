a = 01;%Radius of smaller circle
P1=[0,0];% Center of smaller circle
P2=[-5,1];%Fixed point of pantograph

%Parameters of plot
axis(gca,'equal');%Aspect ratio of the plot
axis([-7 10 -7 20]);%Limits of axes

k=10;%Angular velocity parameter of point on smaller circle (P3)

%Loop to Carry out all animation with pause provided
for t=1:500
    theta=k*(t/100);
    P3=a*[cos(theta), sin(theta)];%points on the smaller circle
    
    %calculations for P4
    l= 12.5/(1-P3(2));
    m = (5+P3(1))/(1-P3(2));
    L=1+m*m;
    M=10+2*l*m-2*m;
    N=l*l-2*l+1;
    A=sqrt(M*M-4*L*N);
    x=(-M+A)/(2*L);
    y=(25+x*(10+2*P3(1)))/(2*(1-P3(2)));
    P4=[x,y];
    
    %Calculations for P5
    C=(y-1)/(x+5);
    D=sqrt(1+C*C);
    x1=(10/D)-5;
    y1=C*(x1+5)+1;
    P5=[x1,y1];
    
    %Calculations for P6
    f=-(y1-P3(2))/(x1-P3(1));
    e=(x1*x1+y1*y1-1)/(2*x1-2*P3(1));
    E=1+f*f;
    F=2*e*f-2*P3(1)*f-2*P3(2);
    G=e*e+P3(1)*P3(1)-2*P3(1)*e+P3(2)*P3(2)-25;
    z=sqrt(F*F-4*E*G);
    y2=(-F-z)/(2*E);
    x2=e+f*y2;
    P6=[x2,y2];
    
    %Calculations for P7
    Z=(x2-x1)/(y2-y1);
    y3=y1-10/(sqrt(1+Z*Z));
    x3=x1+Z*(y3-y1);
    P7=[x3,y3];
    
    %Animations for segments
    segment1 = line([P4(1) P3(1)], [P4(2) P3(2)]);
    segment2 = line([P2(1) P4(1)], [P2(2) P4(2)]);
    segment3 = line([P4(1) P5(1)], [P4(2) P5(2)]);
    segment4 = line([P6(1) P5(1)], [P6(2) P5(2)]);
    segment5 = line([P6(1) P3(1)], [P6(2) P3(2)]);
    segment6 = line([P6(1) P7(1)], [P6(2) P7(2)]);
   
    %Equations of points P4,P5,P6...
    P3_circ = viscircles(P3,0.1);
    P4_circ = viscircles(P4,0.1);
    P5_circ = viscircles(P5,0.1);
    P6_circ = viscircles(P6,0.1);
    P7_circ = viscircles(P7,0.1);
    
    %Pause for vision of persistence 
    pause(0.001);
    
    %Erase screen after pause to start new 
    delete(segment1);
    delete(segment2);
    delete(segment3);
    delete(segment4);
    delete(segment5);
    delete(segment6);
    delete(P4_circ);
    %delete(P3_circ);
     delete(P5_circ);
     delete(P6_circ);
     %delete(P7_circ);
end
