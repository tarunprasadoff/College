%there are two circles within which you can draw (range of pantograph
%you can change link lengths and magnification of pantograph by changing
%the value of variable l1,l2 and m.
%wait an instant after running for screen to initialise
%bring mouse cursor within the circle within 4 seconds after running
%Parameters of plot
axis(gca,'equal');%Aspect ratio of the plot
axis(3*[-80 80 -80 80]);%Limits of axes

set (gcf, 'WindowButtonMotionFcn', @mouse);%sets up the mouse tracking function
%setting the 3 basic parameters of link lengths and magnification parameter
l1=40;
l2=60;
m=3;
%setting the boundary in which we can draw
boundary1 = viscircles([0,0],(l1+l2),'LineStyle','--');
boundary2 = viscircles([0,0],abs(l1-l2),'LineStyle','--');
                for i=1:1000
                    P3 = mouse();%getting mouse coordinate
                    x=P3(1);
                    y=P3(2);
                    if i>40 % delay to bring cursor into drawing region and to avoid random lines
                    [P1,P2,p3,P4,P5,P6]=find(l1,l2,m,x,y);%getting the pantograph points for current instant
                        
                        
                    end
                        if i>50 %initial delay to maximize screen
                             %plotting the input and output lines
                             line([P3i(1) P3(1)], [P3i(2) P3(2)]);
                             line([P6i(1) P6(1)], [P6i(2) P6(2)]);
                            
                             %plotting the pantograph for current instant
                             P1_circ = viscircles(P1,0.1);
                             P2_circ = viscircles(P2,0.1);
                             P3_circ = viscircles(p3,0.1);
                             P4_circ = viscircles(P4,0.1);
                             P5_circ = viscircles(P5,0.1);
                             P6_circ = viscircles(P6,0.1);
                             segment1=line([P1(1) P4(1)], [P1(2) P4(2)]);
                             segment2=line([P2(1) p3(1)], [P2(2) p3(2)]);
                             segment3=line([p3(1) P5(1)], [p3(2) P5(2)]);
                             segment4=line([P4(1) P6(1)], [P4(2) P6(2)]);
                             pause(0.1);%pause to generate a frame for a 0.1 second
                             %deleting pantograph for next instant
                             delete(segment1);
                             delete(segment2);
                             delete(segment3);
                             delete(segment4);

    
                             delete(P3_circ);
                             delete(P1_circ);
                             delete(P4_circ);
                             delete(P5_circ);
                             delete(P6_circ);
                             delete(P2_circ);
                             
                             
                             
                             
                                
                             

                             
    


                        end
                    %storing previous input to generate input line and output line    
                    P3i=P3;
                    P6i=P6;
                    %giving pause in for loop when above if is not running
                    if i<51
                        pause(0.1);
                    end
                end
                
                
   
    %function to get mouse data
    function P3 = mouse(object,eventdata)
        C = get (gca, 'CurrentPoint');
        x=C(1,1);
        y=C(1,2);
        P3=[x,y];
    end
    
function [P1,P2,p3,P4,P5,P6]=find(l1,l2,m,x,y)
%solving the quadratic to get theta and alpha
k=(x^2+y^2+l2^2-l1^2)/(2*l2);
a=x^2+y^2;
b=-(2*x*k);
c=k^2-y^2;
if y >0
    alpha=(-b+((b^2-4*a*c)^0.5))/(2*a);
else
    alpha=(-b-((b^2-4*a*c)^0.5))/(2*a);
end
phi1=acos(alpha);   
theta1=acos((x-alpha*l2)/l1);
r=[[theta1,phi1],[-theta1,phi1],[theta1,-phi1],[-theta1,-phi1]];
%loop to find correct theta's and alpha's as inverse function has limited
%range
for it=1:4
   t2=l1*[cos(r(2*it-1)),sin(r(2*it-1))];
   t3=t2+l2*[cos(r(2*it)),-sin(r(2*it))];
   se=t3(1)-x;
   sf=t3(2)-y;
   if abs(se)<10e-1 && abs(sf)<10e-1
            theta=r(2*it-1);
            phi=r(2*it);
            break;
   end
end
%using the obtained angles to get coordinates of the different joints
P1=[0,0];
P2=l1*[cos(theta),sin(theta)];
p3=P2+l2*[cos(phi),-sin(phi)];
P4=m*P2;
P5=P4+l2*[cos(phi),-sin(phi)];
P6=P4+m*l2*[cos(phi),-sin(phi)];
end