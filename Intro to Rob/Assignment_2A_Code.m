% Hardcode the values of theta1, theta2 and d3.

t1 = pi/4;
t2 = pi/2;
d3 = 100;

disp(Jac(t1,t2,d3));

% The function Jac computes the Jacobian matrix with the DH parameters for
% the DH parameters and relations for d1,a1,a2 and a3 hardcoded in the
% function with t1, t2 and d3 as arguments.

function val = Jac(q1, q2, q3)
    
    syms t1 t2 d3 l1 l2 l3 a
    
    d1 = l1 - l2*cos(a);
    a1 = l2*sin(a);
    a2 = l3;
    a3 = d3;

    DHpar = [t1 d1 a1   pi/2;
          t2  0 a2  -pi/2;
           0  0 a3      0;];

       
    % Calculating the Linear Jacobian 'A'
    L = For(DHpar);
    
    m = L(1,4);
    n = L(2,4);
    o = L(3,4);

    A = jacobian([m; n; o] , [t1; t2; d3]);
    
    % Calculating the Angular Jacobian 'B'
    
    R00 = eye(3);
    
    T01 = Trans(DHpar(1,1), DHpar(1,2), DHpar(1,3), DHpar(1,4));
    R01 = T01(1:3,1:3);
    
    e_3 = [0; 0; 1]; 

    B = [ R00*e_3 R01*e_3 [0; 0; 0]];
    
    val = [A; B];
    val = subs(val,t1,q1);
    val = subs(val,t2,q2);
    val = subs(val,d3,q3);
    val = vpa(val,5);
    
end

% Transformation Function
function val = Trans(t,d,a,r)

    val = [cos(t) -cos(r)*sin(t)  sin(r)*sin(t) a*cos(t);
          sin(t)  cos(r)*cos(t) -sin(r)*cos(t) a*sin(t);
              0          sin(r)         cos(r)       d ;
              0              0              0        1 ;];
end

% Function for forward kinematics
function val = For(DH)
    [n,m] = size(DH);
    I = eye(4);
    
    for i = 1:n
        I = I*Trans(DH(i,1),DH(i,2),DH(i,3),DH(i,4));
    end
    val = I;
end
