close all;

rset = 0:0.05:1;
lambdaset = zeros(size(rset,2),1);
for i=1:size(rset,2)
    
    r = rset(i);
    
    n = 1000;
    x0 = .7;
    xi = x0;

    syms x;
    fdash = diff(itermap(x,r),x);
    sum = 0;

    for j=0:n-1
        sum = sum + log(double(abs(subs(fdash, x, xi))));
        xi = itermap(xi,r);
    end

    lambda = sum / n;
    
    lambdaset(i) = lambda;
end

plot(rset, [lambdaset, zeros(size(lambdaset))])

%     n = 100;
%     x0=.1;
%     syms x;
%     fn = itermap(x,r);
% 
%     for k=1:n-1
%         fn = itermap(fn,r);
%     end
% 
%     lambda = log(double(abs(subs(diff(fn, x),x0))))/n;

