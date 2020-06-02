rset = 0.5:.001:.7;
for i=1:size(rset,2)
    r = rset(i);
    syms x;
    val = vpasolve(itermap(x,r) == x, x, 0.5);
    der = double(abs(subs(diff(itermap(x,r),x), val)));
    if(der>=1)
        r
        der
        break
    end
end