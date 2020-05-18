function dx=model2(t,x)
dx(1)=x(2);
dx(2)=x(1)-(x(1))^3;
dx=[dx(1); dx(2)];
end