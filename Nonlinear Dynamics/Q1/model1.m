function dx=model1(t,x)
dx(1)=x(2);
dx(2)=-x(1)-2*x(2);
dx=[dx(1); dx(2)];
end