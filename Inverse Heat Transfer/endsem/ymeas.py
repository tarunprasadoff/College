import numpy as np

ym = np.power(np.array([296.62,296.33,292.37,292.14,288.36,286.77]),-3) 

t = np.array([50,100,150,200,250,300])

Te = np.array([296.62,296.33,292.37,292.14,288.36,286.77])

def ypfunc(x):
    a = 2.2632 / (10**11)
    b = 303**(-3)
    return ( a * x + b )

print(ym)

yp = np.vectorize(ypfunc)(t)

print(yp)

Tp = np.power(yp,(-1/3))

d = np.array([ym - yp])

print(d)

print(np.matmul(t,np.transpose(d)))

print(Te)

print(Tp)

print(Te-Tp)
